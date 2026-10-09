// OpenDash-аналог для CYD: Ninebot G30 / ES4, BLE, только чтение телеметрии.
// Кнопка BOOT (GPIO0): короткое нажатие - следующая страница, долгое (>1.5 c) - переподключение.
// Страницы: 0 - панель со спидометром, 1 - информация, 2 - отладка.

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <NimBLEDevice.h>
#include <mbedtls/aes.h>
#include <mbedtls/sha1.h>
#include <cstring>

// ---------- Настройки ----------
#define BTN_PIN        0
#define TARGET_NAME    ""      // "" = первый найденный Ninebot; иначе точное имя, напр. "NBScooter1234"
#define POLL_INTERVAL  400     // мс между запросами; один зашифрованный запрос за раз
#define LINK_TIMEOUT   6000    // мс без ответов = считаем связь потерянной
#define GAUGE_MAX      30      // максимум шкалы спидометра, км/ч (делится на 3)

// Масштабы регистров. Если на странице отладки значения не сходятся - правь здесь.
#define SPEED_SCALE    0.1f    // рег. 0x26 -> км/ч
#define TRIP_SCALE     0.01f   // рег. 0xB9 -> км
#define TOTAL_SCALE    0.001f  // рег. 0x29 (u32) -> км
#define ETEMP_SCALE    0.1f    // рег. 0x3E -> °C
#define BMS_CUR_SCALE  0.01f   // А
#define BMS_VOLT_SCALE 0.01f   // В

// ---------- Палитра ----------
constexpr uint16_t C565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
constexpr uint16_t COL_BG     = C565(8, 10, 18);
constexpr uint16_t COL_PANEL  = C565(20, 26, 42);
constexpr uint16_t COL_TRACK  = C565(36, 44, 64);
constexpr uint16_t COL_ACCENT = C565(0, 200, 255);
constexpr uint16_t COL_TXT    = C565(240, 244, 252);
constexpr uint16_t COL_DIM    = C565(125, 138, 165);
constexpr uint16_t COL_GOOD   = C565(60, 225, 120);
constexpr uint16_t COL_WARN   = C565(255, 190, 40);
constexpr uint16_t COL_BAD    = C565(255, 70, 70);

void status(const String& s, uint16_t col = COL_WARN);

// ---------- Nordic UART ----------
static NimBLEUUID NUS_SVC("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID NUS_TX ("6e400002-b5a3-f393-e0a9-e50e24dcca9e");  // запись в самокат
static NimBLEUUID NUS_RX ("6e400003-b5a3-f393-e0a9-e50e24dcca9e");  // уведомления от самоката

// Адреса устройств в протоколе Ninebot
#define ADDR_APP 0x3E
#define ADDR_ESC 0x20
#define ADDR_BMS 0x22

enum Id : uint8_t { SPEED, TRIP, TOTAL_LO, TOTAL_HI, ETEMP, SOC, AMP, VOLT, BTEMP, SERIAL_NO, FW };
struct Item { uint8_t dev, reg, len; Id id; };

static const Item pollList[] = {
  {ADDR_ESC, 0x26, 2, SPEED},
  {ADDR_BMS, 0x32, 2, SOC},
  {ADDR_BMS, 0x33, 2, AMP},
  {ADDR_BMS, 0x34, 2, VOLT},
  {ADDR_BMS, 0x35, 2, BTEMP},
  {ADDR_ESC, 0xB9, 2, TRIP},
  {ADDR_ESC, 0x3E, 2, ETEMP},
  {ADDR_ESC, 0x29, 2, TOTAL_LO},
  {ADDR_ESC, 0x2A, 2, TOTAL_HI},
  {ADDR_ESC, 0x10, 14, SERIAL_NO},
  {ADDR_ESC, 0x1A, 2, FW},
};
static const int POLL_N = sizeof(pollList) / sizeof(pollList[0]);

struct Data {
  float speed = 0, tripKm = 0, totalKm = 0, escTemp = 0, battTemp = 0, volt = 0, amp = 0;
  int soc = -1;
  char serial[15] = "";
  uint16_t fw = 0;
  uint16_t valid = 0;
  uint16_t totalLow = 0, totalHigh = 0;
} d;

TFT_eSPI tft;
NimBLEClient* client = nullptr;
NimBLERemoteCharacteristic *txChr = nullptr, *rxChr = nullptr;
volatile bool connected = false;
volatile uint32_t lastRx = 0;
volatile uint32_t notifyCount = 0;
volatile uint32_t frameCount = 0;
volatile uint32_t checksumErrors = 0;
volatile uint32_t txCount = 0;
volatile uint32_t txFailures = 0;
uint8_t lastNotify[20] = {};
volatile uint8_t lastNotifyLen = 0;
volatile uint32_t txCounter = 0;
volatile uint32_t rxCounter = 0;
volatile bool pressPowerRequired = false;
String devName = "";
String dbg[7];
int page = 0;

// ---------- Протокол ----------
static uint8_t rxBuf[192];
static size_t rxLen = 0;
static size_t rxExpected = 0;

enum AuthState : uint8_t { AUTH_IDLE, AUTH_WAIT_INIT, AUTH_WAIT_PING, AUTH_WAIT_PAIR, AUTH_READY };
enum PendingTx : uint8_t { TX_NONE, TX_INIT, TX_PING, TX_PAIR };
static volatile AuthState authState = AUTH_IDLE;
static volatile PendingTx pendingTx = TX_NONE;
static uint8_t scooterName16[16] = {};
static uint8_t randomBleData[16] = {};
static uint8_t randomAppData[16] = {};
static uint8_t shaKey[16] = {};
static uint8_t scooterSerial[14] = {};
static uint32_t messageCounter = 0;
static uint32_t authLastSend = 0;
static uint32_t lastCommand = 0;

static inline uint16_t u16(const uint8_t* p) { return p[0] | (p[1] << 8); }
static inline int16_t  s16(const uint8_t* p) { return (int16_t)u16(p); }
static const uint8_t FW_DATA[16] = {
  0x97, 0xCF, 0xB8, 0x02, 0x84, 0x41, 0x43, 0xDE,
  0x56, 0x00, 0x2B, 0x3B, 0x34, 0x78, 0x0A, 0x5D
};

bool aesEncrypt(const uint8_t key[16], const uint8_t input[16], uint8_t output[16]) {
  mbedtls_aes_context context;
  mbedtls_aes_init(&context);
  int result = mbedtls_aes_setkey_enc(&context, key, 128);
  if (result == 0) result = mbedtls_aes_crypt_ecb(&context, MBEDTLS_AES_ENCRYPT, input, output);
  mbedtls_aes_free(&context);
  return result == 0;
}

void setShaKey(const uint8_t first[16], const uint8_t second[16]) {
  uint8_t input[32], digest[20];
  memcpy(input, first, 16);
  memcpy(input + 16, second, 16);
  mbedtls_sha1_context context;
  mbedtls_sha1_init(&context);
  mbedtls_sha1_starts_ret(&context);
  mbedtls_sha1_update_ret(&context, input, sizeof(input));
  mbedtls_sha1_finish_ret(&context, digest);
  mbedtls_sha1_free(&context);
  memcpy(shaKey, digest, 16);
}

void resetCrypto(const std::string& name) {
  memset(scooterName16, 0, sizeof(scooterName16));
  memset(randomBleData, 0, sizeof(randomBleData));
  memset(randomAppData, 0, sizeof(randomAppData));
  memset(scooterSerial, 0, sizeof(scooterSerial));
  size_t length = min(name.length(), sizeof(scooterName16));
  memcpy(scooterName16, name.data(), length);
  setShaKey(scooterName16, FW_DATA);
  messageCounter = 0;
  rxLen = 0;
  rxExpected = 0;
}

bool cryptoFirst(const uint8_t* input, uint8_t* output, size_t length) {
  uint8_t stream[16];
  if (!aesEncrypt(shaKey, FW_DATA, stream)) return false;
  for (size_t i = 0; i < length; i++) output[i] = input[i] ^ stream[i % 16];
  return true;
}

bool cryptoNext(const uint8_t* input, uint8_t* output, size_t length, uint32_t counter) {
  uint8_t seed[16] = {};
  seed[0] = 1;
  seed[1] = counter >> 24;
  seed[2] = counter >> 16;
  seed[3] = counter >> 8;
  seed[4] = counter;
  memcpy(seed + 5, randomBleData, 8);
  size_t offset = 0;
  uint8_t blockNumber = 0;
  while (offset < length) {
    seed[15] = ++blockNumber;
    uint8_t stream[16];
    if (!aesEncrypt(shaKey, seed, stream)) return false;
    size_t blockLength = min((size_t)16, length - offset);
    for (size_t i = 0; i < blockLength; i++) output[offset + i] = input[offset + i] ^ stream[i];
    offset += blockLength;
  }
  return true;
}

void crcFirst(const uint8_t* payload, size_t length, uint8_t output[2]) {
  uint32_t sum = 0;
  for (size_t i = 0; i < length; i++) sum += payload[i];
  uint16_t checksum = (uint16_t)~sum;
  output[0] = checksum;
  output[1] = checksum >> 8;
}

bool crcNext(const uint8_t* plain, size_t plainLength, uint32_t counter, uint8_t output[4]) {
  if (plainLength < 3) return false;
  const size_t payloadLength = plainLength - 3;
  uint8_t seed[16] = {};
  seed[0] = 89;
  seed[1] = counter >> 24;
  seed[2] = counter >> 16;
  seed[3] = counter >> 8;
  seed[4] = counter;
  memcpy(seed + 5, randomBleData, 8);
  seed[15] = payloadLength;
  uint8_t chain[16];
  if (!aesEncrypt(shaKey, seed, chain)) return false;

  uint8_t block[16] = {};
  block[0] = plain[0] ^ chain[0];
  block[1] = plain[1] ^ chain[1];
  block[2] = plain[2] ^ chain[2];
  memcpy(block + 3, chain + 3, 13);
  if (!aesEncrypt(shaKey, block, chain)) return false;
  size_t offset = 3, remaining = payloadLength;
  while (remaining) {
    size_t blockLength = min((size_t)16, remaining);
    memset(block, 0, sizeof(block));
    memcpy(block, plain + offset, blockLength);
    for (size_t i = 0; i < sizeof(block); i++) block[i] ^= chain[i];
    if (!aesEncrypt(shaKey, block, chain)) return false;
    offset += blockLength;
    remaining -= blockLength;
  }

  memset(seed, 0, sizeof(seed));
  seed[0] = 1;
  seed[1] = counter >> 24;
  seed[2] = counter >> 16;
  seed[3] = counter >> 8;
  seed[4] = counter;
  memcpy(seed + 5, randomBleData, 8);
  uint8_t mask[16];
  if (!aesEncrypt(shaKey, seed, mask)) return false;
  for (int i = 0; i < 4; i++) output[i] = mask[i] ^ chain[i];
  return true;
}

size_t encryptPacket(const uint8_t* plain, size_t plainLength, uint8_t* output, size_t capacity) {
  if (plainLength < 7 || capacity < plainLength + 6) return 0;
  const size_t payloadLength = plainLength - 3;
  const size_t encryptedLength = plainLength + 6;
  memcpy(output, plain, 3);
  if (messageCounter == 0) {
    uint8_t checksum[2];
    crcFirst(plain + 3, payloadLength, checksum);
    if (!cryptoFirst(plain + 3, output + 3, payloadLength)) return 0;
    output[payloadLength + 3] = 0;
    output[payloadLength + 4] = 0;
    output[payloadLength + 5] = checksum[0];
    output[payloadLength + 6] = checksum[1];
    output[payloadLength + 7] = 0;
    output[payloadLength + 8] = 0;
    messageCounter++;
  } else {
    messageCounter++;
    uint8_t checksum[4];
    if (!crcNext(plain, plainLength, messageCounter, checksum)) return 0;
    if (!cryptoNext(plain + 3, output + 3, payloadLength, messageCounter)) return 0;
    memcpy(output + payloadLength + 3, checksum, sizeof(checksum));
    output[payloadLength + 7] = messageCounter >> 8;
    output[payloadLength + 8] = messageCounter;
  }
  txCounter = messageCounter;
  return encryptedLength;
}

bool decryptPacket(const uint8_t* encrypted, size_t encryptedLength, uint8_t* plain,
                   size_t capacity, size_t& plainLength, uint32_t& counter) {
  if (encryptedLength < 13 || encrypted[0] != 0x5A || encrypted[1] != 0xA5) return false;
  const size_t payloadLength = encryptedLength - 9;
  plainLength = encryptedLength - 6;
  if (capacity < plainLength) return false;
  memcpy(plain, encrypted, 3);
  uint32_t nextCounter = messageCounter;
  if ((nextCounter & 0x8000) && !(encrypted[encryptedLength - 2] & 0x80)) nextCounter += 0x10000;
  nextCounter = (nextCounter & 0xFFFF0000UL) |
                ((uint32_t)encrypted[encryptedLength - 2] << 8) |
                encrypted[encryptedLength - 1];
  counter = nextCounter;

  if (nextCounter == 0) {
    if (!cryptoFirst(encrypted + 3, plain + 3, payloadLength)) return false;
    if (plainLength >= 37 && plain[2] == 0x1E && plain[3] == 0x21 &&
        plain[4] == ADDR_APP && plain[5] == 0x5B) {
      memcpy(randomBleData, plain + 7, 16);
      memcpy(scooterSerial, plain + 23, 14);
      setShaKey(scooterName16, randomBleData);
    }
  } else if (nextCounter > messageCounter) {
    if (!cryptoNext(encrypted + 3, plain + 3, payloadLength, nextCounter)) return false;
    if (plainLength >= 7 && plain[3] == 0x21 && plain[4] == ADDR_APP &&
        plain[5] == 0x5C && plain[6] == 0x01) {
      setShaKey(randomAppData, randomBleData);
    }
    messageCounter = nextCounter;
    rxCounter = nextCounter;
  } else {
    return false;
  }
  return true;
}

void pushDbg(const uint8_t* f, size_t n) {
  String s;
  for (size_t i = 0; i < n && i < 16; i++) { char b[4]; sprintf(b, "%02X", f[i]); s += b; }
  for (int i = 6; i > 0; i--) dbg[i] = dbg[i - 1];
  dbg[0] = s;
}

void handlePlainFrame(const uint8_t* f, size_t n) {
  if (n < 7) return;
  uint8_t src = f[3], dst = f[4], cmd = f[5], arg = f[6], len = f[2];
  frameCount++;
  pushDbg(f, n);

  if (src == 0x21 && dst == ADDR_APP && cmd == 0x5B && authState == AUTH_WAIT_INIT) {
    for (uint8_t& byte : randomAppData) byte = (uint8_t)esp_random();
    authState = AUTH_WAIT_PING;
    pressPowerRequired = true;
    pendingTx = TX_PING;
    authLastSend = millis();
    status("PRESS POWER", COL_WARN);
    Serial.println("INIT OK; sending PING, press the scooter power button when prompted");
    return;
  }
  if (src == 0x21 && dst == ADDR_APP && cmd == 0x5C && authState == AUTH_WAIT_PING) {
    authLastSend = millis();
    if (arg == 0x01) {
      pressPowerRequired = false;
      authState = AUTH_WAIT_PAIR;
      pendingTx = TX_PAIR;
      status("PAIRING", COL_ACCENT);
      Serial.println("POWER CONFIRMED; sending PAIR");
    } else {
      pressPowerRequired = true;
      status("PRESS POWER", COL_WARN);
      Serial.println("Ninebot requests physical power-button confirmation");
    }
    return;
  }
  if (src == 0x21 && dst == ADDR_APP && cmd == 0x5D && authState == AUTH_WAIT_PAIR) {
    authLastSend = millis();
    if (arg == 0x01) {
      authState = AUTH_READY;
      pressPowerRequired = false;
      lastRx = millis();
      status("READY", COL_GOOD);
      Serial.println("AUTH READY; read-only telemetry enabled");
    } else {
      pendingTx = TX_PAIR;
    }
    return;
  }

  if (dst != ADDR_APP || cmd != 0x04 || authState != AUTH_READY || n < 7 + len) return;
  const uint8_t* p = f + 7;
  Serial.printf("RX REG dev=%02X reg=%02X len=%u data=", src, arg, len);
  for (uint8_t i = 0; i < len; i++) Serial.printf("%02X", p[i]);
  Serial.println();
  for (int i = 0; i < POLL_N; i++) {
    const Item& it = pollList[i];
    if (it.dev != src || it.reg != arg || len < it.len) continue;
    switch (it.id) {
      case SPEED: d.speed = fabsf(s16(p) * SPEED_SCALE); break;
      case TRIP: d.tripKm = u16(p) * TRIP_SCALE; break;
      case TOTAL_LO:
        d.totalLow = u16(p);
        d.totalKm = (((uint32_t)d.totalHigh << 16) | d.totalLow) * TOTAL_SCALE;
        break;
      case TOTAL_HI:
        d.totalHigh = u16(p);
        d.totalKm = (((uint32_t)d.totalHigh << 16) | d.totalLow) * TOTAL_SCALE;
        break;
      case ETEMP: d.escTemp = s16(p) * ETEMP_SCALE; break;
      case SOC: d.soc = constrain((int)u16(p), 0, 100); break;
      case AMP: d.amp = s16(p) * BMS_CUR_SCALE; break;
      case VOLT: d.volt = u16(p) * BMS_VOLT_SCALE; break;
      case BTEMP: d.battTemp = ((int)p[0] + (int)p[1]) / 2 - 20; break;
      case SERIAL_NO: memcpy(d.serial, p, 14); d.serial[14] = 0; break;
      case FW: d.fw = u16(p); break;
    }
    d.valid |= (uint16_t)(1U << it.id);
  }
}

void processEncryptedFrame(const uint8_t* frame, size_t length) {
  uint8_t plain[192];
  size_t plainLength = 0;
  uint32_t counter = 0;
  if (!decryptPacket(frame, length, plain, sizeof(plain), plainLength, counter)) {
    checksumErrors++;
    return;
  }
  handlePlainFrame(plain, plainLength);
}

void feed(const uint8_t* data, size_t n) {
  if (n >= 2 && data[0] == 0x5A && data[1] == 0xA5) {
    rxLen = 0;
    rxExpected = 0;
  }
  if (rxLen + n > sizeof(rxBuf)) {
    rxLen = 0;
    rxExpected = 0;
    checksumErrors++;
    return;
  }
  memcpy(rxBuf + rxLen, data, n);
  rxLen += n;
  while (rxLen >= 3) {
    if (rxBuf[0] != 0x5A || rxBuf[1] != 0xA5) {
      rxLen = 0;
      rxExpected = 0;
      checksumErrors++;
      return;
    }
    if (!rxExpected) rxExpected = (size_t)rxBuf[2] + 13;
    if (rxExpected > sizeof(rxBuf)) {
      rxLen = 0;
      rxExpected = 0;
      checksumErrors++;
      return;
    }
    if (rxLen < rxExpected) return;
    processEncryptedFrame(rxBuf, rxExpected);
    size_t remaining = rxLen - rxExpected;
    if (remaining) memmove(rxBuf, rxBuf + rxExpected, remaining);
    rxLen = remaining;
    rxExpected = 0;
  }
}

bool writeEncrypted(const uint8_t* data, size_t length) {
  if (!txChr || !connected) return false;
  bool withResponse = txChr->canWrite();
  if (!withResponse && !txChr->canWriteNoResponse()) return false;
  size_t offset = 0;
  while (offset < length) {
    size_t chunk = min((size_t)20, length - offset);
    if (!txChr->writeValue(data + offset, chunk, withResponse)) {
      txFailures++;
      return false;
    }
    offset += chunk;
    delay(20);
  }
  txCount++;
  return true;
}

void sendPacket(uint8_t dst, uint8_t cmd, uint8_t arg, const uint8_t* data, uint8_t dataLen) {
  uint8_t plain[96] = {0x5A, 0xA5, dataLen, ADDR_APP, dst, cmd, arg};
  uint8_t encrypted[112];
  if (dataLen) memcpy(plain + 7, data, dataLen);
  size_t encryptedLength = encryptPacket(plain, 7 + dataLen, encrypted, sizeof(encrypted));
  if (!encryptedLength || !writeEncrypted(encrypted, encryptedLength)) {
    txFailures++;
    return;
  }
  lastCommand = millis();
}

void sendInit() {
  authState = AUTH_WAIT_INIT;
  authLastSend = millis();
  sendPacket(0x21, 0x5B, 0, nullptr, 0);
}

void sendPing() {
  authLastSend = millis();
  sendPacket(0x21, 0x5C, 0, randomAppData, sizeof(randomAppData));
}

void sendPair() {
  authState = AUTH_WAIT_PAIR;
  authLastSend = millis();
  sendPacket(0x21, 0x5D, 0, scooterSerial, sizeof(scooterSerial));
}

void sendRead(uint8_t dev, uint8_t reg, uint8_t n) {
  sendPacket(dev, 0x01, reg, &n, 1);
}

void serviceAuth() {
  if (!connected) return;
  uint32_t now = millis();
  if (authState == AUTH_WAIT_INIT && now - authLastSend > 5000) {
    if (client && client->isConnected()) client->disconnect();
  } else if (authState == AUTH_WAIT_PING && now - authLastSend > 1000) {
    pendingTx = TX_PING;
    authLastSend = now;
  } else if (authState == AUTH_WAIT_PAIR && now - authLastSend > 1500) {
    pendingTx = TX_PAIR;
    authLastSend = now;
  }
}

void servicePendingTx() {
  PendingTx pending = pendingTx;
  if (pending == TX_NONE || !connected) return;
  pendingTx = TX_NONE;
  if (pending == TX_INIT) sendInit();
  else if (pending == TX_PING) sendPing();
  else if (pending == TX_PAIR) sendPair();
}

// =====================================================================
//                               ИНТЕРФЕЙС
// =====================================================================

// Статус в шапке. status() можно звать из BLE-колбэка: он только запоминает текст,
// а рисует всегда основной цикл.
static char stText[40] = "";
static uint16_t stCol = COL_DIM;
static volatile bool stDirty = true;

void status(const String& s, uint16_t col) {
  if (s.equals(stText) && col == stCol) return;
  strlcpy(stText, s.c_str(), sizeof(stText));
  stCol = col;
  stDirty = true;
}

void drawStatus() {
  stDirty = false;
  tft.fillCircle(229, 11, 3, stCol);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(stCol, COL_PANEL);
  tft.setTextPadding(112);
  tft.drawString(stText, 220, 3, 2);
}

void statusNow(const String& s, uint16_t col = COL_WARN) {
  status(s, col);
  if (stDirty) drawStatus();
}

// Кэш текстовых полей: перерисовываем только то, что изменилось (нет мерцания).
#define SLOTS 48
static String cacheS[SLOTS];
static uint16_t cacheC[SLOTS];
static int uiMode = -1;
static int chipKey = -1;
static int litShown = 0;
static uint32_t barKey = 0xFFFFFFFF;

void clearCache() {
  for (int i = 0; i < SLOTS; i++) { cacheS[i] = "\x01"; cacheC[i] = 0; }
  chipKey = -1;
  litShown = 0;
  barKey = 0xFFFFFFFF;
}

void field(uint8_t slot, int x, int y, const String& s, int w, int font,
           uint16_t fg, uint8_t datum, uint16_t bg) {
  if (cacheC[slot] == fg && cacheS[slot] == s) return;
  cacheS[slot] = s;
  cacheC[slot] = fg;
  tft.setTextDatum(datum);
  tft.setTextColor(fg, bg);
  tft.setTextPadding(w);
  tft.drawString(s, x, y, font);
}

uint16_t lerp565(uint16_t a, uint16_t b, uint8_t t) {
  int ar = (a >> 11) & 31, ag = (a >> 5) & 63, ab = a & 31;
  int br = (b >> 11) & 31, bg = (b >> 5) & 63, bb = b & 31;
  int r = ar + (br - ar) * t / 255;
  int g = ag + (bg - ag) * t / 255;
  int bl = ab + (bb - ab) * t / 255;
  return (r << 11) | (g << 5) | bl;
}

void card(int x, int y, int w, int h) { tft.fillRoundRect(x, y, w, h, 6, COL_PANEL); }

void label(int x, int y, const char* s, uint8_t datum = TL_DATUM) {
  tft.setTextDatum(datum);
  tft.setTextColor(COL_DIM, COL_PANEL);
  tft.setTextPadding(0);
  tft.drawString(s, x, y, 1);
}

uint16_t tempColor(float t) { return t < 45 ? COL_GOOD : t < 60 ? COL_WARN : COL_BAD; }

// ---------- Шапка ----------
void drawHeaderStatic(bool dots) {
  tft.fillRect(0, 0, 240, 23, COL_PANEL);
  tft.drawFastHLine(0, 23, 240, COL_TRACK);
  tft.setTextPadding(0);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COL_ACCENT, COL_PANEL);
  tft.drawString("OPEN", 10, 3, 2);
  tft.setTextColor(COL_TXT, COL_PANEL);
  tft.drawString("DASH", 10 + tft.textWidth("OPEN", 2) + 4, 3, 2);
  if (dots) {
    for (int i = 0; i < 3; i++) tft.fillCircle(104 + i * 11, 11, i == page ? 3 : 2, i == page ? COL_ACCENT : COL_TRACK);
  }
}

// ---------- Спидометр ----------
#define GAUGE_SEGS 40
const int GCX = 120, GCY = 118, GR_O = 82, GR_I = 69;

uint16_t gaugeColor(int i) {
  static const uint16_t stops[4] = {C565(0, 200, 255), C565(60, 230, 120), C565(255, 200, 40), C565(255, 60, 60)};
  static const float pos[4] = {0.0f, 0.5f, 0.75f, 1.0f};
  float t = (float)i / (GAUGE_SEGS - 1);
  for (int k = 0; k < 3; k++)
    if (t <= pos[k + 1]) return lerp565(stops[k], stops[k + 1], (uint8_t)((t - pos[k]) / (pos[k + 1] - pos[k]) * 255));
  return stops[3];
}

void gaugeSeg(int i, uint16_t col) {
  const float DEG = 0.0174533f;
  float a0 = (-120.0f + 6.0f * i + 0.8f) * DEG;
  float a1 = (-120.0f + 6.0f * (i + 1) - 0.8f) * DEG;
  int ox0 = lroundf(GCX + GR_O * sinf(a0)), oy0 = lroundf(GCY - GR_O * cosf(a0));
  int ox1 = lroundf(GCX + GR_O * sinf(a1)), oy1 = lroundf(GCY - GR_O * cosf(a1));
  int ix0 = lroundf(GCX + GR_I * sinf(a0)), iy0 = lroundf(GCY - GR_I * cosf(a0));
  int ix1 = lroundf(GCX + GR_I * sinf(a1)), iy1 = lroundf(GCY - GR_I * cosf(a1));
  tft.fillTriangle(ox0, oy0, ox1, oy1, ix0, iy0, col);
  tft.fillTriangle(ox1, oy1, ix1, iy1, ix0, iy0, col);
}

void updateGauge(float speed) {
  int lit = constrain((int)lroundf(speed / GAUGE_MAX * GAUGE_SEGS), 0, GAUGE_SEGS);
  if (lit == litShown) return;
  if (lit > litShown) { for (int i = litShown; i < lit; i++) gaugeSeg(i, gaugeColor(i)); }
  else                { for (int i = lit; i < litShown; i++) gaugeSeg(i, COL_TRACK); }
  litShown = lit;
}

// ---------- Страница 0: панель ----------
void dashStatic() {
  for (int i = 0; i < GAUGE_SEGS; i++) gaugeSeg(i, COL_TRACK);
  litShown = 0;

  tft.setTextPadding(0);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COL_DIM, COL_BG);
  for (int k = 0; k <= 3; k++) {
    float a = (-120.0f + 80.0f * k) * 0.0174533f;
    char b[6];
    snprintf(b, sizeof(b), "%d", GAUGE_MAX * k / 3);
    tft.drawString(b, lroundf(GCX + 57 * sinf(a)), lroundf(GCY - 57 * cosf(a)), 1);
  }
  tft.setTextColor(COL_ACCENT, COL_BG);
  tft.drawString("km/h", GCX, GCY + 20, 2);
  card(8, 207, 108, 50);
  card(124, 207, 108, 50);
  card(8, 263, 108, 50);
  card(124, 263, 108, 50);
  label(16, 211, "BATTERY");
  label(132, 211, "POWER");
  label(16, 267, "TRIP");
  label(132, 267, "ESC / BAT C");
}

void drawBar(int lit, uint16_t col) {
  for (int i = 0; i < 10; i++)
    tft.fillRoundRect(16 + (i * 92) / 10, 243, 7, 6, 2, i < lit ? col : COL_TRACK);
}

void dashDynamic() {
  const bool hasSpeed = d.valid & (1U << SPEED);
  const bool hasSoc = d.valid & (1U << SOC);
  const bool hasV = d.valid & (1U << VOLT);
  const bool hasA = d.valid & (1U << AMP);
  const bool hasTotal = (d.valid & ((1U << TOTAL_LO) | (1U << TOTAL_HI))) == ((1U << TOTAL_LO) | (1U << TOTAL_HI));

  updateGauge(hasSpeed ? d.speed : 0);
  field(0, GCX, GCY - 8, hasSpeed ? String(d.speed, 1) : String("--"), 120, 7, COL_TXT, MC_DATUM, COL_BG);
  field(1, GCX, 188, hasTotal ? "ODO " + String(d.totalKm, 1) + " km" : String("ODO -- km"), 150, 2, COL_DIM, MC_DATUM, COL_BG);

  // Батарея
  uint16_t bc = !hasSoc ? COL_DIM : d.soc > 50 ? COL_GOOD : d.soc > 20 ? COL_WARN : COL_BAD;
  field(2, 16, 225, hasSoc ? String(d.soc) + "%" : String("--"), 90, 4, bc, TL_DATUM, COL_PANEL);
  int lit = hasSoc ? (d.soc + 5) / 10 : 0;
  uint32_t key = ((uint32_t)lit << 16) | bc;
  if (key != barKey) { barKey = key; drawBar(lit, bc); }

  // Мощность
  float w = d.volt * d.amp;
  uint16_t pc = w < -5 ? COL_GOOD : w > 600 ? COL_WARN : COL_TXT;
  field(3, 132, 225, hasV && hasA ? String((int)w) + " W" : String("-- W"), 96, 4, pc, TL_DATUM, COL_PANEL);
  field(4, 132, 246, hasV && hasA ? String(d.volt, 1) + "V " + String(d.amp, 1) + "A" : String("--"), 96, 1, COL_DIM, TL_DATUM, COL_PANEL);

  // Поездка
  field(5, 16, 281, d.valid & (1U << TRIP) ? String(d.tripKm, 2) : String("--"), 90, 4, COL_TXT, TL_DATUM, COL_PANEL);

  // Температуры
  bool he = d.valid & (1U << ETEMP), hb = d.valid & (1U << BTEMP);
  field(6, 132, 281, he && hb ? String((int)lroundf(d.escTemp)) + " / " + String((int)lroundf(d.battTemp)) :
        he ? String((int)lroundf(d.escTemp)) + " / --" :
        hb ? String("-- / ") + String((int)lroundf(d.battTemp)) : String("-- / --"),
        96, 2, he ? tempColor(d.escTemp) : hb ? tempColor(d.battTemp) : COL_DIM, TL_DATUM, COL_PANEL);
}

// ---------- Страница 1: информация ----------
static void tileXY(int i, int& x, int& y) { x = 6; y = 28 + i * 36; }

void infoStatic() {
  const char* L[8] = {"DEVICE", "SERIAL", "FIRMWARE", "ODOMETER (km)", "TRIP (km)", "VOLTAGE (V)", "CURRENT (A)", "ESC / BAT TEMP"};
  for (int i = 0; i < 8; i++) {
    int x, y;
    tileXY(i, x, y);
    card(x, y, 228, 32);
    label(x + 7, y + 11, L[i]);
  }
}

void infoDynamic() {
  const bool hasTotal = (d.valid & ((1U << TOTAL_LO) | (1U << TOTAL_HI))) == ((1U << TOTAL_LO) | (1U << TOTAL_HI));
  char b[16];
  snprintf(b, sizeof(b), "%X.%02X", d.fw >> 8, d.fw & 0xFF);
  String v[8] = {
    devName,
    d.serial[0] ? String(d.serial) : String("--"),
    d.fw ? String(b) : String("--"),
    hasTotal ? String(d.totalKm, 1) : String("--"),
    d.valid & (1U << TRIP) ? String(d.tripKm, 2) : String("--"),
    d.valid & (1U << VOLT) ? String(d.volt, 2) : String("--"),
    d.valid & (1U << AMP) ? String(d.amp, 2) : String("--"),
    String(d.valid & (1U << ETEMP) ? String((int)lroundf(d.escTemp)) : String("--")) + " / " +
      (d.valid & (1U << BTEMP) ? String((int)lroundf(d.battTemp)) : String("--")),
  };
  for (int i = 0; i < 8; i++) {
    int x, y;
    tileXY(i, x, y);
    field(10 + i, x + 116, y + 9, v[i], 104, 2, COL_TXT, TL_DATUM, COL_PANEL);
  }
}

// ---------- Страница 2: отладка ----------
String hexLine(const uint8_t* p, int n) {
  String s;
  for (int i = 0; i < n; i++) { char b[4]; snprintf(b, sizeof(b), "%02X ", p[i]); s += b; }
  return s;
}

void debugStatic() {
  const char* L[4] = {"RX", "TX", "FRAMES", "BAD"};
  for (int i = 0; i < 4; i++) {
    int x = 8 + (i % 2) * 116;
    int y = 30 + (i / 2) * 42;
    card(x, y, 108, 36);
    label(x + 8, y + 5, L[i]);
  }
  card(8, 116, 224, 48);
  label(16, 120, "LAST RX BYTES");
  card(8, 172, 224, 140);
  label(16, 176, "VALID FRAMES");
}

void debugDynamic() {
  uint32_t v[4] = {notifyCount, txCount, frameCount, checksumErrors};
  for (int i = 0; i < 4; i++) {
    int x = 16 + (i % 2) * 116;
    int y = 44 + (i / 2) * 42;
    field(20 + i, x, y, String(v[i]), 90, 2, i == 3 && v[i] ? COL_BAD : COL_TXT, TL_DATUM, COL_PANEL);
  }
  int n = lastNotifyLen;
  int n1 = min(n, 14), n2 = n > 14 ? n - 14 : 0;
  field(24, 16, 136, hexLine(lastNotify, n1), 208, 1, COL_ACCENT, TL_DATUM, COL_PANEL);
  field(25, 16, 150, hexLine(lastNotify + 14, n2), 208, 1, COL_ACCENT, TL_DATUM, COL_PANEL);
  for (int i = 0; i < 7; i++) field(26 + i, 16, 194 + i * 16, dbg[i], 208, 1, COL_GOOD, TL_DATUM, COL_PANEL);
}

// ---------- Экран подключения ----------
void connectDynamic() {
  int phase;
  if (!connected) phase = 0;
  else if (authState == AUTH_WAIT_INIT) phase = 1;
  else if (authState == AUTH_WAIT_PING) phase = 2;
  else phase = 3;

  uint32_t t = millis();
  bool power = pressPowerRequired && phase == 2;
  bool blink = power && ((t / 400) & 1);
  uint16_t base = power ? COL_WARN : COL_ACCENT;

  // Крутилка
  int head = (t / 90) % 12;
  for (int i = 0; i < 12; i++) {
    int dd = (head - i + 12) % 12;
    uint8_t alpha = dd < 7 ? 255 - dd * 36 : 20;
    float a = i * 30 * 0.0174533f;
    tft.fillCircle(120 + lroundf(34 * sinf(a)), 92 - lroundf(34 * cosf(a)), 5, lerp565(COL_BG, base, alpha));
  }

  static const char* titles[4] = {"SEARCHING", "CONNECTING", "PRESS POWER", "PAIRING"};
  static const char* subs[4] = {"Turn on the scooter and stay close", "Starting secure handshake",
                                "Press the scooter power button once", "Almost there..."};
  field(40, 120, 148, titles[phase], 224, 4, power ? COL_WARN : COL_TXT, MC_DATUM, COL_BG);
  field(41, 120, 180, subs[phase], 224, 2, COL_DIM, MC_DATUM, COL_BG);

  int key = phase * 2 + (blink ? 1 : 0);
  if (key != chipKey) {
    chipKey = key;
    static const char* names[4] = {"SCAN", "LINK", "POWER", "PAIR"};
    for (int i = 0; i < 4; i++) {
      int x = 8 + i * 57;
      uint16_t fill, fg;
      if (i < phase)       { fill = C565(0, 70, 96); fg = COL_TXT; }
      else if (i == phase) { fill = blink ? COL_TXT : base; fg = COL_BG; }
      else                 { fill = COL_PANEL; fg = COL_DIM; }
      tft.fillRoundRect(x, 220, 52, 26, 6, fill);
      tft.setTextDatum(MC_DATUM);
      tft.setTextColor(fg, fill);
      tft.setTextPadding(0);
      tft.drawString(names[i], x + 26, 227, 1);
    }
  }
}

// ---------- Диспетчер экрана ----------
void drawUi() {
  bool live = connected && authState == AUTH_READY;
  int mode = live ? 1 + page : 0;
  if (mode != uiMode) {
    uiMode = mode;
    clearCache();
    tft.fillScreen(COL_BG);
    drawHeaderStatic(mode > 0);
    stDirty = true;
    if (mode == 1) dashStatic();
    else if (mode == 2) infoStatic();
    else if (mode == 3) debugStatic();
  }
  if (stDirty) drawStatus();
  if (mode == 0) connectDynamic();
  else if (mode == 1) dashDynamic();
  else if (mode == 2) infoDynamic();
  else debugDynamic();
}

// ---------- BLE ----------
class ClientCB : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient*) override {
    connected = false;
    txChr = nullptr;
    rxChr = nullptr;
    authState = AUTH_IDLE;
    pendingTx = TX_NONE;
    pressPowerRequired = false;
  }
} clientCB;

void onNotify(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
  notifyCount++;
  lastRx = millis();
  lastNotifyLen = min(len, sizeof(lastNotify));
  memcpy(lastNotify, data, lastNotifyLen);
  feed(data, len);
}

bool nameMatches(const std::string& n) {
  if (n.empty()) return false;
  if (strlen(TARGET_NAME)) return n == TARGET_NAME;
  String s = String(n.c_str()); s.toLowerCase();
  return s.startsWith("nb") || s.indexOf("ninebot") >= 0 || s.indexOf("segway") >= 0 || s.indexOf("es4") >= 0;
}

bool scanAndConnect() {
  statusNow("scan...", COL_WARN);
  drawUi();
  NimBLEScan* sc = NimBLEDevice::getScan();
  sc->setActiveScan(true);
  NimBLEScanResults r = sc->start(4, false);
  NimBLEAddress best; bool found = false; int bestRssi = -999; String bestName;
  for (auto it = r.begin(); it != r.end(); ++it) {
    NimBLEAdvertisedDevice* a = *it;
    if (!nameMatches(a->getName())) continue;
    if (a->getRSSI() > bestRssi) { bestRssi = a->getRSSI(); best = a->getAddress(); bestName = a->getName().c_str(); found = true; }
  }
  sc->clearResults();
  if (!found) { statusNow("not found", COL_BAD); return false; }

  statusNow("connect " + bestName, COL_WARN);
  if (!client) {
    client = NimBLEDevice::createClient();
    client->setClientCallbacks(&clientCB, false);
    client->setConnectTimeout(8);
  }
  if (!client->connect(best)) { statusNow("conn fail", COL_BAD); return false; }
  NimBLERemoteService* s = client->getService(NUS_SVC);
  if (!s) { client->disconnect(); statusNow("no NUS", COL_BAD); return false; }
  txChr = s->getCharacteristic(NUS_TX);
  rxChr = s->getCharacteristic(NUS_RX);
  if (!txChr || !rxChr || !rxChr->subscribe(true, onNotify)) { client->disconnect(); statusNow("chr fail", COL_BAD); return false; }
  devName = bestName;
  resetCrypto(std::string(bestName.c_str()));
  rxLen = 0; rxExpected = 0; lastRx = millis(); connected = true; d.valid = 0;
  txCounter = 0; rxCounter = 0; pressPowerRequired = false;
  authState = AUTH_WAIT_INIT; pendingTx = TX_INIT; authLastSend = millis();
  d.totalLow = 0; d.totalHigh = 0; d.tripKm = 0;
  memset(d.serial, 0, sizeof(d.serial)); d.fw = 0;
  status("AUTH INIT", COL_ACCENT);
  return true;
}

// ---------- Основной цикл ----------
void setup() {
  Serial.begin(115200);
  pinMode(BTN_PIN, INPUT_PULLUP);
  tft.init();
  tft.setRotation(0);  // портретная ориентация, 240x320
  tft.fillScreen(COL_BG);
  clearCache();
  drawUi();
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  Serial.println("Ninebot G30/ES4 dashboard; encrypted handshake required");
}

void handleButton() {
  static uint32_t downAt = 0;
  static bool wasDown = false;
  bool down = digitalRead(BTN_PIN) == LOW;
  if (down && !wasDown) downAt = millis();
  if (!down && wasDown) {
    uint32_t dt = millis() - downAt;
    if (dt > 1500) { if (client && client->isConnected()) client->disconnect(); connected = false; }
    else if (dt > 40) { page = (page + 1) % 3; }
  }
  wasDown = down;
}

void loop() {
  static uint32_t lastPoll = 0, lastDraw = 0;
  static int idx = 0;
  handleButton();

  if (!connected) {
    if (!scanAndConnect()) { drawUi(); delay(1500); return; }
  }

  uint32_t now = millis();
  if (now - lastRx > LINK_TIMEOUT) {
    if (client && client->isConnected()) client->disconnect();
    connected = false;
    status("link lost", COL_BAD);
    drawUi();
    return;
  }

  servicePendingTx();
  serviceAuth();

  if (authState == AUTH_READY && now - lastPoll >= POLL_INTERVAL) {
    lastPoll = now;
    const Item& it = pollList[idx];
    idx = (idx + 1) % POLL_N;
    bool slowDone = (it.id == SERIAL_NO && d.serial[0]) || (it.id == FW && d.fw);
    if (!slowDone) sendRead(it.dev, it.reg, it.len);
  }

  if (now - lastDraw >= 90) {
    lastDraw = now;
    if (authState == AUTH_READY) status("LIVE " + devName, COL_GOOD);
    else if (authState == AUTH_WAIT_INIT) status("AUTH INIT", COL_ACCENT);
    else if (authState == AUTH_WAIT_PING) status("PRESS POWER", COL_WARN);
    else if (authState == AUTH_WAIT_PAIR) status("PAIRING", COL_ACCENT);
    else status("NO LINK", COL_BAD);
    drawUi();
  }
}
