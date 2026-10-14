// OpenDash-аналог для CYD: Ninebot G30 / ES4, BLE, телеметрия.
// По умолчанию прошивка только читает. Редактор настроек SHFW (Настройки -> SHFW) пишет в регистры ESC
// только после явного разрешения на экране SHFW (флаг не сохраняется, после перезапуска снова выключен).
// Управление: сенсорный экран (свайп - страницы, шестерёнка - настройки) и кнопка BOOT (GPIO0):
// короткое нажатие - следующая страница / назад, долгое (>1.5 c) - переподключение.
// Страницы: 0 - панель со спидометром, 1 - информация, 2 - отладка.

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <NimBLEDevice.h>
#include <mbedtls/aes.h>
#include <mbedtls/sha1.h>
#include <cstring>
#include <SPI.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <esp_system.h>
#include <XPT2046_Touchscreen.h>
#include <Adafruit_NeoPixel.h>
#include "ru_fonts.h"   // кириллица для TFT_eSPI (генерируется tools/gen_ru_font.py)
#include "shfw_table.h" // параметры SHFW (генерируется tools/gen_shfw_table.py из tools/shfw_registers.csv)

// ---------- Настройки ----------
#define BTN_PIN        0
#define TARGET_NAME    ""      // "" = первый найденный Ninebot; иначе точное имя, напр. "NBScooter1234"
#define POLL_INTERVAL  400     // мс между запросами; один зашифрованный запрос за раз
#define LINK_TIMEOUT   6000    // мс без ответов = считаем связь потерянной
#define GAUGE_MAX      30      // шкала спидометра по умолчанию, км/ч (дальше меняется в настройках на экране)

// Масштабы регистров. Если на странице отладки значения не сходятся - правь здесь.
#define SPEED_SCALE    0.1f    // рег. 0x26 -> км/ч
#define TRIP_SCALE     0.01f   // рег. 0xB9 -> км
#define TOTAL_SCALE    0.001f  // рег. 0x29 (u32) -> км
#define ETEMP_SCALE    0.1f    // рег. 0x3E -> °C
#define BMS_CUR_SCALE  0.01f   // А
#define BMS_VOLT_SCALE 0.01f   // В
#define BRAKE_REGEN_CURRENT_A 1.0f
#define BRAKE_HOLD_MS 700
#define FAST_TELEMETRY_POLL_MS 200
#define LED_MAX_COUNT 120

static const uint8_t LED_PINS[] = {4, 16, 17, 22, 26, 27};
static const uint8_t LED_PIN_COUNT = sizeof(LED_PINS) / sizeof(LED_PINS[0]);

// ---------- Палитра (меняется темой из настроек, см. applyTheme) ----------
constexpr uint16_t C565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
uint16_t COL_BG     = C565(8, 10, 18);
uint16_t COL_PANEL  = C565(20, 26, 42);
uint16_t COL_TRACK  = C565(36, 44, 64);
uint16_t COL_ACCENT = C565(0, 200, 255);
uint16_t COL_TXT    = C565(240, 244, 252);
uint16_t COL_DIM    = C565(125, 138, 165);
uint16_t COL_GOOD   = C565(60, 225, 120);
uint16_t COL_WARN   = C565(255, 190, 40);
uint16_t COL_BAD    = C565(255, 70, 70);

void status(const String& s, uint16_t col = COL_WARN);

// ---------- Nordic UART ----------
static NimBLEUUID NUS_SVC("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID NUS_TX ("6e400002-b5a3-f393-e0a9-e50e24dcca9e");  // запись в самокат
static NimBLEUUID NUS_RX ("6e400003-b5a3-f393-e0a9-e50e24dcca9e");  // уведомления от самоката

// Адреса устройств в протоколе Ninebot
#define ADDR_APP 0x3E
#define ADDR_ESC 0x20
#define ADDR_BMS 0x22

enum Id : uint8_t { SPEED, TRIP, TOTAL_LO, TOTAL_HI, ETEMP, SOC, AMP, VOLT, BTEMP, SERIAL_NO, FW, RIDE_MODE };
struct Item { uint8_t dev, reg, len; Id id; };

static const Item pollList[] = {
  {ADDR_BMS, 0x32, 2, SOC},
  {ADDR_BMS, 0x34, 2, VOLT},
  {ADDR_BMS, 0x35, 2, BTEMP},
  {ADDR_ESC, 0xB9, 2, TRIP},
  {ADDR_ESC, 0x3E, 2, ETEMP},
  {ADDR_ESC, 0x29, 2, TOTAL_LO},
  {ADDR_ESC, 0x2A, 2, TOTAL_HI},
  {ADDR_ESC, 0x10, 14, SERIAL_NO},
  {ADDR_ESC, 0x1A, 2, FW},
  {ADDR_ESC, 0x75, 2, RIDE_MODE},
};
static const int POLL_N = sizeof(pollList) / sizeof(pollList[0]);

struct Data {
  float speed = 0, tripKm = 0, totalKm = 0, escTemp = 0, battTemp = 0, volt = 0, amp = 0;
  int soc = -1;
  char serial[15] = "";
  uint16_t fw = 0;
  uint16_t valid = 0;
  uint16_t totalLow = 0, totalHigh = 0;
  uint16_t rideMode = 0;
} d;

// Ответ на чтение регистра ESC для редактора SHFW (заполняется в handlePlainFrame, ждёт shfwReadReg).
struct ShfwRx { volatile bool pending, ok; volatile uint8_t reg, len; uint8_t data[SHFW_MAX_XFER]; } shfwRx;
// Ответ на запрос версии SHFW: 5A A5 00 3E 20 39 00 68 FF (dst=ESC, cmd=0x39, arg=0). Формат ответа сохраняется как есть.
struct ShfwVerRx { volatile bool pending, ok; volatile uint8_t len; uint8_t data[SHFW_VER_MAX]; } shfwVerRx;

enum LedEffect : uint8_t { LED_OFF, LED_SOLID, LED_PULSE, LED_RAINBOW, LED_EFFECT_COUNT };
struct LedSettings {
  uint8_t enabled = 0, pinIndex = 3, count = 30;
  uint8_t segmentCount[3] = {10, 10, 10};
  uint8_t effect[3] = {LED_SOLID, LED_PULSE, LED_RAINBOW};
  uint8_t color[3] = {0, 7, 1};
} ledCfg;

static const uint32_t LED_COLORS[] = {
  0x00C8FF, 0x4678FF, 0xAA5AFF, 0xFF50AA,
  0xFF4646, 0xFF8C1E, 0xFFD228, 0x3CE178
};
static const char* const LED_EFFECT_NAMES[] = {"OFF", "SOLID", "PULSE", "RAINBOW"};
static const char* const LED_EFFECT_NAMES_RU[] = {"ВЫКЛ", "ЦВЕТ", "ПУЛЬС", "РАДУГА"};

TFT_eSPI tft;
Adafruit_NeoPixel ledStrip(30, LED_PINS[3], NEO_GRB + NEO_KHZ800);
static bool ledReady = false;
static uint32_t brakeUntil = 0;
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
      Serial.println("AUTH READY; telemetry enabled (SHFW writes stay locked until unlocked on screen)");
    } else {
      pendingTx = TX_PAIR;
    }
    return;
  }

  if (shfwVerRx.pending && src == ADDR_ESC && dst == ADDR_APP && authState == AUTH_READY && n >= (size_t)7 + len) {
    if (cmd == 0x39 || arg == 0x39) {
      uint8_t k = len > SHFW_VER_MAX ? SHFW_VER_MAX : len;
      memcpy(shfwVerRx.data, f + 7, k);
      shfwVerRx.len = k;
      shfwVerRx.ok = true;
      shfwVerRx.pending = false;
      Serial.printf("SHFW VER cmd=%02X arg=%02X len=%u\n", cmd, arg, len);
    } else if (cmd != 0x04) {
      Serial.printf("SHFW VER? unmatched cmd=%02X arg=%02X len=%u\n", cmd, arg, len);
    }
  }
  if (shfwRx.pending && src == ADDR_ESC && dst == ADDR_APP && authState == AUTH_READY && n >= (size_t)7 + len) {
    if ((cmd == 0x34 || cmd == 0x31) && arg == shfwRx.reg && shfwRx.len >= 1 && shfwRx.len <= SHFW_MAX_XFER && len >= shfwRx.len) {
      memcpy(shfwRx.data, f + 7, shfwRx.len);          // ответ на чтение конфига SHFW: cmd=0x34 (по логам с самоката)
      shfwRx.ok = true;
      shfwRx.pending = false;
    } else if (cmd != 0x04) {
      Serial.printf("SHFW CFG? unmatched cmd=%02X arg=%02X len=%u (want arg=%02X)\n", cmd, arg, len, shfwRx.reg);
    }
  }
  if (dst != ADDR_APP || cmd != 0x04 || authState != AUTH_READY || n < 7 + len) return;
  const uint8_t* p = f + 7;
  if (src == ADDR_ESC && arg == 0x26 && len >= 2) {
    d.speed = fabsf(s16(p) * SPEED_SCALE);
    d.valid |= (uint16_t)(1U << SPEED);
    return;
  }
  if (src == ADDR_BMS && arg == 0x33 && len >= 2) {
    d.amp = s16(p) * BMS_CUR_SCALE;
    d.valid |= (uint16_t)(1U << AMP);
    if ((d.valid & (1U << SPEED)) && d.speed > 3.0f &&
        d.amp <= -BRAKE_REGEN_CURRENT_A) {
      brakeUntil = millis() + BRAKE_HOLD_MS;
    }
    return;
  }
  Serial.printf("RX REG dev=%02X reg=%02X len=%u data=", src, arg, len);
  for (uint8_t i = 0; i < len; i++) Serial.printf("%02X", p[i]);
  Serial.println();
  for (int i = 0; i < POLL_N; i++) {
    const Item& it = pollList[i];
    if (it.dev != src || it.reg != arg || len < it.len) continue;
    switch (it.id) {
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
      case VOLT: d.volt = u16(p) * BMS_VOLT_SCALE; break;
      case BTEMP: d.battTemp = ((int)p[0] + (int)p[1]) / 2 - 20; break;
      case SERIAL_NO: memcpy(d.serial, p, 14); d.serial[14] = 0; break;
      case FW: d.fw = u16(p); break;
      case RIDE_MODE: d.rideMode = u16(p); break;
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

// =====================================================================
//          SHFW: ЧТЕНИЕ И ЗАПИСЬ РЕГИСТРОВ ESC (редактор настроек)
// =====================================================================
// Чтение конфига - команда 0x31, запись - 0x32 (см. CMD_CFG_*). Запись выключена, пока её не разрешат на экране SHFW
// (флаг живёт только до перезапуска). После записи регистр читается обратно и сравнивается.
// Конфиг SHFW - отдельные команды (не обычные регистры Ninebot 0x01/0x03):
//   0x31 - чтение конфига: arg = адрес в СЛОВАХ, payload = [длина в БАЙТАХ, не больше 0x50];
//   0x32 - запись конфига: arg = адрес в словах, payload = данные (не больше 0x50 байт);
//   ответ на 0x31 приходит с cmd=0x34, arg = тот же адрес, payload = данные (см. handlePlainFrame).
#define CMD_CFG_READ  0x31
#define CMD_CFG_WRITE 0x32
static void shfwSendCfgRead(uint8_t reg, uint8_t bytes) { sendPacket(ADDR_ESC, CMD_CFG_READ, reg, &bytes, 1); }
static bool shfwEnabled = false;
static uint16_t shfwWord[256];     // кэш прочитанных слов регистров
static uint8_t shfwHave[256];      // сколько байт регистра есть в кэше (0/1/2)
static int8_t shfwDetected = -1;   // -1 не проверяли, 0 SHFW не найдена, 1 найдена
static uint8_t shfwProfile = 0;

static bool shfwLive() { return connected && authState == AUTH_READY; }

// Блокирующее чтение регистра ESC (до ~0,9 с). Результат попадает в кэш.
static bool shfwReadReg(uint8_t reg, uint8_t len, uint16_t& word) {
  for (int attempt = 0; attempt < 2; attempt++) {
    if (!shfwLive()) return false;
    shfwRx.reg = reg; shfwRx.len = len; shfwRx.ok = false; shfwRx.pending = true;
    shfwSendCfgRead(reg, len);
    uint32_t t0 = millis();
    while (shfwRx.pending && millis() - t0 < 450) delay(3);
    shfwRx.pending = false;
    if (shfwRx.ok) {
      word = shfwWordFromBytes(shfwRx.data, len);
      if (len >= 2) { shfwWord[reg] = word; shfwHave[reg] = 2; }
      else {
        shfwWord[reg] = (uint16_t)((shfwHave[reg] >= 2 ? (shfwWord[reg] & 0xFF00) : 0) | (word & 0xFF));
        if (!shfwHave[reg]) shfwHave[reg] = 1;
      }
      return true;
    }
  }
  return false;
}

static bool shfwEnsure(const ShfwItem& it) {
  if (shfwHave[it.reg] >= it.len) return true;
  uint16_t w;
  return shfwReadReg(it.reg, it.len, w);
}

static bool shfwCached(const ShfwItem& it, int32_t& val) {
  if (shfwHave[it.reg] < it.len) return false;
  val = shfwGet(it, shfwWord[it.reg]);
  return true;
}


// ---------- версия SHFW и блочный обмен (адрес в СЛОВАХ, длина в БАЙТАХ, не больше 0x50 за раз) ----------
static uint8_t shfwVer[SHFW_VER_MAX];
static uint8_t shfwVerLen = 0;

static bool shfwReadVersion() {
  for (int attempt = 0; attempt < 2; attempt++) {
    if (!shfwLive()) return false;
    shfwVerRx.ok = false; shfwVerRx.pending = true;
    sendPacket(ADDR_ESC, 0x39, 0x00, nullptr, 0);          // 5A A5 00 3E 20 39 00 68 FF
    uint32_t t0 = millis();
    while (shfwVerRx.pending && millis() - t0 < 600) delay(3);
    shfwVerRx.pending = false;
    if (shfwVerRx.ok && shfwVerRx.len) {
      memcpy(shfwVer, shfwVerRx.data, shfwVerRx.len);
      shfwVerLen = shfwVerRx.len;
      return true;
    }
  }
  shfwVerLen = 0;
  return false;
}

static bool shfwReadBlock(uint8_t reg, uint8_t bytes, uint8_t* out) {
  if (!bytes || bytes > SHFW_MAX_XFER) return false;
  for (int attempt = 0; attempt < 2; attempt++) {
    if (!shfwLive()) return false;
    shfwRx.reg = reg; shfwRx.len = bytes; shfwRx.ok = false; shfwRx.pending = true;
    shfwSendCfgRead(reg, bytes);
    uint32_t t0 = millis();
    while (shfwRx.pending && millis() - t0 < 700) delay(3);
    shfwRx.pending = false;
    if (shfwRx.ok) { memcpy(out, shfwRx.data, bytes); return true; }
  }
  return false;
}

static bool shfwWriteBlock(uint8_t reg, const uint8_t* data, uint8_t bytes) {
  if (!bytes || bytes > SHFW_MAX_XFER || !shfwLive()) return false;
  uint32_t f0 = txFailures;
  sendPacket(ADDR_ESC, CMD_CFG_WRITE, reg, data, bytes);
  delay(150);
  return txFailures == f0;
}

// Признак SHFW: ответ на запрос версии (0x39) ИЛИ регистр 0xB2 (активный профиль 1-3 по таблице).
// Если ничего не сработало, а пользователь включил «принудительно», считаем SHFW найденной.
static char shfwDiag[64] = "";
static bool shfwForce = false, shfwForced = false;

static bool shfwDetect() {
  shfwDetected = 0;
  shfwProfile = 0;
  shfwForced = false;
  bool verOk = shfwReadVersion();
  uint16_t w = 0;
  bool b2Ok = shfwReadReg(0xB2, 2, w);
  uint8_t pr = (uint8_t)(w & 0xFF);
  bool b2Valid = b2Ok && pr >= 1 && pr <= 3;
  char hv[20] = "-", hb[12] = "-";
  if (verOk) shfwHex(shfwVer, shfwVerLen > 8 ? 8 : shfwVerLen, hv, sizeof(hv));
  if (b2Ok) snprintf(hb, sizeof(hb), "%04X", w);
  snprintf(shfwDiag, sizeof(shfwDiag), "ver:%s B2:%s", hv, hb);
  Serial.printf("SHFW detect: %s (verOk=%d b2Ok=%d)\n", shfwDiag, verOk, b2Ok);
  if (b2Valid) shfwProfile = pr;
  if (verOk || b2Valid) shfwDetected = 1;
  else if (shfwForce && shfwLive()) { shfwDetected = 1; shfwForced = true; }
  return shfwDetected == 1;
}

// ---------- бэкап/восстановление: слоты в NVS, у каждого сохранена версия SHFW ----------
#define SHFW_SLOTS 5            // 0-3 ручные, 4 = автокопия перед восстановлением
#define SHFW_UNDO_SLOT 4
struct __attribute__((packed)) ShfwBlobHdr {
  char magic[2]; uint8_t fmt, verLen; uint8_t ver[SHFW_VER_MAX]; uint16_t tableId, dumpBytes, crc;
};
struct ShfwSlot { bool valid; uint8_t verLen; uint8_t ver[SHFW_VER_MAX]; };
static ShfwSlot shfwSlots[SHFW_SLOTS];
static Preferences shfwPrefs;
static uint8_t shfwBlob[sizeof(ShfwBlobHdr) + SHFW_DUMP_BYTES];
static uint8_t shfwDump[SHFW_DUMP_BYTES];
static ShfwChunk shfwChunks[SHFW_RUN_COUNT + SHFW_DUMP_BYTES / SHFW_MAX_XFER + 1];
static uint16_t shfwChunkN = 0;
static uint8_t shfwSavedVer[SHFW_VER_MAX];     // версия из слота (для сообщения о несовпадении)
static uint8_t shfwSavedVerLen = 0;
static uint16_t shfwBakFailCount = 0, shfwBakFailReg = 0;

static void shfwSlotKey(uint8_t i, char* k) { snprintf(k, 8, "shb%d", i); }

// 1 - слот годен, 0 - пуст, -1 - повреждён или от другой таблицы регистров.
static int shfwSlotLoad(uint8_t i, ShfwBlobHdr* hdr, uint8_t* data) {
  char k[8];
  shfwSlotKey(i, k);
  if (!shfwPrefs.begin("shfwbak", true)) return 0;
  size_t n = shfwPrefs.getBytesLength(k);
  int res = 0;
  if (n) {
    res = -1;
    if (n == sizeof(shfwBlob) && shfwPrefs.getBytes(k, shfwBlob, n) == n) {
      ShfwBlobHdr h;
      memcpy(&h, shfwBlob, sizeof(h));
      const uint8_t* body = shfwBlob + sizeof(h);
      if (h.magic[0] == 'S' && h.magic[1] == 'B' && h.fmt == 1 && h.verLen > 0 && h.verLen <= SHFW_VER_MAX &&
          h.tableId == SHFW_TABLE_ID && h.dumpBytes == SHFW_DUMP_BYTES &&
          h.crc == shfwCrc16(body, SHFW_DUMP_BYTES, shfwCrc16(h.ver, h.verLen))) {
        if (hdr) *hdr = h;
        if (data) memcpy(data, body, SHFW_DUMP_BYTES);
        res = 1;
      }
    }
  }
  shfwPrefs.end();
  return res;
}

static bool shfwSlotSave(uint8_t i) {          // версия - из shfwVer, данные - из shfwDump
  ShfwBlobHdr h = {};
  h.magic[0] = 'S'; h.magic[1] = 'B'; h.fmt = 1; h.verLen = shfwVerLen;
  memcpy(h.ver, shfwVer, shfwVerLen);
  h.tableId = SHFW_TABLE_ID;
  h.dumpBytes = SHFW_DUMP_BYTES;
  h.crc = shfwCrc16(shfwDump, SHFW_DUMP_BYTES, shfwCrc16(h.ver, h.verLen));
  memcpy(shfwBlob, &h, sizeof(h));
  memcpy(shfwBlob + sizeof(h), shfwDump, SHFW_DUMP_BYTES);
  char k[8];
  shfwSlotKey(i, k);
  if (!shfwPrefs.begin("shfwbak", false)) return false;
  bool ok = shfwPrefs.putBytes(k, shfwBlob, sizeof(shfwBlob)) == sizeof(shfwBlob);
  shfwPrefs.end();
  return ok;
}

static void shfwRefreshSlots() {
  for (uint8_t i = 0; i < SHFW_SLOTS; i++) {
    ShfwBlobHdr h;
    shfwSlots[i].valid = shfwSlotLoad(i, &h, nullptr) == 1;
    shfwSlots[i].verLen = shfwSlots[i].valid ? h.verLen : 0;
    if (shfwSlots[i].valid) memcpy(shfwSlots[i].ver, h.ver, h.verLen);
  }
}

static bool shfwDumpRead() {
  if (!shfwChunkN) shfwChunkN = shfwBuildChunks(SHFW_RUNS, SHFW_RUN_COUNT, shfwChunks, sizeof(shfwChunks) / sizeof(shfwChunks[0]));
  if (!shfwChunkN) return false;
  for (uint16_t c = 0; c < shfwChunkN; c++)
    if (!shfwReadBlock(shfwChunks[c].reg, shfwChunks[c].bytes, shfwDump + shfwChunks[c].off)) return false;
  return true;
}

enum ShfwBakResult : uint8_t { BK_OK, BK_LOCKED, BK_NO_LINK, BK_MOVING, BK_NOT_SHFW, BK_NO_VERSION, BK_EMPTY, BK_BAD_SLOT,
                               BK_VER_MISMATCH, BK_READ_FAIL, BK_SAVE_FAIL, BK_WRITE_FAIL, BK_RESERVED };

static ShfwBakResult shfwBackup(uint8_t slot) {
  if (slot >= SHFW_UNDO_SLOT) return BK_RESERVED;
  if (!shfwLive()) return BK_NO_LINK;
  if (shfwDetected != 1 && !shfwDetect()) return BK_NOT_SHFW;
  if (!shfwReadVersion()) return BK_NO_VERSION;           // без версии копию не делаем
  if (!shfwDumpRead()) return BK_READ_FAIL;
  return shfwSlotSave(slot) ? BK_OK : BK_SAVE_FAIL;
}

static ShfwBakResult shfwRestore(uint8_t slot) {
  static uint8_t saved[SHFW_DUMP_BYTES];
  if (slot >= SHFW_SLOTS) return BK_BAD_SLOT;
  if (!shfwEnabled) return BK_LOCKED;
  if (!shfwLive()) return BK_NO_LINK;
  if (!(d.valid & (1U << SPEED)) || d.speed > 1.0f) return BK_MOVING;
  if (shfwDetected != 1 && !shfwDetect()) return BK_NOT_SHFW;
  if (!shfwReadVersion()) return BK_NO_VERSION;
  ShfwBlobHdr h;
  int ld = shfwSlotLoad(slot, &h, saved);
  if (ld == 0) return BK_EMPTY;
  if (ld < 0) return BK_BAD_SLOT;
  shfwSavedVerLen = h.verLen;
  memcpy(shfwSavedVer, h.ver, h.verLen);
  if (h.verLen != shfwVerLen || memcmp(h.ver, shfwVer, shfwVerLen) != 0) return BK_VER_MISMATCH;   // версия должна совпасть
  if (slot != SHFW_UNDO_SLOT) {                            // сначала копия текущего состояния
    if (!shfwDumpRead()) return BK_READ_FAIL;
    if (!shfwSlotSave(SHFW_UNDO_SLOT)) return BK_SAVE_FAIL;
  }
  if (!shfwChunkN) shfwChunkN = shfwBuildChunks(SHFW_RUNS, SHFW_RUN_COUNT, shfwChunks, sizeof(shfwChunks) / sizeof(shfwChunks[0]));
  if (!shfwChunkN) return BK_READ_FAIL;
  uint8_t tmp[SHFW_MAX_XFER];
  shfwBakFailCount = 0;
  for (uint16_t c = 0; c < shfwChunkN; c++) {
    const ShfwChunk& ch = shfwChunks[c];
    bool ok = shfwWriteBlock(ch.reg, saved + ch.off, ch.bytes) && shfwReadBlock(ch.reg, ch.bytes, tmp) &&
              memcmp(tmp, saved + ch.off, ch.bytes) == 0;
    Serial.printf("SHFW restore reg=%02X bytes=%u %s\n", ch.reg, ch.bytes, ok ? "ok" : "FAIL");
    if (!ok) { if (!shfwBakFailCount) shfwBakFailReg = ch.reg; shfwBakFailCount++; }
  }
  memset(shfwHave, 0, sizeof(shfwHave));                   // кэш редактора устарел
  return shfwBakFailCount ? BK_WRITE_FAIL : BK_OK;
}

enum ShfwResult : uint8_t { SW_OK, SW_LOCKED, SW_NO_LINK, SW_MOVING, SW_NOT_SHFW, SW_READ_FAIL, SW_VERIFY_FAIL };

// Читает слово, меняет только поле параметра, пишет, читает обратно и сверяет.
static ShfwResult shfwWriteItem(const ShfwItem& it, int32_t value) {
  if (!shfwEnabled) return SW_LOCKED;
  if (!shfwLive()) return SW_NO_LINK;
  if (!(d.valid & (1U << SPEED)) || d.speed > 1.0f) return SW_MOVING;     // только на стоянке
  if (shfwDetected != 1 && !shfwDetect()) return SW_NOT_SHFW;
  value = shfwClamp(it, SHFW_OPTS, value);
  uint16_t cur;
  if (!shfwReadReg(it.reg, it.len, cur)) return SW_READ_FAIL;
  uint16_t next = shfwSet(it, cur, (uint32_t)value);
  if (next == cur) return SW_OK;
  uint8_t bytes[2];
  shfwBytesFromWord(next, it.len, bytes);
  Serial.printf("SHFW write reg=%02X len=%u %04X -> %04X (%s = %ld)\n", it.reg, it.len, cur, next, it.name, (long)value);
  sendPacket(ADDR_ESC, CMD_CFG_WRITE, it.reg, bytes, it.len);
  delay(150);
  uint16_t back;
  if (!shfwReadReg(it.reg, it.len, back)) return SW_VERIFY_FAIL;
  return shfwGet(it, back) == (uint16_t)value ? SW_OK : SW_VERIFY_FAIL;
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
//                 ПОЛЬЗОВАТЕЛЬСКИЕ НАСТРОЙКИ (хранятся в NVS)
// =====================================================================
#define SET_VER 1

enum Metric : uint8_t { M_BATT, M_POWER, M_TRIP, M_TEMP, M_ODO, M_VOLT, M_CURR, M_MAXSPD, M_COUNT };
static const char* const METRIC_NAME[M_COUNT] = {
  "BATTERY", "POWER", "TRIP", "ESC / BAT", "ODOMETER", "VOLTAGE", "CURRENT", "TOP SPEED"
};

struct Cal { float x0, xk, y0, yk; uint8_t swap, flip, valid, pad; };
struct Settings {
  uint8_t ver, theme, accent, style, units, bright, dim, flip, lock, showInfo, showDebug, gmaxKmh, gmaxMph;
  uint8_t card[4];
  uint8_t lang;        // 0 - English, 1 - Русский
  uint16_t hue;
  Cal cal;
};
static Settings cfg;
static bool otaEnabled = false;
static bool otaSettingsError = false;

struct Theme { const char* name; const char* nameRu; uint16_t bg, panel, track, txt, dim, good, warn, bad; };
static const Theme THEMES[4] = {
  {"NAVY", "Синяя", C565(8, 10, 18), C565(20, 26, 42), C565(36, 44, 64),  C565(240, 244, 252), C565(125, 138, 165), C565(60, 225, 120), C565(255, 190, 40), C565(255, 70, 70)},
  {"BLACK", "Чёрная", C565(0, 0, 0),   C565(18, 18, 22), C565(44, 44, 52),  C565(245, 245, 250), C565(130, 135, 150), C565(60, 225, 120), C565(255, 190, 40), C565(255, 70, 70)},
  {"SLATE", "Серая", C565(30, 34, 40), C565(46, 52, 60), C565(74, 82, 94), C565(236, 240, 246), C565(150, 160, 176), C565(60, 225, 120), C565(255, 190, 40), C565(255, 80, 80)},
  {"LIGHT", "Светлая", C565(232, 236, 244), C565(255, 255, 255), C565(205, 212, 226), C565(16, 20, 32), C565(92, 104, 126), C565(0, 150, 70), C565(225, 140, 0), C565(215, 40, 40)},
};
static const uint16_t ACCENTS[8] = {
  C565(0, 200, 255), C565(70, 120, 255), C565(170, 90, 255), C565(255, 80, 170),
  C565(255, 70, 70), C565(255, 140, 30), C565(255, 210, 40), C565(60, 225, 120)
};
static const char* const ACCENT_NAME[8] = {"cyan", "blue", "purple", "pink", "red", "orange", "yellow", "green"};
static const char* const STYLE_NAME[3] = {"fade", "solid", "zones"};
static const uint8_t DIM_SEC[4] = {0, 15, 30, 60};

// ---------- Язык и кириллица ----------
#define TR(en, ru) (cfg.lang ? (ru) : (en))
static const char* const ACCENT_NAME_RU[8] = {"голубой", "синий", "сиреневый", "розовый", "красный", "оранжевый", "жёлтый", "зелёный"};
static const char* const STYLE_NAME_RU[3] = {"плавная", "один цвет", "зоны"};
static const char* const METRIC_NAME_RU[M_COUNT] = {"Батарея", "Мощность", "Поездка", "Темп. ESC/АКБ", "Одометр", "Напряжение", "Ток", "Макс. скорость"};
static inline const char* metricName(int m) { m %= M_COUNT; return cfg.lang ? METRIC_NAME_RU[m] : METRIC_NAME[m]; }

static bool nonAscii(const char* s) { while (*s) if ((uint8_t)*s++ >= 0x80) return true; return false; }
static const GFXfont* ruFont(int font) { return font == 4 ? &RU4 : font == 2 ? &RU2 : &RU1; }

// Рисует строку: текст с кириллицей - растровым шрифтом RU1/RU2/RU4 (по размеру встроенного шрифта 1/2/4),
// всё остальное (цифры, латиница) - как раньше встроенными шрифтами TFT_eSPI.
int drawS(const String& s, int x, int y, int font) {
  if ((font == 1 || font == 2 || font == 4) && nonAscii(s.c_str())) {
    tft.setFreeFont(ruFont(font));
    int r = tft.drawString(s, x, y, 1);
    tft.setFreeFont(nullptr);
    return r;
  }
  return tft.drawString(s, x, y, font);
}
int textW(const String& s, int font) {
  if ((font == 1 || font == 2 || font == 4) && nonAscii(s.c_str())) {
    tft.setFreeFont(ruFont(font));
    int w = tft.textWidth(s, 1);
    tft.setFreeFont(nullptr);
    return w;
  }
  return tft.textWidth(s, font);
}

// Статусные строки задаются в коде по-английски; перевод подставляется при отрисовке.
String statusText(const char* s) {
  if (!cfg.lang) return String(s);
  static const char* const M[][2] = {
    {"AUTH INIT", "Авторизация"}, {"PRESS POWER", "Нажмите кнопку"}, {"PAIRING", "Сопряжение"},
    {"NO LINK", "Нет связи"}, {"scan...", "Поиск..."}, {"not found", "Не найден"},
    {"conn fail", "Сбой подключения"}, {"no NUS", "Нет NUS"}, {"chr fail", "Ошибка BLE"},
    {"link lost", "Связь потеряна"}, {"READY", "Готово"}};
  for (auto& m : M) if (!strcmp(s, m[0])) return String(m[1]);
  if (!strncmp(s, "LIVE ", 5)) return String("На связи ") + (s + 5);
  if (!strncmp(s, "connect ", 8)) return String("Подключение ") + (s + 8);
  return String(s);
}

uint16_t hue565(int h) {
  h = ((h % 360) + 360) % 360;
  int f = (h % 60) * 255 / 60, r, g, b;
  switch (h / 60) {
    case 0: r = 255; g = f; b = 0; break;
    case 1: r = 255 - f; g = 255; b = 0; break;
    case 2: r = 0; g = 255; b = f; break;
    case 3: r = 0; g = 255 - f; b = 255; break;
    case 4: r = f; g = 0; b = 255; break;
    default: r = 255; g = 0; b = 255 - f; break;
  }
  return C565(r, g, b);
}

void applyTheme() {
  const Theme& t = THEMES[cfg.theme < 4 ? cfg.theme : 0];
  COL_BG = t.bg; COL_PANEL = t.panel; COL_TRACK = t.track; COL_TXT = t.txt; COL_DIM = t.dim;
  COL_GOOD = t.good; COL_WARN = t.warn; COL_BAD = t.bad;
  COL_ACCENT = cfg.accent < 8 ? ACCENTS[cfg.accent] : hue565(cfg.hue);
}

void setDefaults(bool keepCal) {
  Cal c = cfg.cal;
  memset(&cfg, 0, sizeof(cfg));
  cfg.ver = SET_VER;
  cfg.hue = 200;
  cfg.bright = 100;
  cfg.lock = 1;
  cfg.showInfo = 1;
  cfg.showDebug = 1;
  cfg.gmaxKmh = GAUGE_MAX;
  cfg.gmaxMph = 18;
  cfg.card[0] = M_BATT; cfg.card[1] = M_POWER; cfg.card[2] = M_TRIP; cfg.card[3] = M_TEMP;
  if (keepCal) cfg.cal = c;
}

static Preferences prefs;
static Preferences otaPrefs;
static Preferences ledPrefs;
static bool saveDue = false;
static uint32_t saveAt = 0;

void loadSettings() {
  setDefaults(false);
  if (!prefs.begin("opendash", false)) return;
  if (prefs.getBytesLength("cfg") == sizeof(Settings)) {
    Settings t;
    prefs.getBytes("cfg", &t, sizeof(t));
    if (t.ver == SET_VER && t.theme < 4 && t.accent <= 8 && t.style < 3 && t.dim < 4 && t.lang < 2) cfg = t;
  }
  prefs.end();
}

void loadOtaSetting() {
  if (!otaPrefs.begin("opendash", true)) {
    otaSettingsError = true;
    Serial.println("ERROR: cannot open OTA settings");
    return;
  }
  otaEnabled = otaPrefs.getBool("otaEnabled", false);
  otaPrefs.end();
}

bool saveOtaSetting() {
  if (!otaPrefs.begin("opendash", false)) {
    Serial.println("ERROR: cannot open OTA settings for saving");
    otaSettingsError = true;
    return false;
  }
  size_t saved = otaPrefs.putBool("otaEnabled", otaEnabled);
  otaPrefs.end();
  otaSettingsError = saved == 0;
  if (otaSettingsError) Serial.println("ERROR: failed to save OTA setting");
  return !otaSettingsError;
}

static WebServer otaServer(80);
static char otaSsid[24];
static char otaPassword[13];
static bool otaApReady = false;
static bool otaUploadStarted = false;
static bool otaUploadFailed = false;
static bool otaUploadComplete = false;
static uint32_t otaRebootAt = 0;

void handleOtaUpload() {
  HTTPUpload& upload = otaServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    otaUploadStarted = false;
    otaUploadFailed = false;
    otaUploadComplete = false;
    String filename = upload.filename;
    filename.toLowerCase();
    if (!filename.endsWith(".bin")) {
      otaUploadFailed = true;
      Serial.println("ERROR: OTA upload must be a .bin file");
      return;
    }
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
      otaUploadFailed = true;
      Update.printError(Serial);
      return;
    }
    otaUploadStarted = true;
    Serial.printf("OTA upload started: %s\n", upload.filename.c_str());
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (otaUploadStarted && !otaUploadFailed &&
        Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      otaUploadFailed = true;
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (otaUploadStarted && !otaUploadFailed) {
      otaUploadComplete = Update.end(true);
      if (!otaUploadComplete) {
        otaUploadFailed = true;
        Update.printError(Serial);
      } else {
        Serial.printf("OTA upload complete: %u bytes\n", upload.totalSize);
      }
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    otaUploadFailed = true;
    if (otaUploadStarted) Update.abort();
    Serial.println("ERROR: OTA upload aborted");
  }
}

void startOtaAccessPoint() {
  uint64_t mac = ESP.getEfuseMac();
  snprintf(otaSsid, sizeof(otaSsid), "OpenDash-%06llX", mac & 0xFFFFFFULL);
  const char hex[] = "0123456789ABCDEF";
  uint32_t randomBits[2] = {esp_random(), esp_random()};
  for (size_t i = 0; i < 12; i++)
    otaPassword[i] = hex[(randomBits[i / 8] >> ((i % 8) * 4)) & 0x0F];
  otaPassword[12] = '\0';

  WiFi.mode(WIFI_AP);
  otaApReady = WiFi.softAP(otaSsid, otaPassword);
  if (!otaApReady) {
    Serial.println("ERROR: failed to start OTA Wi-Fi access point");
    return;
  }

  otaServer.on("/", HTTP_GET, []() {
    otaServer.sendHeader("Cache-Control", "no-store");
    otaServer.send(200, "text/html; charset=utf-8",
      "<!doctype html><html lang='ru'><meta name='viewport' content='width=device-width'>"
      "<title>OpenDash OTA</title><style>body{font:16px system-ui;max-width:38em;margin:3em auto;padding:0 1em}"
      "button,input{font:inherit;margin:.5em 0}progress{width:100%}</style>"
      "<h1>OpenDash — обновление</h1><p>Выберите файл прошивки .bin для этой платы. "
      "Не отключайте питание во время загрузки.</p><form id='f'><input id='b' type='file' accept='.bin' required>"
      "<button>Загрузить и установить</button></form><progress id='p' max='100' value='0' hidden></progress>"
      "<p id='s'></p><script>f.onsubmit=async e=>{e.preventDefault();if(!b.files.length)return;"
      "p.hidden=false;s.textContent='Загрузка…';let x=new XMLHttpRequest();x.open('POST','/update');"
      "x.upload.onprogress=e=>{if(e.lengthComputable)p.value=e.loaded/e.total*100};"
      "x.onload=()=>s.textContent=x.status===200?x.responseText:'Ошибка: '+x.responseText;"
      "x.onerror=()=>s.textContent='Ошибка соединения';let d=new FormData();d.append('firmware',b.files[0]);x.send(d)};"
      "</script></html>");
  });
  otaServer.on("/update", HTTP_POST, []() {
    if (!otaUploadStarted || otaUploadFailed || !otaUploadComplete) {
      otaServer.send(400, "text/plain; charset=utf-8", "Ошибка записи прошивки. См. журнал Serial.");
      return;
    }
    otaServer.send(200, "text/plain; charset=utf-8", "Прошивка установлена. Устройство перезагрузится.");
    otaRebootAt = millis() + 1200;
  }, handleOtaUpload);
  otaServer.onNotFound([]() { otaServer.send(404, "text/plain", "Not found"); });
  otaServer.begin();
  Serial.printf("OTA access point: %s, password: %s, address: %s\n",
                otaSsid, otaPassword, WiFi.softAPIP().toString().c_str());
}

void stopOtaAccessPoint() {
  otaServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  otaApReady = false;
  otaRebootAt = 0;
}

void serviceOtaWeb() {
  if (otaEnabled && otaApReady) otaServer.handleClient();
  if (otaRebootAt && (int32_t)(millis() - otaRebootAt) >= 0) ESP.restart();
}

void setOtaEnabled(bool enabled) {
  bool previous = otaEnabled;
  otaEnabled = enabled;
  if (!saveOtaSetting()) {
    otaEnabled = previous;
    return;
  }
  if (otaEnabled) startOtaAccessPoint();
  else stopOtaAccessPoint();
}

void saveSettings() {
  if (!prefs.begin("opendash", false)) return;
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
}
void queueSave() { saveDue = true; saveAt = millis() + 1200; }

uint8_t ledPin() { return LED_PINS[ledCfg.pinIndex]; }

void applyLedHardware() {
  ledStrip.setPin(ledPin());
  ledStrip.updateLength(ledCfg.count);
  ledStrip.begin();
  ledStrip.setBrightness(64);
  ledStrip.clear();
  ledStrip.show();
  ledReady = true;
}

void loadLedSettings() {
  if (!ledPrefs.begin("opendashled", false)) {
    Serial.println("ERROR: cannot open LED settings");
    return;
  }
  if (ledPrefs.getBytesLength("config") == sizeof(ledCfg)) {
    LedSettings saved;
    if (ledPrefs.getBytes("config", &saved, sizeof(saved)) == sizeof(saved) &&
        saved.enabled <= 1 && saved.pinIndex < LED_PIN_COUNT &&
        saved.count >= 1 && saved.count <= LED_MAX_COUNT) {
      uint16_t allocated = 0;
      bool valid = true;
      for (uint8_t i = 0; i < 3; i++) {
        valid &= saved.segmentCount[i] <= LED_MAX_COUNT;
        valid &= saved.effect[i] < LED_EFFECT_COUNT;
        valid &= saved.color[i] < sizeof(LED_COLORS) / sizeof(LED_COLORS[0]);
        allocated += saved.segmentCount[i];
      }
      if (valid && allocated <= saved.count) ledCfg = saved;
    }
  }
  ledPrefs.end();
}

void saveLedSettings() {
  if (!ledPrefs.begin("opendashled", false)) {
    Serial.println("ERROR: cannot open LED settings for saving");
    return;
  }
  size_t saved = ledPrefs.putBytes("config", &ledCfg, sizeof(ledCfg));
  ledPrefs.end();
  if (saved != sizeof(ledCfg)) Serial.println("ERROR: failed to save LED settings");
}

void serviceLedStrip() {
  static uint32_t lastFrame = 0;
  static bool wasLit = false;
  uint32_t now = millis();
  if (!ledReady || now - lastFrame < 40) return;
  lastFrame = now;

  if (!ledCfg.enabled) {
    if (wasLit) {
      ledStrip.clear();
      ledStrip.show();
      wasLit = false;
    }
    return;
  }

  wasLit = true;
  bool braking = brakeUntil && (int32_t)(brakeUntil - now) > 0;
  if (braking) {
    for (uint16_t pixel = 0; pixel < ledCfg.count; pixel++)
      ledStrip.setPixelColor(pixel, ledStrip.Color(255, 0, 0));
    ledStrip.show();
    return;
  }
  ledStrip.clear();
  for (uint8_t segment = 0; segment < 3; segment++) {
    uint16_t start = 0;
    for (uint8_t i = 0; i < segment; i++) start += ledCfg.segmentCount[i];
    uint16_t end = min((uint16_t)ledCfg.count, (uint16_t)(start + ledCfg.segmentCount[segment]));
    for (uint16_t pixel = start; pixel < end; pixel++) {
      uint32_t color = 0;
      uint8_t effect = ledCfg.effect[segment];
      if (effect == LED_SOLID) {
        color = LED_COLORS[ledCfg.color[segment]];
      } else if (effect == LED_PULSE) {
        float phase = (sinf(now * 0.004f) + 1.0f) * 0.5f;
        float level = 0.12f + phase * 0.88f;
        uint32_t base = LED_COLORS[ledCfg.color[segment]];
        color = ledStrip.Color(
          (uint8_t)(((base >> 16) & 0xFF) * level),
          (uint8_t)(((base >> 8) & 0xFF) * level),
          (uint8_t)((base & 0xFF) * level));
      } else if (effect == LED_RAINBOW) {
        color = ledStrip.ColorHSV((uint16_t)(now * 12 + pixel * 65535UL / ledCfg.count));
      }
      ledStrip.setPixelColor(pixel, color);
    }
  }
  ledStrip.show();
}

// Единицы измерения (внутри всё хранится в км/ч и км)
static inline float kmU(float km) { return cfg.units ? km * 0.621371f : km; }
static inline const char* uSpd() { return cfg.units ? TR("mph", "миль/ч") : TR("km/h", "км/ч"); }
static inline const char* uDst() { return cfg.units ? TR("mi", "миль") : TR("km", "км"); }
static inline int gmax() { return cfg.units ? cfg.gmaxMph : cfg.gmaxKmh; }
static inline bool pageOn(int p) { return p == 0 || (p == 1 && cfg.showInfo) || (p == 2 && cfg.showDebug); }
void nextPage(int dir) {
  int n = page;
  for (int i = 0; i < 3; i++) { n = (n + dir + 3) % 3; if (pageOn(n)) { page = n; return; } }
}

// ---------- Экраны ----------
enum Scr : uint8_t { S_MAIN, S_SET, S_APPEAR, S_DISPLAY, S_GAUGE, S_WIDGETS, S_LED, S_OTA, S_PICK, S_CAL,
                     S_SHFW, S_SHFW_LIST, S_SHFW_EDIT, S_SHFW_BAK };
static Scr scr = S_MAIN;
static Scr pickReturn = S_WIDGETS;
static uint8_t pickSlot = 0;
static uint8_t calStep = 0;
static bool calFromBoot = false, calFail = false;
static float calRaw[3][2];
static int uiMode = -1;
static float sessionMax = 0;

// ---------- Подсветка ----------
#define BL_PIN 21
#define BL_CH  0
static bool dimmed = false;
static uint32_t lastAct = 0;

void setBacklight(int pct) { pct = constrain(pct, 5, 100); ledcWrite(BL_CH, (255 * pct) / 100); }
void markActivity() { lastAct = millis(); if (dimmed) { dimmed = false; setBacklight(cfg.bright); } }

// =====================================================================
//                                 ТАЧ
// =====================================================================
#define TOUCH_CS   33
#define TOUCH_IRQ  36
#define TOUCH_CLK  25
#define TOUCH_MISO 39
#define TOUCH_MOSI 32
#define CAL_M      20      // отступ калибровочных точек от краёв
#define LONG_MS    650

SPIClass touchSPI(VSPI);
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

struct TouchState {
  bool down, moved, longDone, swallow, evDown, evTap, evLong, evUp;
  int x, y, sx, sy;
  int8_t swipe;            // +1 свайп влево (след. страница), -1 вправо
  uint32_t t0, lastSeen;
  long rAx, rAy;
  int rN;
} tch;

static inline bool hit(int x, int y, int rx, int ry, int rw, int rh) {
  return x >= rx && x < rx + rw && y >= ry && y < ry + rh;
}

void mapRaw(int rx, int ry, int& sx, int& sy) {
  float a = cfg.cal.swap ? ry : rx;
  float b = cfg.cal.swap ? rx : ry;
  sx = constrain((int)(CAL_M + (a - cfg.cal.x0) * cfg.cal.xk), 0, 239);
  sy = constrain((int)(CAL_M + (b - cfg.cal.y0) * cfg.cal.yk), 0, 319);
  if (cfg.cal.flip != cfg.flip) { sx = 239 - sx; sy = 319 - sy; }
}

void touchUpdate() {
  tch.evDown = tch.evTap = tch.evLong = tch.evUp = false;
  tch.swipe = 0;
  uint32_t now = millis();
  bool pressed = false;
  int rx = 0, ry = 0;
  if (ts.touched()) {
    TS_Point p = ts.getPoint();
    if (p.z > 350) { pressed = true; rx = p.x; ry = p.y; }
  }
  if (pressed) {
    tch.lastSeen = now;
    int sx, sy;
    mapRaw(rx, ry, sx, sy);
    if (!tch.down) {
      tch.down = true; tch.moved = false; tch.longDone = false;
      tch.x = tch.sx = sx; tch.y = tch.sy = sy; tch.t0 = now;
      tch.rAx = tch.rAy = 0; tch.rN = 0;
      tch.swallow = dimmed;            // первое касание только будит экран
      markActivity();
      tch.evDown = !tch.swallow;
    } else {
      tch.x = (tch.x + sx) / 2;
      tch.y = (tch.y + sy) / 2;
    }
    if (tch.rN < 200) { tch.rAx += rx; tch.rAy += ry; tch.rN++; }
    if (!tch.moved && (abs(tch.x - tch.sx) > 14 || abs(tch.y - tch.sy) > 14)) tch.moved = true;
    if (!tch.swallow && !tch.moved && !tch.longDone && now - tch.t0 > LONG_MS) {
      tch.longDone = true;
      tch.evLong = true;
    }
  } else if (tch.down && now - tch.lastSeen > 45) {
    tch.down = false;
    if (!tch.swallow) {
      tch.evUp = true;
      if (!tch.moved && !tch.longDone) tch.evTap = true;
      else if (tch.moved) {
        int dx = tch.x - tch.sx, dy = tch.y - tch.sy;
        if (abs(dx) >= 40 && abs(dx) > abs(dy) * 3 / 2) tch.swipe = dx < 0 ? 1 : -1;
      }
    }
  }
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

String fitText(const String& s, int maxW, int font) {
  if (textW(s, font) <= maxW) return s;
  String t = s;
  while (t.length() > 1 && textW(t + "..", font) > maxW) t.remove(t.length() - 1);
  return t + "..";
}

void drawStatus() {
  if (scr != S_MAIN) return;       // в настройках шапку не трогаем
  stDirty = false;
  tft.fillCircle(10, 11, 3, stCol);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(stCol, COL_PANEL);
  tft.setTextPadding(140);
  drawS(fitText(statusText(stText), 138, 2), 18, 3, 2);
}

void statusNow(const String& s, uint16_t col = COL_WARN) {
  status(s, col);
  if (stDirty) drawStatus();
}

// Кэш текстовых полей: перерисовываем только то, что изменилось (нет мерцания).
#define SLOTS 48
static String cacheS[SLOTS];
static uint16_t cacheC[SLOTS];
static int chipKey = -1;
static int litShown = 0;
static uint32_t barKeys[4];

void clearCache() {
  for (int i = 0; i < SLOTS; i++) { cacheS[i] = "\x01"; cacheC[i] = 0; }
  chipKey = -1;
  litShown = 0;
  for (int i = 0; i < 4; i++) barKeys[i] = 0xFFFFFFFF;
}

void field(uint8_t slot, int x, int y, const String& s, int w, int font,
           uint16_t fg, uint8_t datum, uint16_t bg) {
  if (cacheC[slot] == fg && cacheS[slot] == s) return;
  cacheS[slot] = s;
  cacheC[slot] = fg;
  tft.setTextDatum(datum);
  tft.setTextColor(fg, bg);
  tft.setTextPadding(w);
  drawS(s, x, y, font);
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
  drawS(s, x, y, 1);
}

uint16_t tempColor(float t) { return t < 45 ? COL_GOOD : t < 60 ? COL_WARN : COL_BAD; }

// ---------- Шапка ----------
void drawGear(int cx, int cy, uint16_t col, uint16_t bg) {
  tft.fillRect(cx - 2, cy - 9, 4, 4, col);
  tft.fillRect(cx - 2, cy + 5, 4, 4, col);
  tft.fillRect(cx - 9, cy - 2, 4, 4, col);
  tft.fillRect(cx + 5, cy - 2, 4, 4, col);
  tft.fillCircle(cx, cy, 6, col);
  tft.fillCircle(cx, cy, 2, bg);
}

void drawHeaderStatic(bool dots) {
  tft.fillRect(0, 0, 240, 23, COL_PANEL);
  tft.drawFastHLine(0, 23, 240, COL_TRACK);
  if (dots) {
    int n = 0;
    for (int p = 0; p < 3; p++) if (pageOn(p)) n++;
    int x = 196 - (n - 1) * 12;
    for (int p = 0; p < 3; p++) {
      if (!pageOn(p)) continue;
      tft.fillCircle(x, 11, p == page ? 3 : 2, p == page ? COL_ACCENT : COL_TRACK);
      x += 12;
    }
  }
  drawGear(224, 11, COL_DIM, COL_PANEL);
}

// ---------- Спидометр ----------
#define GAUGE_SEGS 40
const int GCX = 120, GCY = 118, GR_O = 82, GR_I = 69;

uint16_t gaugeColor(int i) {
  float t = (float)i / (GAUGE_SEGS - 1);
  if (cfg.style == 1) return COL_ACCENT;
  if (cfg.style == 2) return t < 0.5f ? COL_GOOD : t < 0.75f ? COL_WARN : COL_BAD;
  const uint16_t stops[4] = {COL_ACCENT, COL_GOOD, COL_WARN, COL_BAD};
  static const float pos[4] = {0.0f, 0.5f, 0.75f, 1.0f};
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
  int lit = constrain((int)lroundf(speed / gmax() * GAUGE_SEGS), 0, GAUGE_SEGS);
  if (lit == litShown) return;
  if (lit > litShown) { for (int i = litShown; i < lit; i++) gaugeSeg(i, gaugeColor(i)); }
  else                { for (int i = lit; i < litShown; i++) gaugeSeg(i, COL_TRACK); }
  litShown = lit;
}

// ---------- Страница 0: панель ----------
static const int CX[4] = {8, 124, 8, 124};
static const int CY[4] = {207, 207, 263, 263};

void dashStatic() {
  for (int i = 0; i < GAUGE_SEGS; i++) gaugeSeg(i, COL_TRACK);
  litShown = 0;

  tft.setTextPadding(0);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COL_DIM, COL_BG);
  for (int k = 0; k <= 3; k++) {
    float a = (-120.0f + 80.0f * k) * 0.0174533f;
    char b[6];
    snprintf(b, sizeof(b), "%d", gmax() * k / 3);
    drawS(b, lroundf(GCX + 57 * sinf(a)), lroundf(GCY - 57 * cosf(a)), 1);
  }
  tft.setTextColor(COL_ACCENT, COL_BG);
  drawS(uSpd(), GCX, GCY + 20, 2);
  for (int i = 0; i < 4; i++) {
    card(CX[i], CY[i], 108, 50);
    label(CX[i] + 8, CY[i] + 4, metricName(cfg.card[i]));
  }
}

void drawBar(int x, int y, int lit, uint16_t col) {
  for (int i = 0; i < 10; i++)
    tft.fillRoundRect(x + (i * 92) / 10, y, 7, 6, 2, i < lit ? col : COL_TRACK);
}

struct CardOut { String v, sub; uint16_t col; int bar; };

String rideModeLabel() {
  if (!(d.valid & (1U << RIDE_MODE))) return "--";
  switch (d.rideMode) {
    case 0: return TR("DRIVE", "ДРАЙВ");
    case 1: return TR("ECO", "ЭКО");
    case 2: return TR("SPORT", "СПОРТ");
    default: return String(TR("MODE ", "РЕЖИМ ")) + String(d.rideMode);
  }
}

uint16_t rideModeColor() {
  if (!(d.valid & (1U << RIDE_MODE))) return COL_DIM;
  switch (d.rideMode) {
    case 0: return COL_ACCENT;
    case 1: return COL_GOOD;
    case 2: return COL_WARN;
    default: return COL_DIM;
  }
}

CardOut metricOut(uint8_t m) {
  CardOut o;
  o.v = "--"; o.sub = " "; o.col = COL_DIM; o.bar = -1;
  const bool hasTotal = (d.valid & ((1U << TOTAL_LO) | (1U << TOTAL_HI))) == ((1U << TOTAL_LO) | (1U << TOTAL_HI));
  switch (m) {
    case M_BATT:
      o.bar = 0;
      if (d.valid & (1U << SOC)) {
        o.v = String(d.soc) + "%";
        o.col = d.soc > 50 ? COL_GOOD : d.soc > 20 ? COL_WARN : COL_BAD;
        o.bar = (d.soc + 5) / 10;
      }
      break;
    case M_POWER:
      if ((d.valid & (1U << VOLT)) && (d.valid & (1U << AMP))) {
        float w = d.volt * d.amp;
        o.v = String((int)w) + " W";
        o.col = w < -5 ? COL_GOOD : w > 600 ? COL_WARN : COL_TXT;
        o.sub = String(d.volt, 1) + "V " + String(d.amp, 1) + "A";
      } else o.v = "-- W";
      break;
    case M_TRIP:
      if (d.valid & (1U << TRIP)) { o.v = String(kmU(d.tripKm), 2); o.col = COL_TXT; }
      o.sub = uDst();
      break;
    case M_TEMP: {
      bool he = d.valid & (1U << ETEMP), hb = d.valid & (1U << BTEMP);
      o.v = (he ? String((int)lroundf(d.escTemp)) : String("--")) + "/" + (hb ? String((int)lroundf(d.battTemp)) : String("--"));
      o.col = he ? tempColor(d.escTemp) : hb ? tempColor(d.battTemp) : COL_DIM;
      o.sub = TR("ESC/BAT C", "ESC/АКБ C");
      break;
    }
    case M_ODO:
      if (hasTotal) {
        float t = kmU(d.totalKm);
        o.v = t >= 1000 ? String((int)lroundf(t)) : String(t, 1);
        o.col = COL_TXT;
      }
      o.sub = uDst();
      break;
    case M_VOLT:
      if (d.valid & (1U << VOLT)) { o.v = String(d.volt, 1) + " V"; o.col = COL_TXT; }
      break;
    case M_CURR:
      if (d.valid & (1U << AMP)) { o.v = String(d.amp, 1) + " A"; o.col = d.amp < -0.5f ? COL_GOOD : COL_TXT; }
      break;
    case M_MAXSPD:
      if (d.valid & (1U << SPEED)) { o.v = String(kmU(sessionMax), 1); o.col = COL_TXT; }
      o.sub = String(uSpd()) + " " + TR("this ride", "за поездку");
      break;
  }
  return o;
}

void dashDynamic() {
  const bool hasSpeed = d.valid & (1U << SPEED);
  const bool hasTotal = (d.valid & ((1U << TOTAL_LO) | (1U << TOTAL_HI))) == ((1U << TOTAL_LO) | (1U << TOTAL_HI));

  updateGauge(hasSpeed ? kmU(d.speed) : 0);
  field(0, GCX, GCY - 8, hasSpeed ? String(kmU(d.speed), 1) : String("--"), 120, 7, COL_TXT, MC_DATUM, COL_BG);
  field(10, GCX, GCY + 40, rideModeLabel(), 100, 2, rideModeColor(), MC_DATUM, COL_BG);
  field(1, GCX, 188, hasTotal ? TR("ODO ", "ОДО ") + String(kmU(d.totalKm), 1) + " " + uDst() : String(TR("ODO -- ", "ОДО -- ")) + uDst(), 150, 2, COL_DIM, MC_DATUM, COL_BG);

  for (int i = 0; i < 4; i++) {
    CardOut c = metricOut(cfg.card[i] % M_COUNT);
    field(2 + 2 * i, CX[i] + 8, CY[i] + 18, c.v, 92, 4, c.col, TL_DATUM, COL_PANEL);
    if (c.bar >= 0) {
      uint32_t key = ((uint32_t)c.bar << 16) | c.col;
      if (key != barKeys[i]) { barKeys[i] = key; drawBar(CX[i] + 8, CY[i] + 36, c.bar, c.col); }
    } else {
      field(3 + 2 * i, CX[i] + 8, CY[i] + 39, c.sub, 92, 1, COL_DIM, TL_DATUM, COL_PANEL);
    }
  }
}

// ---------- Страница 1: информация ----------
static void tileXY(int i, int& x, int& y) { x = 6; y = 28 + i * 36; }

void infoStatic() {
  char odoL[28], tripL[28];
  snprintf(odoL, sizeof(odoL), TR("ODOMETER (%s)", "Одометр (%s)"), uDst());
  snprintf(tripL, sizeof(tripL), TR("TRIP (%s)", "Поездка (%s)"), uDst());
  const char* L[8] = {TR("DEVICE", "Устройство"), TR("SERIAL", "Серийный номер"), TR("FIRMWARE", "Прошивка"), odoL, tripL,
                      TR("VOLTAGE (V)", "Напряжение (В)"), TR("CURRENT (A)", "Ток (А)"), TR("ESC / BAT TEMP", "Темп. ESC / АКБ")};
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
    hasTotal ? String(kmU(d.totalKm), 1) : String("--"),
    d.valid & (1U << TRIP) ? String(kmU(d.tripKm), 2) : String("--"),
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
void connectStatic() {
  tft.setTextPadding(0);
  tft.setTextDatum(TL_DATUM);
  int w1 = tft.textWidth("OPEN", 2), w2 = tft.textWidth("DASH", 2);
  int x = 120 - (w1 + 4 + w2) / 2;
  tft.setTextColor(COL_ACCENT, COL_BG);
  drawS("OPEN", x, 31, 2);
  tft.setTextColor(COL_TXT, COL_BG);
  drawS("DASH", x + w1 + 4, 31, 2);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COL_DIM, COL_BG);
  drawS(TR("Tap the gear for settings", "Настройки: значок шестерёнки"), 120, 292, 1);
}

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
    tft.fillCircle(120 + lroundf(34 * sinf(a)), 100 - lroundf(34 * cosf(a)), 5, lerp565(COL_BG, base, alpha));
  }

  const char* titles[4] = {TR("SEARCHING", "Поиск"), TR("CONNECTING", "Подключение"), TR("PRESS POWER", "Нажмите кнопку"), TR("PAIRING", "Сопряжение")};
  const char* subs[4] = {TR("Turn on the scooter and stay close", "Включите самокат и будьте рядом"),
                         TR("Starting secure handshake", "Устанавливаем защищённую связь"),
                         TR("Press the scooter power button once", "Нажмите кнопку питания самоката"),
                         TR("Almost there...", "Почти готово...")};
  field(40, 120, 156, titles[phase], 224, 4, power ? COL_WARN : COL_TXT, MC_DATUM, COL_BG);
  field(41, 120, 186, subs[phase], 224, 2, COL_DIM, MC_DATUM, COL_BG);

  int key = phase * 2 + (blink ? 1 : 0);
  if (key != chipKey) {
    chipKey = key;
    const char* names[4] = {TR("SCAN", "Поиск"), TR("LINK", "Связь"), TR("POWER", "Кнопка"), TR("PAIR", "Пара")};
    for (int i = 0; i < 4; i++) {
      int x = 8 + i * 57;
      uint16_t fill, fg;
      if (i < phase)       { fill = lerp565(COL_PANEL, COL_ACCENT, 90); fg = COL_TXT; }
      else if (i == phase) { fill = blink ? COL_TXT : base; fg = COL_BG; }
      else                 { fill = COL_PANEL; fg = COL_DIM; }
      tft.fillRoundRect(x, 224, 52, 26, 6, fill);
      tft.setTextDatum(MC_DATUM);
      tft.setTextColor(fg, fill);
      tft.setTextPadding(0);
      drawS(names[i], x + 26, 231, 1);
    }
  }
}

// =====================================================================
//                         ЭКРАНЫ НАСТРОЕК
// =====================================================================
void btn(int x, int y, int w, int h, const char* txt, bool sel, int font = 2) {
  uint16_t fill = sel ? COL_ACCENT : COL_PANEL;
  uint16_t fg = sel ? COL_BG : COL_TXT;
  tft.fillRoundRect(x, y, w, h, 6, fill);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(fg, fill);
  tft.setTextPadding(0);
  drawS(txt, x + w / 2, y + h / 2, font);
}

void btnText(int x, int y, int w, int h, const String& txt, bool sel, int font = 2) {
  uint16_t fill = sel ? COL_ACCENT : COL_PANEL;
  uint16_t fg = sel ? COL_BG : COL_TXT;
  tft.fillRoundRect(x, y, w, h, 6, fill);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(fg, fill);
  tft.setTextPadding(0);
  drawS(txt, x + w / 2, y + h / 2, font);
}

void secLabel(int x, int y, const char* s) {
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.setTextPadding(0);
  drawS(s, x, y, 1);
}

void settingsHeader(const char* title) {
  tft.fillRect(0, 0, 240, 28, COL_PANEL);
  tft.drawFastHLine(0, 28, 240, COL_TRACK);
  tft.fillTriangle(16, 14, 25, 7, 25, 21, COL_ACCENT);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COL_TXT, COL_PANEL);
  tft.setTextPadding(0);
  drawS(title, 130, 14, 2);
}

void drawSlider(int y, float frac) {
  frac = constrain(frac, 0.0f, 1.0f);
  tft.fillRoundRect(10, y, 220, 28, 8, COL_PANEL);
  tft.fillRoundRect(10, y, 12 + (int)(frac * 208), 28, 8, COL_ACCENT);
}

void drawBrightness() {
  char b[8];
  snprintf(b, sizeof(b), "%d%%", cfg.bright);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(COL_ACCENT, COL_BG);
  tft.setTextPadding(48);
  drawS(b, 232, 34, 2);
  drawSlider(52, (cfg.bright - 10) / 90.0f);
}

void drawHueSlider() {
  tft.fillRect(8, 200, 224, 6, COL_BG);
  tft.fillRect(8, 236, 224, 6, COL_BG);
  for (int i = 0; i < 220; i++) tft.drawFastVLine(10 + i, 206, 30, hue565(i * 359 / 219));
  int kx = 10 + cfg.hue * 219 / 359;
  tft.fillRoundRect(kx - 4, 201, 9, 40, 3, COL_TXT);
  tft.fillRect(kx - 2, 206, 5, 30, hue565(cfg.hue));
}

void drawGaugePreview() {
  for (int i = 0; i < GAUGE_SEGS; i++) tft.fillRect(20 + i * 5, 250, 4, 18, gaugeColor(i));
}

uint16_t ledSegmentStart(uint8_t segment) {
  uint16_t start = 0;
  for (uint8_t i = 0; i < segment; i++) start += ledCfg.segmentCount[i];
  return start;
}

// ---------- SHFW: состояние экранов и помощники ----------
#define SHFW_ROWS 6
static const char* const SHFW_GROUP_RU[SG_COUNT] = {"Мощность и лимиты", "Режимы и профили", "Свет",
                                                    "Приборная панель", "Система", "Эксперт"};
static uint8_t shfwGrp = 0, shfwPage = 0, shfwListN = 0;
static uint8_t shfwList[SHFW_ITEM_COUNT];     // индексы параметров выбранной группы
static int shfwSel = -1;                       // редактируемый параметр (индекс в SHFW_ITEMS)
static int32_t shfwEdit = 0, shfwOrig = 0;     // выбранное и считанное значение
static bool shfwEditReady = false;             // значение удалось прочитать
static char shfwMsg[200] = "";
static int8_t shfwBakSel = 0;
static uint16_t shfwMsgCol = 0;

static const char* shfwGroupName(uint8_t g) { return cfg.lang ? SHFW_GROUP_RU[g] : SHFW_GROUP_NAMES[g]; }

static uint8_t shfwGroupCount(uint8_t g) {
  uint8_t n = 0;
  for (uint16_t i = 0; i < SHFW_ITEM_COUNT; i++) if (SHFW_ITEMS[i].group == g) n++;
  return n;
}
static void shfwBuildList() {
  shfwListN = 0;
  for (uint16_t i = 0; i < SHFW_ITEM_COUNT; i++) if (SHFW_ITEMS[i].group == shfwGrp) shfwList[shfwListN++] = (uint8_t)i;
}
static uint8_t shfwPages() { return (shfwListN + SHFW_ROWS - 1) / SHFW_ROWS; }

static void shfwBusy(const char* text) {
  tft.fillRect(0, 300, 240, 20, COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COL_WARN, COL_BG);
  tft.setTextPadding(0);
  drawS(text, 120, 310, 1);
}

// Читает значения параметров текущей страницы списка; при первой же неудаче (нет связи/SHFW) останавливается.
static void shfwLoadPage() {
  shfwBusy(TR("Reading from scooter...", "Чтение из самоката..."));
  uint8_t first = shfwPage * SHFW_ROWS;
  for (uint8_t i = first; i < shfwListN && i < first + SHFW_ROWS; i++)
    if (!shfwEnsure(SHFW_ITEMS[shfwList[i]])) break;
}

// Строка, обрезанная с «..» под заданную ширину.
static String shfwFit(const char* text, int font, int maxW) {
  String t(text);
  if (textW(t, font) <= maxW) return t;
  while (t.length() > 1 && textW(t + "..", font) > maxW) t.remove(t.length() - 1);
  return t + "..";
}

// Текст в несколько строк шрифтом 1 по словам; цвет задаёт вызывающий.
static void shfwDrawWrapped(const char* text, int x, int y, int maxW, int lines) {
  String rest(text);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(0);
  for (int ln = 0; ln < lines && rest.length(); ln++) {
    String line;
    if (ln == lines - 1) {
      line = shfwFit(rest.c_str(), 1, maxW);
      rest = "";
    } else {
      int cut = rest.length();
      while (cut > 0 && textW(rest.substring(0, cut), 1) > maxW) {
        int sp = rest.lastIndexOf(' ', cut - 1);
        cut = sp > 0 ? sp : 0;
      }
      if (cut == 0) cut = min((int)rest.length(), 20);
      line = rest.substring(0, cut);
      rest = rest.substring(cut);
      rest.trim();
    }
    drawS(line, x, y + ln * 11, 1);
  }
}

// Шаги кнопок -/+ для числовых параметров, зависят от размаха диапазона.
static void shfwSteps(const ShfwItem& it, int& s1, int& s2) {
  int range = it.maxV - it.minV;
  if (range <= 30) { s1 = 1; s2 = 5; }
  else if (range <= 300) { s1 = 1; s2 = 10; }
  else if (range <= 3000) { s1 = 10; s2 = 100; }
  else { s1 = 100; s2 = 1000; }
}

static void shfwSetMsg(uint16_t col, const char* text) {
  snprintf(shfwMsg, sizeof(shfwMsg), "%s", text);
  shfwMsgCol = col;
}

void drawSettingsScreen() {
  char b[56];
  switch (scr) {
    case S_SET: {
      settingsHeader(TR("SETTINGS", "Настройки"));
      const char* titles[10] = {TR("APPEARANCE", "Внешний вид"), TR("DISPLAY", "Экран"), TR("GAUGE & UNITS", "Шкала и единицы"),
                                TR("WIDGETS & PAGES", "Виджеты и страницы"), TR("LANGUAGE", "Язык"),
                                TR("TOUCH CALIBRATION", "Калибровка тача"), TR("ARGB LED STRIP", "ARGB-лента"),
                                TR("OTA UPDATE", "Обновление OTA"), TR("SHFW SETTINGS", "Настройки SHFW"),
                                TR("RESET TO DEFAULTS", "Сброс настроек")};
      char dm[10];
      if (cfg.dim) snprintf(dm, sizeof(dm), TR("%ds", "%d с"), DIM_SEC[cfg.dim]); else strcpy(dm, TR("off", "выкл"));
      const char* tn = cfg.lang ? THEMES[cfg.theme].nameRu : THEMES[cfg.theme].name;
      const char* an = cfg.accent < 8 ? (cfg.lang ? ACCENT_NAME_RU[cfg.accent] : ACCENT_NAME[cfg.accent]) : TR("custom", "свой");
      const char* sn = cfg.lang ? STYLE_NAME_RU[cfg.style] : STYLE_NAME[cfg.style];
      char sub[10][56];
      snprintf(sub[0], 56, TR("%s theme, %s accent", "%s, акцент: %s"), tn, an);
      snprintf(sub[1], 56, TR("brightness %d%%, auto-dim %s", "яркость %d%%, затемнение: %s"), cfg.bright, dm);
      snprintf(sub[2], 56, TR("%s, max %d, %s gauge", "%s, макс. %d, шкала: %s"), uSpd(), gmax(), sn);
      snprintf(sub[3], 56, TR("4 cards, info %s, debug %s", "инфо: %s, отладка: %s"),
               cfg.showInfo ? TR("on", "вкл") : TR("off", "выкл"), cfg.showDebug ? TR("on", "вкл") : TR("off", "выкл"));
      snprintf(sub[4], 56, "%s", cfg.lang ? "Русский (нажмите: English)" : "English (tap: Русский)");
      snprintf(sub[5], 56, "%s", TR("re-map the touch panel (3 taps)", "откалибровать сенсор (3 касания)"));
      snprintf(sub[6], 56, TR("%d LEDs, GPIO %d, %s", "%d светодиодов, GPIO %d, %s"),
               (int)ledCfg.count, (int)ledPin(), ledCfg.enabled ? TR("on", "вкл") : TR("off", "выкл"));
      snprintf(sub[7], 56, TR("web access %s", "веб-доступ: %s"),
               otaEnabled ? TR("on", "вкл") : TR("off", "выкл"));
      snprintf(sub[8], 56, "%s", TR("custom firmware settings", "настройки прошивки SHFW"));
      snprintf(sub[9], 56, "%s", TR("press and HOLD to restore defaults", "удерживайте для сброса"));
      for (int i = 0; i < 10; i++) {
        int y = 32 + i * 28;
        tft.fillRoundRect(8, y, 224, 27, 6, COL_PANEL);
        tft.fillRect(8, y + 3, 3, 21, i == 9 ? COL_BAD : COL_ACCENT);
        tft.setTextDatum(TL_DATUM);
        tft.setTextPadding(0);
        tft.setTextColor(i == 9 ? COL_BAD : COL_TXT, COL_PANEL);
        drawS(titles[i], 20, y, 2);
        tft.setTextColor(COL_DIM, COL_PANEL);
        drawS(sub[i], 20, y + 16, 1);
        if (i != 4 && i != 9) {
          tft.drawLine(216, y + 8, 222, y + 14, COL_DIM);
          tft.drawLine(222, y + 14, 216, y + 20, COL_DIM);
        }
      }
      break;
    }
    case S_APPEAR: {
      settingsHeader(TR("APPEARANCE", "Внешний вид"));
      secLabel(8, 36, TR("THEME", "Тема"));
      for (int i = 0; i < 4; i++) {
        int x = 8 + i * 58, y = 48;
        const Theme& t = THEMES[i];
        tft.fillRoundRect(x, y, 54, 40, 6, t.bg);
        tft.drawRoundRect(x, y, 54, 40, 6, cfg.theme == i ? COL_ACCENT : COL_TRACK);
        if (cfg.theme == i) tft.drawRoundRect(x + 1, y + 1, 52, 38, 5, COL_ACCENT);
        tft.fillRoundRect(x + 6, y + 6, 42, 12, 3, t.panel);
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(t.txt, t.bg);
        tft.setTextPadding(0);
        drawS(cfg.lang ? t.nameRu : t.name, x + 27, y + 29, 1);
      }
      secLabel(8, 98, TR("ACCENT COLOR", "Цвет акцента"));
      for (int i = 0; i < 8; i++) {
        int x = 8 + (i % 4) * 58, y = 110 + (i / 4) * 38;
        tft.fillRoundRect(x, y, 54, 34, 6, ACCENTS[i]);
        if (cfg.accent == i) {
          tft.drawRoundRect(x, y, 54, 34, 6, COL_TXT);
          tft.drawRoundRect(x + 1, y + 1, 52, 32, 5, COL_TXT);
          tft.fillCircle(x + 27, y + 17, 4, COL_BG);
        }
      }
      secLabel(8, 190, TR("CUSTOM HUE (drag)", "Свой оттенок (проведите пальцем)"));
      drawHueSlider();
      secLabel(8, 254, TR("Pick a preset or drag the hue strip.", "Выберите цвет или двигайте полосу."));
      break;
    }
    case S_DISPLAY: {
      settingsHeader(TR("DISPLAY", "Экран"));
      secLabel(8, 38, TR("BRIGHTNESS", "Яркость"));
      drawBrightness();
      secLabel(8, 98, TR("AUTO-DIM WHEN IDLE", "Затемнение при бездействии"));
      const char* dl[4] = {TR("OFF", "Выкл"), TR("15 s", "15 с"), TR("30 s", "30 с"), TR("60 s", "60 с")};
      for (int i = 0; i < 4; i++) btn(8 + i * 58, 110, 54, 40, dl[i], cfg.dim == i);
      secLabel(8, 160, TR("ORIENTATION", "Ориентация"));
      btn(8, 172, 108, 40, TR("NORMAL", "Обычная"), cfg.flip == 0);
      btn(124, 172, 108, 40, TR("FLIPPED 180", "Перевёрнута"), cfg.flip == 1);
      secLabel(8, 222, TR("RIDE LOCK (ignore taps above 3 km/h)", "Блок. в движении (быстрее 3 км/ч)"));
      btn(8, 234, 108, 40, TR("ON", "Вкл"), cfg.lock == 1);
      btn(124, 234, 108, 40, TR("OFF", "Выкл"), cfg.lock == 0);
      secLabel(8, 284, TR("Swipes between pages always work.", "Свайпы страниц работают всегда."));
      break;
    }
    case S_GAUGE: {
      settingsHeader(TR("GAUGE & UNITS", "Шкала и единицы"));
      secLabel(8, 36, TR("SPEED UNIT", "Единицы скорости"));
      btn(8, 48, 108, 44, TR("KM/H", "км/ч"), cfg.units == 0);
      btn(124, 48, 108, 44, TR("MPH", "миль/ч"), cfg.units == 1);
      secLabel(8, 104, TR("GAUGE MAX (step 3)", "Максимум шкалы (шаг 3)"));
      btn(8, 116, 64, 44, "-", false, 4);
      btn(168, 116, 64, 44, "+", false, 4);
      tft.setTextDatum(MC_DATUM);
      tft.setTextPadding(0);
      tft.setTextColor(COL_TXT, COL_BG);
      snprintf(b, sizeof(b), "%d", gmax());
      drawS(b, 120, 131, 4);
      tft.setTextColor(COL_DIM, COL_BG);
      drawS(uSpd(), 120, 150, 1);
      secLabel(8, 172, TR("GAUGE COLORS", "Цвета шкалы"));
      btn(8, 184, 72, 44, TR("FADE", "Плавно"), cfg.style == 0);
      btn(84, 184, 72, 44, TR("SOLID", "Один"), cfg.style == 1);
      btn(160, 184, 72, 44, TR("ZONES", "Зоны"), cfg.style == 2);
      secLabel(8, 238, TR("PREVIEW", "Предпросмотр"));
      drawGaugePreview();
      break;
    }
    case S_WIDGETS: {
      settingsHeader(TR("WIDGETS & PAGES", "Виджеты и страницы"));
      secLabel(8, 36, TR("TAP A CARD TO CHOOSE ITS DATA", "Нажмите карточку, чтобы выбрать данные"));
      for (int i = 0; i < 4; i++) {
        int x = 8 + (i % 2) * 116, y = 48 + (i / 2) * 70;
        tft.fillRoundRect(x, y, 108, 64, 6, COL_PANEL);
        tft.fillRect(x, y + 12, 3, 40, COL_ACCENT);
        snprintf(b, sizeof(b), TR("CARD %d", "Карточка %d"), i + 1);
        tft.setTextDatum(TL_DATUM);
        tft.setTextPadding(0);
        tft.setTextColor(COL_DIM, COL_PANEL);
        drawS(b, x + 10, y + 8, 1);
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(COL_TXT, COL_PANEL);
        drawS(metricName(cfg.card[i]), x + 54, y + 38, 2);
      }
      secLabel(8, 194, TR("PAGES", "Страницы"));
      snprintf(b, sizeof(b), TR("INFO: %s", "Инфо: %s"), cfg.showInfo ? TR("ON", "вкл") : TR("OFF", "выкл"));
      btn(8, 206, 108, 44, b, cfg.showInfo);
      snprintf(b, sizeof(b), TR("DEBUG: %s", "Отладка: %s"), cfg.showDebug ? TR("ON", "вкл") : TR("OFF", "выкл"));
      btn(124, 206, 108, 44, b, cfg.showDebug);
      secLabel(8, 262, TR("On the dashboard: tap a card = next data,", "Нажатие на карточку: следующие данные,"));
      secLabel(8, 274, TR("hold a card = pick from list,", "удержание: выбор из списка,"));
      secLabel(8, 286, TR("tap the gauge = switch km/h / mph.", "нажатие на шкалу: км/ч / миль/ч."));
      break;
    }
    case S_LED: {
      settingsHeader(TR("ARGB LED STRIP", "ARGB-лента"));
      secLabel(8, 33, TR("TOTAL LEDS", "Всего светодиодов"));
      btn(8, 44, 58, 30, "-", false, 2);
      btnText(74, 44, 92, 30, String(ledCfg.count) + " LED", false, 1);
      btn(174, 44, 58, 30, "+", false, 2);

      secLabel(8, 78, TR("DATA PIN (tap to change)", "ПИН ДАННЫХ (нажмите)"));
      btnText(8, 88, 224, 27, String("GPIO ") + ledPin(), false, 1);
      btnText(8, 121, 224, 27, ledCfg.enabled ? TR("OUTPUT: ON", "ЛЕНТА: ВКЛ") : TR("OUTPUT: OFF", "ЛЕНТА: ВЫКЛ"),
              ledCfg.enabled, 1);

      for (uint8_t i = 0; i < 3; i++) {
        int y = 158 + i * 48;
        uint16_t start = ledSegmentStart(i);
        uint16_t end = start + ledCfg.segmentCount[i];
        tft.fillRoundRect(8, y, 224, 44, 6, COL_PANEL);
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(COL_TXT, COL_PANEL);
        tft.setTextPadding(0);
        String range = ledCfg.segmentCount[i]
          ? String(start + 1) + "-" + String(end)
          : String(TR("empty", "пусто"));
        drawS(String(TR("SEG ", "СЕГМЕНТ ")) + String(i + 1) + ": " +
              String(TR("LED ", "СВ ")) + range, 14, y + 2, 1);
        btn(14, y + 20, 28, 21, "-", false, 1);
        btnText(45, y + 20, 34, 21, String(ledCfg.segmentCount[i]), false, 1);
        btn(82, y + 20, 28, 21, "+", false, 1);
        uint8_t effect = ledCfg.effect[i] < LED_EFFECT_COUNT ? ledCfg.effect[i] : LED_OFF;
        btnText(114, y + 20, 76, 21,
                cfg.lang ? LED_EFFECT_NAMES_RU[effect] : LED_EFFECT_NAMES[effect], false, 1);
        uint8_t colorIndex = ledCfg.color[i] % (sizeof(LED_COLORS) / sizeof(LED_COLORS[0]));
        uint32_t rgb = LED_COLORS[colorIndex];
        uint16_t swatch = tft.color565((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF);
        tft.fillRoundRect(196, y + 20, 30, 21, 4, swatch);
        tft.drawRoundRect(196, y + 20, 30, 21, 4, COL_TXT);
      }
      break;
    }
    case S_OTA: {
      settingsHeader(TR("WEB FIRMWARE UPDATE", "Обновление по Wi-Fi"));
      secLabel(8, 40, TR("WEB UPDATE ACCESS", "ДОСТУП К ОБНОВЛЕНИЮ"));
      btnText(8, 54, 224, 44,
              otaEnabled ? TR("OTA ACCESS: ON", "OTA-ДОСТУП: ВКЛ") : TR("OTA ACCESS: OFF", "OTA-ДОСТУП: ВЫКЛ"),
              otaEnabled, 2);
      if (otaEnabled && otaApReady) {
        secLabel(8, 116, TR("CONNECT TO WI-FI", "ПОДКЛЮЧИТЕСЬ К WI-FI"));
        drawS(String("SSID: ") + otaSsid, 8, 136, 1);
        drawS(String(TR("PASSWORD: ", "ПАРОЛЬ: ")) + otaPassword, 8, 156, 1);
        drawS(String("URL: http://") + WiFi.softAPIP().toString(), 8, 176, 1);
        secLabel(8, 216, TR("Open the URL and upload a .bin file.", "Откройте адрес и загрузите файл .bin."));
        secLabel(8, 232, TR("Keep power connected during update.", "Не отключайте питание во время обновления."));
      } else if (otaEnabled) {
        drawS(TR("Wi-Fi access point failed to start.", "Не удалось запустить точку доступа Wi-Fi."), 8, 124, 1);
      } else {
        drawS(TR("Enable access only while updating.", "Включайте доступ только при обновлении."), 8, 124, 1);
      }
      if (otaSettingsError)
        drawS(TR("ERROR: setting could not be saved.", "ОШИБКА: настройка не сохранена."), 8, 268, 1);
      break;
    }
    case S_SHFW: {
      settingsHeader(TR("SHFW SETTINGS", "Настройки SHFW"));
      char sb[96];
      const char* st;
      uint16_t sc;
      if (!shfwLive()) { st = TR("Not connected to the scooter", "Нет связи с самокатом"); sc = COL_WARN; }
      else if (shfwDetected == 1) {
        snprintf(sb, sizeof(sb), TR("SHFW detected, active profile %d", "SHFW найдена, активный профиль %d"), shfwProfile);
        st = sb; sc = COL_GOOD;
        if (shfwProfile == 0) st = TR("SHFW detected (version answered)", "SHFW найдена (версия получена)");
      } else if (shfwDetected < 0) { st = TR("Reopen this menu to check SHFW", "Откройте меню заново для проверки"); sc = COL_WARN; }
      else if (shfwForced) { st = TR("FORCED: SHFW not verified!", "ПРИНУДИТЕЛЬНО: SHFW не проверена!"); sc = COL_WARN; }
      else { st = TR("Not detected. HOLD here = force", "Не найдена. Удерж. здесь = принуд."); sc = COL_BAD; }
      tft.setTextDatum(TL_DATUM);
      tft.setTextPadding(0);
      tft.setTextColor(sc, COL_BG);
      drawS(st, 8, 36, 1);
      tft.fillRoundRect(8, 50, 224, 40, 6, shfwEnabled ? COL_WARN : COL_PANEL);
      tft.setTextDatum(TL_DATUM);
      tft.setTextColor(shfwEnabled ? COL_BG : COL_TXT, shfwEnabled ? COL_WARN : COL_PANEL);
      drawS(shfwEnabled ? TR("EDITING: ON", "Правка: ВКЛ") : TR("EDITING: OFF", "Правка: ВЫКЛ"), 20, 55, 2);
      tft.setTextColor(shfwEnabled ? COL_BG : COL_DIM, shfwEnabled ? COL_WARN : COL_PANEL);
      drawS(TR("hold to change (resets on restart)", "удерживайте (сбросится при перезапуске)"), 20, 76, 1);
      for (int g = 0; g <= SG_COUNT; g++) {                     // последняя строка - копии
        int y = 98 + g * 28;
        bool bak = g == SG_COUNT;
        tft.fillRoundRect(8, y, 224, 25, 6, COL_PANEL);
        tft.fillRect(8, y + 3, 3, 19, bak ? COL_WARN : (g == SG_EXPERT ? COL_BAD : COL_ACCENT));
        tft.setTextDatum(ML_DATUM);
        tft.setTextPadding(0);
        tft.setTextColor(g == SG_EXPERT ? COL_BAD : COL_TXT, COL_PANEL);
        drawS(bak ? TR("Backup / restore", "Копии / восстановление") : shfwGroupName(g), 20, y + 12, 2);
        if (!bak) {
          snprintf(sb, sizeof(sb), "%d", shfwGroupCount(g));
          tft.setTextDatum(MR_DATUM);
          tft.setTextColor(COL_DIM, COL_PANEL);
          drawS(sb, 206, y + 12, 2);
        }
        tft.drawLine(216, y + 7, 222, y + 12, COL_DIM);
        tft.drawLine(222, y + 12, 216, y + 17, COL_DIM);
      }
      secLabel(8, 297, TR("Standstill only; verified after write", "Только на стоянке; запись проверяется"));
      if (shfwLive() && shfwDetected >= 0 && shfwDiag[0]) secLabel(8, 307, shfwDiag);
      else secLabel(8, 307, TR("Wrong values can damage the scooter", "Неверные значения могут повредить"));
      break;
    }
    case S_SHFW_BAK: {
      settingsHeader(TR("SHFW BACKUP", "Копии SHFW"));
      char hb[72], vb[96];
      tft.setTextDatum(TL_DATUM);
      tft.setTextPadding(0);
      tft.setTextColor(COL_DIM, COL_BG);
      if (shfwVerLen) {
        shfwHex(shfwVer, shfwVerLen > 14 ? 14 : shfwVerLen, hb, sizeof(hb));
        snprintf(vb, sizeof(vb), "%s %s", TR("version:", "версия:"), hb);
      } else snprintf(vb, sizeof(vb), "%s", TR("version: unknown", "версия: неизвестна"));
      drawS(shfwFit(vb, 1, 224), 8, 34, 1);
      for (int i = 0; i < SHFW_SLOTS; i++) {
        int y = 48 + i * 36;
        tft.fillRoundRect(8, y, 224, 33, 6, COL_PANEL);
        if (i == shfwBakSel) tft.drawRoundRect(8, y, 224, 33, 6, COL_ACCENT);
        tft.setTextDatum(TL_DATUM);
        tft.setTextPadding(0);
        tft.setTextColor(COL_TXT, COL_PANEL);
        if (i == SHFW_UNDO_SLOT) snprintf(vb, sizeof(vb), "%s", TR("UNDO (auto, before restore)", "Отмена (авто, до восстановления)"));
        else snprintf(vb, sizeof(vb), "%s %d", TR("Slot", "Слот"), i + 1);
        drawS(vb, 16, y + 3, 2);
        tft.setTextColor(COL_DIM, COL_PANEL);
        if (shfwSlots[i].valid) {
          shfwHex(shfwSlots[i].ver, shfwSlots[i].verLen > 8 ? 8 : shfwSlots[i].verLen, hb, sizeof(hb));
          drawS(hb, 16, y + 21, 1);
          bool same = shfwVerLen && shfwSlots[i].verLen == shfwVerLen && !memcmp(shfwSlots[i].ver, shfwVer, shfwVerLen);
          tft.setTextDatum(TR_DATUM);
          tft.setTextColor(same ? COL_GOOD : COL_BAD, COL_PANEL);
          drawS(same ? TR("version OK", "версия совпадает") : TR("other version", "другая версия"), 224, y + 21, 1);
        } else drawS(TR("empty", "пусто"), 16, y + 21, 1);
      }
      tft.fillRoundRect(8, 234, 108, 40, 6, COL_PANEL);
      tft.fillRoundRect(124, 234, 108, 40, 6, shfwEnabled ? COL_ACCENT : COL_PANEL);
      tft.setTextDatum(MC_DATUM);
      tft.setTextColor(COL_TXT, COL_PANEL);
      drawS(TR("BACKUP", "КОПИЯ"), 62, 247, 2);
      tft.setTextColor(COL_DIM, COL_PANEL);
      drawS(TR("hold = overwrite", "удерж.=замена"), 62, 264, 1);
      tft.setTextColor(shfwEnabled ? COL_BG : COL_DIM, shfwEnabled ? COL_ACCENT : COL_PANEL);
      drawS(TR("RESTORE", "ВОССТАНОВИТЬ"), 178, 247, 2);
      drawS(shfwEnabled ? TR("hold", "удерживайте") : TR("editing is OFF", "правка выключена"), 178, 264, 1);
      if (shfwMsg[0]) {
        tft.setTextColor(shfwMsgCol, COL_BG);
        shfwDrawWrapped(shfwMsg, 8, 278, 224, 3);
      }
      break;
    }
    case S_SHFW_LIST: {
      settingsHeader(shfwGroupName(shfwGrp));
      uint8_t first = shfwPage * SHFW_ROWS;
      for (uint8_t r = 0; r < SHFW_ROWS && first + r < shfwListN; r++) {
        const ShfwItem& it = SHFW_ITEMS[shfwList[first + r]];
        int y = 34 + r * 40;
        tft.fillRoundRect(8, y, 224, 38, 6, COL_PANEL);
        tft.fillRect(8, y + 5, 3, 28, it.group == SG_EXPERT ? COL_BAD : COL_ACCENT);
        tft.setTextDatum(TL_DATUM);
        tft.setTextPadding(0);
        tft.setTextColor(COL_DIM, COL_PANEL);
        drawS(shfwFit(it.name, 1, 190), 18, y + 4, 1);
        int32_t v;
        char vb[48];
        if (shfwCached(it, v)) shfwFormat(it, SHFW_OPTS, v, cfg.lang, vb, sizeof(vb)); else strcpy(vb, "--");
        tft.setTextColor(COL_TXT, COL_PANEL);
        drawS(shfwFit(vb, 2, 190), 18, y + 16, 2);
        tft.drawLine(216, y + 13, 222, y + 19, COL_DIM);
        tft.drawLine(222, y + 19, 216, y + 25, COL_DIM);
      }
      btn(8, 282, 64, 32, "<", false, 2);
      btn(168, 282, 64, 32, ">", false, 2);
      snprintf(b, sizeof(b), "%d / %d", shfwPage + 1, shfwPages());
      tft.setTextDatum(MC_DATUM);
      tft.setTextPadding(0);
      tft.setTextColor(COL_DIM, COL_BG);
      drawS(b, 120, 298, 2);
      break;
    }
    case S_SHFW_EDIT: {
      if (shfwSel < 0) break;
      const ShfwItem& it = SHFW_ITEMS[shfwSel];
      settingsHeader(shfwGroupName(it.group));
      tft.setTextColor(COL_TXT, COL_BG);
      shfwDrawWrapped(it.name, 8, 36, 224, 2);
      char vb[48];
      if (shfwEditReady) shfwFormat(it, SHFW_OPTS, shfwEdit, cfg.lang, vb, sizeof(vb)); else strcpy(vb, "--");
      int vf = it.kind == SK_ENUM ? 2 : 4;
      if (textW(vb, vf) > 220) vf = 2;
      tft.setTextDatum(MC_DATUM);
      tft.setTextPadding(0);
      tft.setTextColor(shfwEditReady && shfwEdit != shfwOrig ? COL_ACCENT : COL_TXT, COL_BG);
      drawS(vb, 120, 78, vf);
      tft.setTextColor(COL_DIM, COL_BG);
      if (shfwEditReady) {
        char ob[48];
        shfwFormat(it, SHFW_OPTS, shfwOrig, cfg.lang, ob, sizeof(ob));
        snprintf(b, sizeof(b), "%s %s", TR("now:", "сейчас:"), ob);
        drawS(b, 120, 98, 1);
      }
      if (it.kind == SK_NUM) {
        char lo[24], hi[24];
        shfwFormat(it, SHFW_OPTS, it.minV, cfg.lang, lo, sizeof(lo));
        shfwFormat(it, SHFW_OPTS, it.maxV, cfg.lang, hi, sizeof(hi));
        snprintf(b, sizeof(b), "%s %s .. %s", TR("range:", "диапазон:"), lo, hi);
        drawS(b, 120, 110, 1);
      }
      if (it.kind == SK_NUM) {
        int s1, s2;
        shfwSteps(it, s1, s2);
        snprintf(b, sizeof(b), "-%d", s2); btnText(8, 124, 52, 44, b, false, 2);
        snprintf(b, sizeof(b), "-%d", s1); btnText(64, 124, 52, 44, b, false, 2);
        snprintf(b, sizeof(b), "+%d", s1); btnText(120, 124, 52, 44, b, false, 2);
        snprintf(b, sizeof(b), "+%d", s2); btnText(176, 124, 52, 44, b, false, 2);
      } else if (it.kind == SK_ENUM) {
        btn(8, 124, 108, 44, "<", false, 4);
        btn(124, 124, 108, 44, ">", false, 4);
      } else {
        btn(8, 124, 108, 44, TR("OFF", "ВЫКЛ"), shfwEditReady && shfwEdit == 0);
        btn(124, 124, 108, 44, TR("ON", "ВКЛ"), shfwEditReady && shfwEdit == 1);
      }
      btn(8, 176, 100, 44, TR("REVERT", "Отмена"), false, 2);
      bool canWrite = shfwEnabled && shfwDetected == 1 && shfwEditReady && shfwEdit != shfwOrig;
      uint16_t fill = canWrite ? (it.group == SG_EXPERT ? COL_BAD : COL_ACCENT) : COL_PANEL;
      tft.fillRoundRect(116, 176, 116, 44, 6, fill);
      tft.setTextDatum(MC_DATUM);
      tft.setTextColor(canWrite ? COL_BG : COL_DIM, fill);
      drawS(TR("WRITE", "ЗАПИСЬ"), 174, 191, 2);
      drawS(TR("hold to confirm", "удерживайте"), 174, 208, 1);
      if (shfwMsg[0]) {
        tft.setTextColor(shfwMsgCol, COL_BG);
        shfwDrawWrapped(shfwMsg, 8, 228, 224, 3);
      }
      const char* warn = nullptr;
      uint16_t wc = COL_WARN;
      if (!shfwEnabled) warn = TR("Editing is OFF (SHFW menu: hold)", "Правка выключена (меню SHFW)");
      else if (it.group == SG_EXPERT) { warn = TR("EXPERT: wrong values can damage it", "ЭКСПЕРТ: можно повредить самокат"); wc = COL_BAD; }
      if (warn) {
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(wc, COL_BG);
        drawS(warn, 8, 300, 1);
      }
      break;
    }
    case S_PICK: {
      settingsHeader(TR("CHOOSE DATA", "Выбор данных"));
      for (int i = 0; i < M_COUNT; i++)
        btn(8 + (i % 2) * 116, 36 + (i / 2) * 50, 108, 44, metricName(i), cfg.card[pickSlot] == i);
      snprintf(b, sizeof(b), TR("Card %d", "Карточка %d"), pickSlot + 1);
      secLabel(8, 244, b);
      break;
    }
    case S_CAL: {
      static const int TX[3] = {CAL_M, 239 - CAL_M, CAL_M};
      static const int TY[3] = {CAL_M, CAL_M, 319 - CAL_M};
      tft.setTextDatum(MC_DATUM);
      tft.setTextPadding(0);
      tft.setTextColor(COL_TXT, COL_BG);
      drawS(TR("TOUCH CALIBRATION", "Калибровка тача"), 120, 120, 2);
      snprintf(b, sizeof(b), TR("Tap the target (%d / 3)", "Коснитесь метки (%d / 3)"), calStep + 1);
      tft.setTextColor(COL_DIM, COL_BG);
      drawS(b, 120, 150, 2);
      if (calFail) { tft.setTextColor(COL_BAD, COL_BG); drawS(TR("Points too close - try again", "Слишком близко, повторите"), 120, 176, 2); }
      tft.setTextColor(COL_DIM, COL_BG);
      drawS(cfg.cal.valid ? TR("BOOT button = cancel", "Кнопка BOOT: отмена") : TR("A stylus gives best accuracy", "Лучше использовать стилус"), 120, 200, 1);
      int x = TX[calStep], y = TY[calStep];
      tft.drawCircle(x, y, 10, COL_ACCENT);
      tft.drawCircle(x, y, 5, COL_ACCENT);
      tft.drawFastHLine(x - 16, y, 33, COL_ACCENT);
      tft.drawFastVLine(x, y - 16, 33, COL_ACCENT);
      break;
    }
    default: break;
  }
}

// ---------- Диспетчер экрана ----------
void drawUi() {
  bool live = connected && authState == AUTH_READY;
  int mode = scr != S_MAIN ? 100 + (int)scr : (live ? 1 + page : 0);
  if (mode != uiMode) {
    uiMode = mode;
    clearCache();
    tft.fillScreen(COL_BG);
    if (scr == S_MAIN) {
      drawHeaderStatic(mode > 0);
      stDirty = true;
      if (mode == 0) connectStatic();
      else if (mode == 1) dashStatic();
      else if (mode == 2) infoStatic();
      else debugStatic();
    } else {
      drawSettingsScreen();
    }
  }
  if (scr != S_MAIN) return;
  if (stDirty) drawStatus();
  if (mode == 0) connectDynamic();
  else if (mode == 1) dashDynamic();
  else if (mode == 2) infoDynamic();
  else debugDynamic();
}

// =====================================================================
//                         ОБРАБОТКА КАСАНИЙ
// =====================================================================
void goScr(Scr s) { scr = s; uiMode = -1; }
void openSettings() { goScr(S_SET); }
void leaveSettings() { goScr(S_MAIN); }
void openPicker(uint8_t slot, Scr ret) { pickSlot = slot; pickReturn = ret; goScr(S_PICK); }

void shfwOpenRoot() {
  memset(shfwHave, 0, sizeof(shfwHave));
  shfwDetected = -1;
  if (shfwLive()) {
    shfwBusy(TR("Checking SHFW...", "Проверка SHFW..."));
    shfwDetect();
  }
  goScr(S_SHFW);
}

void shfwOpenBak() {
  shfwMsg[0] = 0;
  shfwBakSel = 0;
  if (shfwLive()) {
    shfwBusy(TR("Reading version...", "Чтение версии..."));
    shfwReadVersion();
  } else shfwVerLen = 0;
  shfwRefreshSlots();
  goScr(S_SHFW_BAK);
}

static void shfwBakMessage(ShfwBakResult r, bool restore) {
  char mb[200], a[40], b[40];
  switch (r) {
    case BK_OK:
      if (restore) shfwSetMsg(COL_GOOD, TR("Restored and verified. Previous settings are in the UNDO slot.", "Восстановлено и проверено. Прежние настройки - в слоте Отмена."));
      else { snprintf(mb, sizeof(mb), "%s %d", TR("Saved to slot", "Сохранено в слот"), shfwBakSel + 1); shfwSetMsg(COL_GOOD, mb); }
      break;
    case BK_LOCKED:   shfwSetMsg(COL_WARN, TR("Editing is OFF (SHFW menu: hold)", "Правка выключена (меню SHFW)")); break;
    case BK_NO_LINK:  shfwSetMsg(COL_BAD, TR("No link to the scooter", "Нет связи с самокатом")); break;
    case BK_MOVING:   shfwSetMsg(COL_WARN, TR("Stop the scooter first", "Сначала остановите самокат")); break;
    case BK_NOT_SHFW: shfwSetMsg(COL_BAD, TR("SHFW not detected - nothing done", "SHFW не найдена - ничего не сделано")); break;
    case BK_NO_VERSION: shfwSetMsg(COL_BAD, TR("Version request got no answer - nothing done", "Версия не получена - ничего не сделано")); break;
    case BK_EMPTY:    shfwSetMsg(COL_WARN, TR("Slot is empty", "Слот пуст")); break;
    case BK_BAD_SLOT: shfwSetMsg(COL_BAD, TR("Slot is damaged or from another table", "Слот повреждён или от другой таблицы")); break;
    case BK_RESERVED: shfwSetMsg(COL_WARN, TR("UNDO slot is filled automatically", "Слот Отмена заполняется автоматически")); break;
    case BK_READ_FAIL: shfwSetMsg(COL_BAD, TR("Read failed - nothing written", "Чтение не удалось - ничего не записано")); break;
    case BK_SAVE_FAIL: shfwSetMsg(COL_BAD, TR("Could not save to flash", "Не удалось сохранить во flash")); break;
    case BK_VER_MISMATCH:
      shfwHex(shfwSavedVer, shfwSavedVerLen > 6 ? 6 : shfwSavedVerLen, a, sizeof(a));
      shfwHex(shfwVer, shfwVerLen > 6 ? 6 : shfwVerLen, b, sizeof(b));
      snprintf(mb, sizeof(mb), "%s %s / %s %s", TR("VERSION DIFFERS: saved", "ВЕРСИЯ ДРУГАЯ: в копии"), a, TR("now", "сейчас"), b);
      shfwSetMsg(COL_BAD, mb);
      break;
    case BK_WRITE_FAIL:
      snprintf(mb, sizeof(mb), "%u %s %02X. %s", shfwBakFailCount, TR("block(s) not applied, first at reg", "блок(ов) не принято, первый с рег."),
               shfwBakFailReg, TR("Previous settings: UNDO slot", "Прежние: слот Отмена"));
      shfwSetMsg(COL_BAD, mb);
      break;
  }
}

void shfwDoBackup() {
  shfwBusy(TR("Reading config...", "Чтение настроек..."));
  ShfwBakResult r = shfwBackup(shfwBakSel);
  shfwRefreshSlots();
  shfwBakMessage(r, false);
  uiMode = -1;
}

void shfwDoRestore() {
  shfwBusy(TR("Restoring...", "Восстановление..."));
  ShfwBakResult r = shfwRestore(shfwBakSel);
  shfwRefreshSlots();
  shfwBakMessage(r, true);
  uiMode = -1;
}

void shfwOpenGroup(uint8_t g) {
  shfwGrp = g;
  shfwPage = 0;
  shfwBuildList();
  shfwLoadPage();
  goScr(S_SHFW_LIST);
}

void shfwOpenItem(int idx) {
  shfwSel = idx;
  shfwMsg[0] = 0;
  shfwEditReady = false;
  const ShfwItem& it = SHFW_ITEMS[idx];
  shfwBusy(TR("Reading from scooter...", "Чтение из самоката..."));
  uint16_t w;
  if (shfwReadReg(it.reg, it.len, w)) {
    shfwOrig = shfwEdit = shfwGet(it, w);
    shfwEditReady = true;
  } else {
    shfwSetMsg(COL_BAD, TR("Read failed", "Не удалось прочитать"));
  }
  goScr(S_SHFW_EDIT);
}

void shfwDoApply() {
  if (shfwSel < 0 || !shfwEditReady || shfwEdit == shfwOrig) return;
  const ShfwItem& it = SHFW_ITEMS[shfwSel];
  shfwBusy(TR("Writing...", "Запись..."));
  ShfwResult r = shfwWriteItem(it, shfwEdit);
  int32_t v;
  switch (r) {
    case SW_OK:
      if (shfwCached(it, v)) shfwOrig = shfwEdit = v; else shfwOrig = shfwEdit;
      shfwSetMsg(COL_GOOD, TR("Written and verified", "Записано и проверено"));
      break;
    case SW_LOCKED:   shfwSetMsg(COL_WARN, TR("Editing is OFF (SHFW menu)", "Правка выключена (меню SHFW)")); break;
    case SW_NO_LINK:  shfwSetMsg(COL_BAD, TR("No link to the scooter", "Нет связи с самокатом")); break;
    case SW_MOVING:   shfwSetMsg(COL_WARN, TR("Stop the scooter first", "Сначала остановите самокат")); break;
    case SW_NOT_SHFW: shfwSetMsg(COL_BAD, TR("SHFW not detected - nothing written", "SHFW не найдена - запись не выполнена")); break;
    case SW_READ_FAIL: shfwSetMsg(COL_BAD, TR("Read failed - nothing written", "Чтение не удалось - ничего не записано")); break;
    case SW_VERIFY_FAIL: {
      char vb[48], mb[96];
      if (shfwCached(it, v)) {
        shfwOrig = v;
        shfwFormat(it, SHFW_OPTS, v, cfg.lang, vb, sizeof(vb));
        snprintf(mb, sizeof(mb), "%s %s", TR("Not applied, reads back:", "Не принято, читается:"), vb);
      } else {
        snprintf(mb, sizeof(mb), "%s", TR("Could not verify the write", "Запись не удалось проверить"));
      }
      shfwSetMsg(COL_BAD, mb);
      break;
    }
  }
  uiMode = -1;
}

void goBack() {
  switch (scr) {
    case S_SET: leaveSettings(); break;
    case S_PICK: if (pickReturn == S_MAIN) leaveSettings(); else goScr(S_WIDGETS); break;
    case S_CAL: if (cfg.cal.valid) goScr(calFromBoot ? S_MAIN : S_SET); break;
    case S_SHFW_EDIT: goScr(S_SHFW_LIST); break;
    case S_SHFW_BAK: goScr(S_SHFW); break;
    case S_SHFW_LIST: goScr(S_SHFW); break;
    default: goScr(S_SET); break;
  }
}

void applyDisplayCfg() {
  tft.setRotation(cfg.flip ? 2 : 0);
  setBacklight(cfg.bright);
  applyTheme();
}

bool finishCal() {
  float ax0, ax1, ay0, ay1;
  bool sw = fabsf(calRaw[1][1] - calRaw[0][1]) > fabsf(calRaw[1][0] - calRaw[0][0]);
  ax0 = sw ? calRaw[0][1] : calRaw[0][0];
  ax1 = sw ? calRaw[1][1] : calRaw[1][0];
  ay0 = sw ? calRaw[0][0] : calRaw[0][1];
  ay1 = sw ? calRaw[2][0] : calRaw[2][1];
  if (fabsf(ax1 - ax0) < 300 || fabsf(ay1 - ay0) < 300) return false;
  cfg.cal.swap = sw;
  cfg.cal.x0 = ax0;
  cfg.cal.xk = (239 - 2 * CAL_M) / (ax1 - ax0);
  cfg.cal.y0 = ay0;
  cfg.cal.yk = (319 - 2 * CAL_M) / (ay1 - ay0);
  cfg.cal.flip = cfg.flip;
  cfg.cal.valid = 1;
  return true;
}

void touchCal() {
  if (!tch.evUp || tch.rN < 3) return;
  calRaw[calStep][0] = (float)tch.rAx / tch.rN;
  calRaw[calStep][1] = (float)tch.rAy / tch.rN;
  calStep++;
  if (calStep >= 3) {
    if (finishCal()) {
      saveSettings();
      goScr(calFromBoot ? S_MAIN : S_SET);
      calFromBoot = false;
      calFail = false;
      calStep = 0;
      return;
    }
    calStep = 0;
    calFail = true;
  } else {
    calFail = false;
  }
  uiMode = -1;
}

void touchMain() {
  const bool live = connected && authState == AUTH_READY;
  const bool locked = cfg.lock && live && d.speed > 3.0f;
  if (tch.swipe && live) { nextPage(tch.swipe > 0 ? 1 : -1); return; }
  if (locked) return;
  const bool onDash = live && page == 0;
  if (tch.evTap) {
    int x = tch.sx, y = tch.sy;
    if (hit(x, y, 196, 0, 44, 44)) { openSettings(); return; }
    if (live && hit(x, y, 150, 0, 46, 44)) { nextPage(1); uiMode = -1; return; }
    if (onDash) {
      for (int i = 0; i < 4; i++)
        if (hit(x, y, CX[i], CY[i], 108, 50)) {
          cfg.card[i] = (cfg.card[i] + 1) % M_COUNT;
          queueSave();
          uiMode = -1;
          return;
        }
      if (hit(x, y, 0, 44, 240, 158)) { cfg.units ^= 1; queueSave(); uiMode = -1; return; }
    }
  }
  if (tch.evLong && onDash) {
    for (int i = 0; i < 4; i++)
      if (hit(tch.sx, tch.sy, CX[i], CY[i], 108, 50)) { openPicker(i, S_MAIN); return; }
  }
}

void touchSettings() {
  static int dragId = -1;
  // ---- ползунки (перетаскивание) ----
  if (tch.evDown) {
    dragId = -1;
    if (scr == S_APPEAR && hit(tch.sx, tch.sy, 0, 196, 240, 50)) dragId = 1;
    else if (scr == S_DISPLAY && hit(tch.sx, tch.sy, 0, 44, 240, 50)) dragId = 2;
  }
  if (dragId >= 0) {
    float f = constrain((tch.x - 10) / 220.0f, 0.0f, 1.0f);
    if (dragId == 1) {
      int h = (int)(f * 359);
      if (h != cfg.hue || cfg.accent != 8) { cfg.hue = h; cfg.accent = 8; drawHueSlider(); }
    } else {
      int bv = 10 + (int)(f * 90);
      if (bv != cfg.bright) { cfg.bright = bv; setBacklight(bv); drawBrightness(); }
    }
    if (!tch.down) {
      int id = dragId;
      dragId = -1;
      queueSave();
      if (id == 1) { applyTheme(); uiMode = -1; }
    }
    return;
  }

  if (tch.swipe) return;
  const int x = tch.sx, y = tch.sy;

  if (tch.evTap && hit(x, y, 0, 0, 80, 32)) { goBack(); return; }

  switch (scr) {
    case S_SET:
      for (int i = 0; i < 10; i++) {
        if (!hit(x, y, 8, 32 + i * 28, 224, 27)) continue;
        if (tch.evTap) {
          if (i <= 3) goScr((Scr)(S_APPEAR + i));
          else if (i == 4) { cfg.lang ^= 1; queueSave(); uiMode = -1; }      // язык переключается сразу
          else if (i == 5) { calStep = 0; calFail = false; calFromBoot = false; goScr(S_CAL); }
          else if (i == 6) goScr(S_LED);
          else if (i == 7) goScr(S_OTA);
          else if (i == 8) shfwOpenRoot();
        }
        if (tch.evLong && i == 9) {
          setDefaults(true);
          shfwEnabled = false;
          shfwForce = false;
          ledCfg = LedSettings{};
          saveLedSettings();
          applyLedHardware();
          applyDisplayCfg();
          saveSettings();
          uiMode = -1;
        }
      }
      break;

    case S_APPEAR:
      if (!tch.evTap) break;
      for (int i = 0; i < 4; i++)
        if (hit(x, y, 8 + i * 58, 48, 54, 40)) { cfg.theme = i; applyTheme(); queueSave(); uiMode = -1; return; }
      for (int i = 0; i < 8; i++)
        if (hit(x, y, 8 + (i % 4) * 58, 110 + (i / 4) * 38, 54, 34)) { cfg.accent = i; applyTheme(); queueSave(); uiMode = -1; return; }
      break;

    case S_DISPLAY:
      if (!tch.evTap) break;
      for (int i = 0; i < 4; i++)
        if (hit(x, y, 8 + i * 58, 110, 54, 40)) { cfg.dim = i; queueSave(); uiMode = -1; return; }
      if (hit(x, y, 8, 172, 108, 40))   { cfg.flip = 0; applyDisplayCfg(); queueSave(); uiMode = -1; }
      if (hit(x, y, 124, 172, 108, 40)) { cfg.flip = 1; applyDisplayCfg(); queueSave(); uiMode = -1; }
      if (hit(x, y, 8, 234, 108, 40))   { cfg.lock = 1; queueSave(); uiMode = -1; }
      if (hit(x, y, 124, 234, 108, 40)) { cfg.lock = 0; queueSave(); uiMode = -1; }
      break;

    case S_GAUGE: {
      if (!tch.evTap) break;
      uint8_t& gm = cfg.units ? cfg.gmaxMph : cfg.gmaxKmh;
      if (hit(x, y, 8, 48, 108, 44))   { cfg.units = 0; }
      else if (hit(x, y, 124, 48, 108, 44)) { cfg.units = 1; }
      else if (hit(x, y, 8, 116, 64, 44))   { if (gm > 15) gm -= 3; }
      else if (hit(x, y, 168, 116, 64, 44)) { if (gm < 60) gm += 3; }
      else if (hit(x, y, 8, 184, 72, 44))   { cfg.style = 0; }
      else if (hit(x, y, 84, 184, 72, 44))  { cfg.style = 1; }
      else if (hit(x, y, 160, 184, 72, 44)) { cfg.style = 2; }
      else break;
      queueSave();
      uiMode = -1;
      break;
    }

    case S_WIDGETS:
      if (!tch.evTap) break;
      for (int i = 0; i < 4; i++)
        if (hit(x, y, 8 + (i % 2) * 116, 48 + (i / 2) * 70, 108, 64)) { openPicker(i, S_WIDGETS); return; }
      if (hit(x, y, 8, 206, 108, 44))   { cfg.showInfo ^= 1; page = 0; queueSave(); uiMode = -1; }
      if (hit(x, y, 124, 206, 108, 44)) { cfg.showDebug ^= 1; page = 0; queueSave(); uiMode = -1; }
      break;

    case S_LED:
      if (!tch.evTap) break;
      if (hit(x, y, 8, 44, 58, 30) || hit(x, y, 174, 44, 58, 30)) {
        int count = ledCfg.count + (x < 100 ? -1 : 1);
        ledCfg.count = constrain(count, 1, LED_MAX_COUNT);
        uint16_t allocated = 0;
        for (uint8_t i = 0; i < 3; i++) allocated += ledCfg.segmentCount[i];
        for (int i = 2; allocated > ledCfg.count && i >= 0; i--) {
          uint8_t remove = min((uint16_t)ledCfg.segmentCount[i], (uint16_t)(allocated - ledCfg.count));
          ledCfg.segmentCount[i] -= remove;
          allocated -= remove;
        }
        applyLedHardware();
        saveLedSettings();
        uiMode = -1;
      } else if (hit(x, y, 8, 88, 224, 27)) {
        ledCfg.pinIndex = (ledCfg.pinIndex + 1) % LED_PIN_COUNT;
        applyLedHardware();
        saveLedSettings();
        uiMode = -1;
      } else if (hit(x, y, 8, 121, 224, 27)) {
        ledCfg.enabled ^= 1;
        if (!ledCfg.enabled) {
          ledStrip.clear();
          ledStrip.show();
        }
        saveLedSettings();
        uiMode = -1;
      } else {
        for (uint8_t i = 0; i < 3; i++) {
          int rowY = 158 + i * 48;
          if (hit(x, y, 14, rowY + 20, 28, 21) && ledCfg.segmentCount[i] > 0) {
            ledCfg.segmentCount[i]--;
          } else if (hit(x, y, 82, rowY + 20, 28, 21)) {
            uint16_t allocated = 0;
            for (uint8_t j = 0; j < 3; j++) allocated += ledCfg.segmentCount[j];
            if (allocated < ledCfg.count) ledCfg.segmentCount[i]++;
          } else if (hit(x, y, 114, rowY + 20, 76, 21)) {
            ledCfg.effect[i] = (ledCfg.effect[i] + 1) % LED_EFFECT_COUNT;
          } else if (hit(x, y, 196, rowY + 20, 30, 21)) {
            ledCfg.color[i] = (ledCfg.color[i] + 1) % (sizeof(LED_COLORS) / sizeof(LED_COLORS[0]));
          } else {
            continue;
          }
          saveLedSettings();
          uiMode = -1;
          break;
        }
      }
      break;

    case S_OTA:
      if (tch.evTap && hit(x, y, 8, 54, 224, 44)) {
        setOtaEnabled(!otaEnabled);
        uiMode = -1;
      }
      break;

    case S_SHFW:
      if (tch.evLong && hit(x, y, 8, 32, 224, 16) && shfwLive() && shfwDetected >= 0 && (shfwDetected == 0 || shfwForced)) {
        shfwForce = !shfwForce;                       // обход проверки SHFW: только осознанно
        shfwBusy(TR("Checking SHFW...", "Проверка SHFW..."));
        shfwDetect();
        uiMode = -1;
        break;
      }
      if (tch.evLong && hit(x, y, 8, 50, 224, 40)) { shfwEnabled = !shfwEnabled; uiMode = -1; break; }
      if (!tch.evTap) break;
      for (uint8_t g = 0; g < SG_COUNT; g++)
        if (hit(x, y, 8, 98 + g * 28, 224, 25)) { shfwOpenGroup(g); return; }
      if (hit(x, y, 8, 98 + SG_COUNT * 28, 224, 25)) shfwOpenBak();
      break;

    case S_SHFW_BAK:
      if (tch.evLong && hit(x, y, 8, 234, 108, 40)) { shfwDoBackup(); return; }
      if (tch.evLong && hit(x, y, 124, 234, 108, 40)) { shfwDoRestore(); return; }
      if (!tch.evTap) break;
      for (int i = 0; i < SHFW_SLOTS; i++)
        if (hit(x, y, 8, 48 + i * 36, 224, 33)) { shfwBakSel = i; shfwMsg[0] = 0; uiMode = -1; return; }
      if (hit(x, y, 8, 234, 108, 40)) {
        if (shfwBakSel < SHFW_UNDO_SLOT && !shfwSlots[shfwBakSel].valid) shfwDoBackup();       // пустой слот - сразу
        else { shfwSetMsg(COL_WARN, TR("Hold BACKUP to overwrite the slot", "Удерживайте КОПИЯ для замены слота")); uiMode = -1; }
      } else if (hit(x, y, 124, 234, 108, 40)) {
        shfwSetMsg(COL_WARN, TR("Hold RESTORE to confirm", "Удерживайте ВОССТАНОВИТЬ")); uiMode = -1;
      }
      break;

    case S_SHFW_LIST:
      if (!tch.evTap) break;
      for (uint8_t r = 0; r < SHFW_ROWS; r++) {
        uint8_t li = shfwPage * SHFW_ROWS + r;
        if (li < shfwListN && hit(x, y, 8, 34 + r * 40, 224, 38)) { shfwOpenItem(shfwList[li]); return; }
      }
      if (shfwPages() > 1 && hit(x, y, 8, 282, 64, 32)) {
        shfwPage = (shfwPage + shfwPages() - 1) % shfwPages();
        shfwLoadPage();
        uiMode = -1;
      } else if (shfwPages() > 1 && hit(x, y, 168, 282, 64, 32)) {
        shfwPage = (shfwPage + 1) % shfwPages();
        shfwLoadPage();
        uiMode = -1;
      }
      break;

    case S_SHFW_EDIT: {
      if (shfwSel < 0) break;
      const ShfwItem& it = SHFW_ITEMS[shfwSel];
      if (tch.evLong && hit(x, y, 116, 176, 116, 44)) { shfwDoApply(); return; }   // запись - только удержанием
      if (!tch.evTap) break;
      if (shfwEditReady) {
        int32_t nv = shfwEdit;
        bool ch = true;
        if (it.kind == SK_NUM) {
          int s1, s2;
          shfwSteps(it, s1, s2);
          if (hit(x, y, 8, 124, 52, 44)) nv -= s2;
          else if (hit(x, y, 64, 124, 52, 44)) nv -= s1;
          else if (hit(x, y, 120, 124, 52, 44)) nv += s1;
          else if (hit(x, y, 176, 124, 52, 44)) nv += s2;
          else ch = false;
        } else if (it.kind == SK_ENUM) {
          if (hit(x, y, 8, 124, 108, 44)) nv = shfwEnumStep(it, SHFW_OPTS, nv, -1);
          else if (hit(x, y, 124, 124, 108, 44)) nv = shfwEnumStep(it, SHFW_OPTS, nv, +1);
          else ch = false;
        } else {
          if (hit(x, y, 8, 124, 108, 44)) nv = 0;
          else if (hit(x, y, 124, 124, 108, 44)) nv = 1;
          else ch = false;
        }
        if (ch) { shfwEdit = shfwClamp(it, SHFW_OPTS, nv); shfwMsg[0] = 0; uiMode = -1; return; }
      }
      if (hit(x, y, 8, 176, 100, 44)) { shfwEdit = shfwOrig; shfwMsg[0] = 0; uiMode = -1; }
      else if (hit(x, y, 116, 176, 116, 44)) {
        shfwSetMsg(COL_WARN, TR("Hold the WRITE button to confirm", "Удерживайте кнопку ЗАПИСЬ"));
        uiMode = -1;
      }
      break;
    }

    case S_PICK:
      if (!tch.evTap) break;
      for (int i = 0; i < M_COUNT; i++)
        if (hit(x, y, 8 + (i % 2) * 116, 36 + (i / 2) * 50, 108, 44)) {
          cfg.card[pickSlot] = i;
          queueSave();
          if (pickReturn == S_MAIN) leaveSettings(); else goScr(S_WIDGETS);
          return;
        }
      break;

    default: break;
  }
}

void handleTouch() {
  touchUpdate();
  if (scr == S_MAIN) touchMain();
  else if (scr == S_CAL) touchCal();
  else touchSettings();
}

void housekeeping() {
  uint32_t now = millis();
  serviceLedStrip();
  if (saveDue && (int32_t)(now - saveAt) >= 0) { saveDue = false; saveSettings(); }
  if (connected && authState == AUTH_READY && d.speed > 1.0f) markActivity();   // в движении не гасим экран
  if (cfg.dim && !dimmed && now - lastAct > (uint32_t)DIM_SEC[cfg.dim] * 1000UL) {
    dimmed = true;
    setBacklight(max(5, (int)cfg.bright / 6));
  }
  if (connected && authState == AUTH_READY && d.speed > sessionMax) sessionMax = d.speed;
}

void handleButton() {
  static uint32_t downAt = 0;
  static bool wasDown = false;
  bool down = digitalRead(BTN_PIN) == LOW;
  if (down && !wasDown) { downAt = millis(); markActivity(); }
  if (!down && wasDown) {
    uint32_t dt = millis() - downAt;
    if (dt > 1500) { if (client && client->isConnected()) client->disconnect(); connected = false; }
    else if (dt > 40) {
      if (scr == S_MAIN) nextPage(1);
      else goBack();
    }
  }
  wasDown = down;
}

// Один «тик» интерфейса: кнопка, тач, подсветка, перерисовка. Зовётся и из долгих ожиданий BLE.
void uiTick() {
  static uint32_t lastDraw = 0;
  handleButton();
  handleTouch();
  serviceOtaWeb();
  housekeeping();
  uint32_t now = millis();
  if (now - lastDraw >= 90) { lastDraw = now; drawUi(); }
}

void idleWait(uint32_t ms) {
  uint32_t t = millis();
  while (millis() - t < ms) { uiTick(); delay(4); }
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
  sc->clearResults();
  // Скан не блокирует цикл: пока он идёт, интерфейс и тач продолжают работать.
  sc->start(4, nullptr, false);
  uint32_t t0 = millis();
  while (sc->isScanning() && millis() - t0 < 6000) { uiTick(); delay(4); }
  NimBLEScanResults r = sc->getResults();
  NimBLEAddress best; bool found = false; int bestRssi = -999; String bestName;
  for (int i = 0; i < r.getCount(); i++) {
    NimBLEAdvertisedDevice a = r.getDevice(i);
    if (!nameMatches(a.getName())) continue;
    if (a.getRSSI() > bestRssi) { bestRssi = a.getRSSI(); best = a.getAddress(); bestName = a.getName().c_str(); found = true; }
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
  d.totalLow = 0; d.totalHigh = 0; d.tripKm = 0; sessionMax = 0;
  brakeUntil = 0;
  memset(d.serial, 0, sizeof(d.serial)); d.fw = 0;
  status("AUTH INIT", COL_ACCENT);
  return true;
}

// ---------- Основной цикл ----------
void setup() {
  Serial.begin(115200);
  pinMode(BTN_PIN, INPUT_PULLUP);
  loadSettings();
  loadOtaSetting();
  loadLedSettings();
  applyLedHardware();
  applyTheme();
  tft.init();
  tft.setRotation(cfg.flip ? 2 : 0);   // портрет 240x320 (0 или 180°)
  ledcSetup(BL_CH, 5000, 8);
  ledcAttachPin(BL_PIN, BL_CH);
  setBacklight(cfg.bright);
  touchSPI.begin(TOUCH_CLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);
  ts.begin(touchSPI);
  ts.setRotation(0);
  if (!cfg.cal.valid) { scr = S_CAL; calStep = 0; calFromBoot = true; }   // первый запуск: калибровка
  tft.fillScreen(COL_BG);
  clearCache();
  lastAct = millis();
  drawUi();
  if (otaEnabled) startOtaAccessPoint();
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  Serial.println("Ninebot G30/ES4 dashboard; encrypted handshake required");
}

void loop() {
  static uint32_t lastPoll = 0, lastFastPoll = 0, lastDraw = 0;
  static int idx = 0;
  static bool pollBrakeCurrent = false;
  handleButton();
  handleTouch();
  serviceOtaWeb();
  housekeeping();

  if (!connected) {
    if (!scanAndConnect()) { drawUi(); idleWait(1500); return; }
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

  if (authState == AUTH_READY && now - lastFastPoll >= FAST_TELEMETRY_POLL_MS) {
    lastFastPoll = now;
    if (pollBrakeCurrent) sendRead(ADDR_BMS, 0x33, 2);
    else sendRead(ADDR_ESC, 0x26, 2);
    pollBrakeCurrent = !pollBrakeCurrent;
  }

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
