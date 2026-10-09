#include <Arduino.h>
#include <NimBLEDevice.h>
#include <mbedtls/aes.h>
#include <mbedtls/sha1.h>
#include <cstring>
#include <string>

#define TARGET_NAME ""
#define APP_ADDR 0x3E
#define ESC_ADDR 0x20
#define BLE_ADDR 0x21
#define BMS_ADDR 0x22

static NimBLEUUID NUS_SVC("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID NUS_WRITE("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID NUS_NOTIFY("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
static const uint8_t FW_DATA[16] = {
  0x97, 0xCF, 0xB8, 0x02, 0x84, 0x41, 0x43, 0xDE,
  0x56, 0x00, 0x2B, 0x3B, 0x34, 0x78, 0x0A, 0x5D
};

struct Probe { uint8_t device, reg, length; };
static const Probe readProbes[] = {
  {BMS_ADDR, 0x32, 2}, {ESC_ADDR, 0x26, 2}, {BMS_ADDR, 0x34, 2},
  {BMS_ADDR, 0x33, 2}, {BMS_ADDR, 0x35, 2}, {ESC_ADDR, 0x29, 2},
  {ESC_ADDR, 0x2A, 2}, {ESC_ADDR, 0x3E, 2}, {ESC_ADDR, 0xB9, 2},
  {ESC_ADDR, 0x10, 14}, {ESC_ADDR, 0x1A, 2}
};

enum AuthState : uint8_t { AUTH_IDLE, AUTH_WAIT_INIT, AUTH_WAIT_PING, AUTH_WAIT_PAIR, AUTH_READY };
enum PendingTx : uint8_t { TX_NONE, TX_INIT, TX_PING, TX_PAIR };

static NimBLEClient* client = nullptr;
static NimBLERemoteCharacteristic* writeChar = nullptr;
static NimBLERemoteCharacteristic* notifyChar = nullptr;
static volatile bool connected = false;
static volatile AuthState authState = AUTH_IDLE;
static volatile PendingTx pendingTx = TX_NONE;
static uint8_t scooterName16[16] = {};
static uint8_t randomBleData[16] = {};
static uint8_t randomAppData[16] = {};
static uint8_t shaKey[16] = {};
static uint8_t scooterSerial[14] = {};
static uint32_t messageCounter = 0;
static uint32_t authLastSend = 0;
static uint32_t lastCommandAt = 0;
static uint8_t rxBuffer[192] = {};
static size_t rxLength = 0;
static size_t rxExpected = 0;
static size_t readIndex = 0;
static uint8_t sweepIndex = 0;
static bool sweepRunning = false;
static bool manualPending = false;
static Probe manualProbe = {};

void printHex(const uint8_t* data, size_t length) {
  for (size_t i = 0; i < length; i++) Serial.printf("%02X", data[i]);
}

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
  rxLength = 0;
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

  size_t offset = 3;
  size_t remaining = payloadLength;
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
    if (plainLength >= 37 && plain[2] == 0x1E && plain[3] == BLE_ADDR &&
        plain[4] == APP_ADDR && plain[5] == 0x5B) {
      memcpy(randomBleData, plain + 7, 16);
      memcpy(scooterSerial, plain + 23, 14);
      setShaKey(scooterName16, randomBleData);
      Serial.println("INIT decoded: BLE random and serial captured");
    }
  } else if (nextCounter > messageCounter) {
    if (!cryptoNext(encrypted + 3, plain + 3, payloadLength, nextCounter)) return false;
    if (plainLength >= 7 && plain[3] == BLE_ADDR && plain[4] == APP_ADDR &&
        plain[5] == 0x5C && plain[6] == 0x01) {
      setShaKey(randomAppData, randomBleData);
      Serial.println("PING 0x5C01 decoded: session key switched");
    }
    messageCounter = nextCounter;
  } else {
    Serial.printf("RX counter out of order: %lu <= %lu\n",
                  (unsigned long)nextCounter, (unsigned long)messageCounter);
    return false;
  }
  return true;
}

bool nameMatches(const std::string& name) {
  if (name.empty()) return false;
  if (strlen(TARGET_NAME)) return name == TARGET_NAME;
  String lower = name.c_str();
  lower.toLowerCase();
  return lower.startsWith("nb") || lower.indexOf("ninebot") >= 0 ||
         lower.indexOf("segway") >= 0 || lower.indexOf("es4") >= 0;
}

void handlePlainPacket(const uint8_t* plain, size_t length);

void processEncryptedFrame(const uint8_t* frame, size_t length) {
  uint8_t plain[192];
  size_t plainLength = 0;
  uint32_t counter = 0;
  Serial.printf("RX ENC n=%u hex=", (unsigned)length);
  printHex(frame, length);
  Serial.println();
  if (!decryptPacket(frame, length, plain, sizeof(plain), plainLength, counter)) {
    Serial.println("RX decrypt failed");
    return;
  }
  Serial.printf("RX PLAIN counter=%lu hex=", (unsigned long)counter);
  printHex(plain, plainLength);
  Serial.println();
  handlePlainPacket(plain, plainLength);
}

void onNotify(NimBLERemoteCharacteristic*, uint8_t* data, size_t length, bool) {
  Serial.printf("RX BLE chunk n=%u hex=", (unsigned)length);
  printHex(data, length);
  Serial.println();
  if (length >= 2 && data[0] == 0x5A && data[1] == 0xA5) {
    rxLength = 0;
    rxExpected = 0;
  }
  if (rxLength + length > sizeof(rxBuffer)) {
    Serial.println("RX buffer overflow");
    rxLength = 0;
    rxExpected = 0;
    return;
  }
  memcpy(rxBuffer + rxLength, data, length);
  rxLength += length;
  while (rxLength >= 3) {
    if (rxBuffer[0] != 0x5A || rxBuffer[1] != 0xA5) {
      Serial.println("RX frame header lost");
      rxLength = 0;
      rxExpected = 0;
      return;
    }
    if (!rxExpected) rxExpected = (size_t)rxBuffer[2] + 13;
    if (rxExpected > sizeof(rxBuffer)) {
      rxLength = 0;
      rxExpected = 0;
      return;
    }
    if (rxLength < rxExpected) return;
    processEncryptedFrame(rxBuffer, rxExpected);
    size_t remaining = rxLength - rxExpected;
    if (remaining) memmove(rxBuffer, rxBuffer + rxExpected, remaining);
    rxLength = remaining;
    rxExpected = 0;
  }
}

class ProbeClientCallbacks : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient*) override {
    connected = false;
    writeChar = nullptr;
    notifyChar = nullptr;
    authState = AUTH_IDLE;
    pendingTx = TX_NONE;
    Serial.println("BLE disconnected");
  }
} clientCallbacks;

void printGattTable() {
  std::vector<NimBLERemoteService*>* services = client->getServices(true);
  Serial.printf("GATT services: %u\n", (unsigned)services->size());
  for (NimBLERemoteService* service : *services) {
    std::string serviceUuid = service->getUUID().toString();
    Serial.printf("SERVICE %s\n", serviceUuid.c_str());
    std::vector<NimBLERemoteCharacteristic*>* chars = service->getCharacteristics(true);
    for (NimBLERemoteCharacteristic* characteristic : *chars) {
      std::string uuid = characteristic->getUUID().toString();
      Serial.printf("  CHAR %s R%d W%d WNR%d N%d I%d\n", uuid.c_str(),
                    characteristic->canRead(), characteristic->canWrite(),
                    characteristic->canWriteNoResponse(), characteristic->canNotify(),
                    characteristic->canIndicate());
    }
  }
}

bool scanAndConnect() {
  Serial.println("SCAN start");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  NimBLEScanResults results = scan->start(5, false);
  NimBLEAddress address;
  String selectedName;
  int bestRssi = -999;
  bool found = false;
  for (auto it = results.begin(); it != results.end(); ++it) {
    NimBLEAdvertisedDevice* device = *it;
    std::string name = device->getName();
    if (!nameMatches(name)) continue;
    Serial.printf("FOUND name=%s rssi=%d address=%s\n", name.c_str(), device->getRSSI(),
                  device->getAddress().toString().c_str());
    if (device->getRSSI() > bestRssi) {
      bestRssi = device->getRSSI();
      address = device->getAddress();
      selectedName = name.c_str();
      found = true;
    }
  }
  scan->clearResults();
  if (!found) {
    Serial.println("No matching scooter found; set TARGET_NAME if needed");
    return false;
  }
  if (!client) {
    client = NimBLEDevice::createClient();
    client->setClientCallbacks(&clientCallbacks, false);
    client->setConnectTimeout(8);
  }
  Serial.printf("CONNECT name=%s\n", selectedName.c_str());
  if (!client->connect(address)) {
    Serial.println("CONNECT failed");
    return false;
  }
  connected = true;
  printGattTable();
  NimBLERemoteService* service = client->getService(NUS_SVC);
  if (!service) {
    Serial.println("NUS service missing");
    client->disconnect();
    return false;
  }
  writeChar = service->getCharacteristic(NUS_WRITE);
  notifyChar = service->getCharacteristic(NUS_NOTIFY);
  if (!writeChar || !notifyChar) {
    Serial.println("NUS characteristic missing");
    client->disconnect();
    return false;
  }
  bool subscribed = notifyChar->canNotify() ? notifyChar->subscribe(true, onNotify)
                                            : notifyChar->subscribe(false, onNotify);
  if (!subscribed) {
    Serial.println("NUS subscribe failed");
    client->disconnect();
    return false;
  }
  resetCrypto(std::string(selectedName.c_str()));
  authState = AUTH_WAIT_INIT;
  authLastSend = millis();
  pendingTx = TX_INIT;
  readIndex = 0;
  sweepRunning = false;
  Serial.println("NUS ready; starting encrypted 0x5B INIT (read-only session)");
  return true;
}

bool writeFrame(const uint8_t* data, size_t length) {
  if (!writeChar || !connected) return false;
  bool withResponse = writeChar->canWrite();
  if (!withResponse && !writeChar->canWriteNoResponse()) return false;
  size_t offset = 0;
  while (offset < length) {
    size_t chunk = min((size_t)20, length - offset);
    bool ok = writeChar->writeValue(data + offset, chunk, withResponse);
    Serial.printf("BLE TX chunk %u..%u ACK=%d\n", (unsigned)offset,
                  (unsigned)(offset + chunk - 1), ok);
    if (!ok) return false;
    offset += chunk;
    delay(20);
  }
  return true;
}

void sendPacket(uint8_t destination, uint8_t command, uint8_t argument,
                const uint8_t* data, uint8_t dataLength) {
  uint8_t plain[96] = {0x5A, 0xA5, dataLength, APP_ADDR, destination, command, argument};
  uint8_t encrypted[112];
  if (dataLength) memcpy(plain + 7, data, dataLength);
  size_t plainLength = 7 + dataLength;
  Serial.printf("TX PLAIN dst=%02X cmd=%02X arg=%02X frame=", destination, command, argument);
  printHex(plain, plainLength);
  Serial.println();
  size_t encryptedLength = encryptPacket(plain, plainLength, encrypted, sizeof(encrypted));
  if (!encryptedLength) {
    Serial.println("TX encrypt failed");
    return;
  }
  Serial.printf("TX ENC counter=%lu hex=", (unsigned long)messageCounter);
  printHex(encrypted, encryptedLength);
  Serial.println();
  if (!writeFrame(encrypted, encryptedLength)) Serial.println("TX GATT write failed");
  lastCommandAt = millis();
}

void sendInit() {
  authState = AUTH_WAIT_INIT;
  authLastSend = millis();
  sendPacket(BLE_ADDR, 0x5B, 0, nullptr, 0);
}

void sendPair() {
  authState = AUTH_WAIT_PAIR;
  authLastSend = millis();
  sendPacket(BLE_ADDR, 0x5D, 0, scooterSerial, sizeof(scooterSerial));
}

void sendRead(const Probe& probe) {
  uint8_t requestedLength = probe.length;
  sendPacket(probe.device, 0x01, probe.reg, &requestedLength, 1);
}

uint32_t readLittleEndian(const uint8_t* data, size_t length) {
  uint32_t value = 0;
  if (length > 4) length = 4;
  for (size_t i = 0; i < length; i++) value |= (uint32_t)data[i] << (8 * i);
  return value;
}

void handleTelemetry(const uint8_t* plain, size_t length) {
  if (length < 7 || plain[4] != APP_ADDR || plain[5] != 0x04) return;
  uint8_t source = plain[3], reg = plain[6], dataLength = plain[2];
  if (length < (size_t)7 + dataLength) return;
  const uint8_t* data = plain + 7;
  Serial.printf("READ src=%02X reg=%02X len=%u data=", source, reg, dataLength);
  printHex(data, dataLength);
  Serial.println();
  if (source == ESC_ADDR && reg == 0x26 && dataLength >= 2)
    Serial.printf("VALUE speed=%.1f km/h\n", (int16_t)readLittleEndian(data, 2) / 10.0f);
  else if (source == BMS_ADDR && reg == 0x32 && dataLength >= 2)
    Serial.printf("VALUE battery=%u%%\n", (unsigned)readLittleEndian(data, 2));
  else if (source == BMS_ADDR && reg == 0x33 && dataLength >= 2)
    Serial.printf("VALUE current=%.2f A\n", (int16_t)readLittleEndian(data, 2) / 100.0f);
  else if (source == BMS_ADDR && reg == 0x34 && dataLength >= 2)
    Serial.printf("VALUE voltage=%.2f V\n", readLittleEndian(data, 2) / 100.0f);
  else if (source == BMS_ADDR && reg == 0x35 && dataLength >= 2)
    Serial.printf("VALUE battery_temp=%d/%d C\n", (int)data[0] - 20, (int)data[1] - 20);
  else if (source == ESC_ADDR && reg == 0x3E && dataLength >= 2)
    Serial.printf("VALUE esc_temp=%.1f C\n", readLittleEndian(data, 2) / 10.0f);
  else if (source == ESC_ADDR && (reg == 0x29 || reg == 0x2A) && dataLength >= 2)
    Serial.printf("VALUE odometer_%s=%u m\n", reg == 0x29 ? "low" : "high",
                  (unsigned)readLittleEndian(data, 2));
}

void handlePlainPacket(const uint8_t* plain, size_t length) {
  if (length < 7) return;
  uint8_t source = plain[3], destination = plain[4], command = plain[5], argument = plain[6];
  Serial.printf("PACKET src=%02X dst=%02X cmd=%02X arg=%02X len=%u\n",
                source, destination, command, argument, plain[2]);
  if (source == BLE_ADDR && destination == APP_ADDR && command == 0x5B &&
      authState == AUTH_WAIT_INIT) {
    for (uint8_t& byte : randomAppData) byte = (uint8_t)esp_random();
    authState = AUTH_WAIT_PING;
    authLastSend = millis();
    pendingTx = TX_PING;
    Serial.println("INIT accepted; scheduling encrypted 0x5C PING");
    return;
  }
  if (source == BLE_ADDR && destination == APP_ADDR && command == 0x5C &&
      authState == AUTH_WAIT_PING) {
    authLastSend = millis();
    if (argument == 1) {
      authState = AUTH_WAIT_PAIR;
      pendingTx = TX_PAIR;
      Serial.println("Power confirmed; scheduling encrypted 0x5D PAIR");
    } else {
      Serial.println("Scooter requests power-button confirmation; press its power button once");
    }
    return;
  }
  if (source == BLE_ADDR && destination == APP_ADDR && command == 0x5D &&
      authState == AUTH_WAIT_PAIR) {
    authLastSend = millis();
    if (argument == 1) {
      authState = AUTH_READY;
      readIndex = 0;
      Serial.println("AUTH READY; starting read-only G30 telemetry probes");
    } else {
      Serial.printf("PAIR response arg=%02X; retry scheduled\n", argument);
    }
    return;
  }
  if (authState == AUTH_READY) handleTelemetry(plain, length);
}

void serviceAuth() {
  if (!connected) return;
  uint32_t now = millis();
  if (authState == AUTH_WAIT_INIT && now - authLastSend > 5000) {
    Serial.println("INIT timeout; restarting BLE session");
    client->disconnect();
  } else if (authState == AUTH_WAIT_PING && now - authLastSend > 1000) {
    Serial.println("Retry 0x5C; press power button if requested");
    authLastSend = now;
    pendingTx = TX_PING;
  } else if (authState == AUTH_WAIT_PAIR && now - authLastSend > 1500) {
    Serial.println("Retry 0x5D PAIR");
    authLastSend = now;
    pendingTx = TX_PAIR;
  }
}

void servicePendingTx() {
  PendingTx next = pendingTx;
  if (next == TX_NONE || !connected) return;
  pendingTx = TX_NONE;
  if (next == TX_INIT) sendInit();
  else if (next == TX_PING) sendPacket(BLE_ADDR, 0x5C, 0, randomAppData, sizeof(randomAppData));
  else if (next == TX_PAIR) sendPair();
}

void readSerialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.equalsIgnoreCase("k")) {
    if (connected && client) client->disconnect();
    Serial.println("Restarting authenticated session");
    return;
  }
  if (line.equalsIgnoreCase("s")) {
    if (authState != AUTH_READY) {
      Serial.println("Wait for AUTH READY before scanning registers");
      return;
    }
    sweepIndex = 0;
    sweepRunning = true;
    Serial.println("Encrypted read-only scan: ESC 20-4F, BMS 30-3F");
    return;
  }
  unsigned device, reg, length;
  if (sscanf(line.c_str(), "r %x %x %x", &device, &reg, &length) == 3 &&
      device <= 0xFF && reg <= 0xFF && length > 0 && length <= 0x40) {
    if (authState != AUTH_READY) {
      Serial.println("Manual reads require AUTH READY");
      return;
    }
    manualProbe = {(uint8_t)device, (uint8_t)reg, (uint8_t)length};
    manualPending = true;
    Serial.println("Manual encrypted read queued");
    return;
  }
  Serial.println("Commands: k = reconnect, s = read-only scan, r DEV REG LEN (hex)");
}

void runProbeStep() {
  if (!connected || authState != AUTH_READY || millis() - lastCommandAt < 600) return;
  if (manualPending) {
    manualPending = false;
    sendRead(manualProbe);
    return;
  }
  if (sweepRunning) {
    if (sweepIndex < 48) {
      sendRead({ESC_ADDR, (uint8_t)(0x20 + sweepIndex++), 2});
      return;
    }
    if (sweepIndex < 64) {
      uint8_t reg = (uint8_t)(0x30 + sweepIndex++ - 48);
      sendRead({BMS_ADDR, reg, 2});
      return;
    }
    sweepRunning = false;
    Serial.println("Register scan complete");
  }
  if (readIndex < sizeof(readProbes) / sizeof(readProbes[0])) {
    sendRead(readProbes[readIndex++]);
  } else {
    readIndex = 0;
    Serial.println("Telemetry probe cycle complete");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  Serial.println("Ninebot G30/MAX authenticated read-only probe; Serial 115200");
  Serial.println("Close phone app; press scooter power if 0x5C requests confirmation");
}

void loop() {
  readSerialCommands();
  if (!connected) {
    static uint32_t lastScan = 0;
    if (millis() - lastScan >= 1500) {
      lastScan = millis();
      scanAndConnect();
    }
    return;
  }
  servicePendingTx();
  serviceAuth();
  runProbeStep();
}