// Хост-тест логики SHFW (без железа):  g++ -std=c++17 -Wall -Wextra -Isrc tools/test_shfw.cpp -o /tmp/test_shfw && /tmp/test_shfw
#include <cstdio>
#include <cstdlib>
#include <string>
#include "../src/shfw_table.h"

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static const ShfwItem* find(const char* name) {
  for (uint16_t i = 0; i < SHFW_ITEM_COUNT; i++) if (!strcmp(SHFW_ITEMS[i].name, name)) return &SHFW_ITEMS[i];
  return nullptr;
}

int main() {
  // --- общие свойства всех параметров ---
  for (uint16_t i = 0; i < SHFW_ITEM_COUNT; i++) {
    const ShfwItem& it = SHFW_ITEMS[i];
    CHECK(it.len == 1 || it.len == 2);
    CHECK(it.mask != 0);
    CHECK(it.group < SG_COUNT);
    if (it.len == 1) CHECK((it.mask & 0xFF00) == 0);          // 1-байтовое чтение не может содержать поле во 2-м байте
    for (uint16_t w : {0x0000, 0xFFFF, 0x5AA5, 0x1234}) {
      uint16_t word = (uint16_t)w;
      if (it.len == 1) word &= 0xFF;
      if (it.kind == SK_FLAG) {
        for (int v = 0; v <= 1; v++) {
          uint16_t n = shfwSet(it, word, v);
          CHECK(shfwGet(it, n) == v);
          CHECK((n & ~it.mask) == (word & ~it.mask));           // чужие биты не тронуты
        }
      } else if (it.kind == SK_ENUM) {
        CHECK(it.optCount >= 2);
        for (uint16_t k = 0; k < it.optCount; k++) {
          uint16_t v = SHFW_OPTS[it.optFirst + k].val;
          uint16_t n = shfwSet(it, word, v);
          CHECK(shfwGet(it, n) == v);
          CHECK((n & ~it.mask) == (word & ~it.mask));
        }
      } else {
        CHECK(it.minV <= it.maxV);
        CHECK(it.maxV <= shfwFieldMax(it));                     // максимум помещается в поле
        for (int32_t v : {it.minV, it.maxV, (it.minV + it.maxV) / 2}) {
          uint16_t n = shfwSet(it, word, (uint32_t)v);
          CHECK(shfwGet(it, n) == v);
          CHECK((n & ~it.mask) == (word & ~it.mask));
        }
      }
    }
  }

  // --- конкретные регистры из CSV ---
  uint8_t b14[2] = {0x03, 0x64};                                // рег. 0x14: старт-скорость | усиление торможения
  const ShfwItem* start = find("Motor Start speed");
  const ShfwItem* boost = find("Brake boost");
  CHECK(start && boost);
  CHECK(start->len == 1 && boost->len == 2);
  CHECK(shfwGet(*start, shfwWordFromBytes(b14, 1)) == 3);
  CHECK(shfwGet(*boost, shfwWordFromBytes(b14, 2)) == 100);
  uint8_t out[2];
  shfwBytesFromWord(shfwSet(*boost, shfwWordFromBytes(b14, 2), 50), 2, out);
  CHECK(out[0] == 0x03 && out[1] == 50);                        // байт 0 сохранён
  shfwBytesFromWord(shfwSet(*start, shfwWordFromBytes(b14, 1), 7), 1, out);
  CHECK(out[0] == 7);

  const ShfwItem* lim = find("Current Limit");                  // рег. 0x7A: E8 03 = 1000 -> 10.00 A
  uint8_t b7a[2] = {0xE8, 0x03};
  char buf[64];
  CHECK(lim && shfwGet(*lim, shfwWordFromBytes(b7a, 2)) == 1000);
  shfwFormat(*lim, SHFW_OPTS, 1000, false, buf, sizeof(buf));
  CHECK(!strcmp(buf, "10.00 A"));

  const ShfwItem* wheel = find("Wheel size");
  shfwFormat(*wheel, SHFW_OPTS, 93, false, buf, sizeof(buf));
  CHECK(!strcmp(buf, "9.3 in"));

  const ShfwItem* vf = find("Variable field current");
  shfwFormat(*vf, SHFW_OPTS, 250, false, buf, sizeof(buf));
  CHECK(!strcmp(buf, "2500 mA*km/h"));

  // флаг "Disable Boot beep": рег. 0x16, байт 0 бит 1
  const ShfwItem* beep = find("Disable Boot beep");
  CHECK(beep && beep->reg == 0x16 && beep->mask == 0x0002);
  CHECK(shfwSet(*beep, 0x0408, 1) == 0x040A);
  CHECK(shfwSet(*beep, 0x040A, 0) == 0x0408);

  // список: "Default Mode" Drive/Eco/Sport/Last, шаг по кругу
  const ShfwItem* dm = find("Default Mode");
  CHECK(dm && dm->optCount == 4);
  CHECK(shfwEnumStep(*dm, SHFW_OPTS, 3, +1) == 0);
  CHECK(shfwEnumStep(*dm, SHFW_OPTS, 0, -1) == 3);
  CHECK(!strcmp(shfwOptLabel(*dm, SHFW_OPTS, 2), "Sport"));

  // "Profile Activation Beep type": рег. 0x1B, значение во 2-м байте; байт 0 (Default Mode) не затрагивается
  const ShfwItem* pb = find("Profile Activation Beep type");
  uint8_t b1b[2] = {0x03, 0x00};
  CHECK(pb);
  shfwBytesFromWord(shfwSet(*pb, shfwWordFromBytes(b1b, 2), 4), 2, out);
  CHECK(out[0] == 0x03 && out[1] == 0x04);

  // clamp
  const ShfwItem* sl = find("Speed limit [S]");
  CHECK(shfwClamp(*sl, SHFW_OPTS, 0) == 1);
  CHECK(shfwClamp(*sl, SHFW_OPTS, 500) == 100);
  CHECK(shfwClamp(*dm, SHFW_OPTS, 9) == 0);                     // неизвестный вариант -> первый
  CHECK(shfwClamp(*beep, SHFW_OPTS, 5) == 1);

  // у каждого имени параметра - уникальность внутри группы
  for (uint16_t i = 0; i < SHFW_ITEM_COUNT; i++)
    for (uint16_t j = i + 1; j < SHFW_ITEM_COUNT; j++)
      CHECK(strcmp(SHFW_ITEMS[i].name, SHFW_ITEMS[j].name) != 0);

  // --- бэкап: блоки не больше 0x50 байт, адрес в словах, длина в байтах ---
  {
    static ShfwChunk ch[64];
    uint16_t n = shfwBuildChunks(SHFW_RUNS, SHFW_RUN_COUNT, ch, 64);
    CHECK(n > 0);
    uint16_t total = 0;
    for (uint16_t i = 0; i < n; i++) {
      CHECK(ch[i].bytes >= 2 && ch[i].bytes <= SHFW_MAX_XFER);
      CHECK(ch[i].bytes % 2 == 0);                              // целое число слов
      CHECK(ch[i].off == total);                                // дамп без дыр
      total += ch[i].bytes;
    }
    CHECK(total == SHFW_DUMP_BYTES);
    // каждый регистр таблицы лежит внутри какого-то диапазона
    for (uint16_t i = 0; i < SHFW_ITEM_COUNT; i++) {
      bool in = false;
      for (uint8_t r = 0; r < SHFW_RUN_COUNT; r++)
        if (SHFW_ITEMS[i].reg >= SHFW_RUNS[r].reg && SHFW_ITEMS[i].reg < SHFW_RUNS[r].reg + SHFW_RUNS[r].words) in = true;
      CHECK(in);
    }
    // длинный диапазон режется: 100 слов = 200 байт -> 0x50 + 0x50 + 0x28
    ShfwRun big = {0x00, 100};
    ShfwChunk c2[8];
    CHECK(shfwBuildChunks(&big, 1, c2, 8) == 3);
    CHECK(c2[0].bytes == 0x50 && c2[1].bytes == 0x50 && c2[2].bytes == 0x28);
    CHECK(c2[1].reg == 0x28 && c2[2].reg == 0x50);              // шаг адреса = байты / 2
    CHECK(shfwBuildChunks(&big, 1, c2, 2) == 0);                // не хватило места
  }
  const uint8_t crcv[] = {'1','2','3','4','5','6','7','8','9'};
  CHECK(shfwCrc16(crcv, 9) == 0x29B1);                          // эталон CRC-16/CCITT-FALSE
  {
    // кадр запроса версии 5A A5 00 3E 20 39 00 68 FF: crc = ~сумма(len..arg), младший байт первым
    const uint8_t f[] = {0x00, 0x3E, 0x20, 0x39, 0x00};
    unsigned sum = 0;
    for (uint8_t b : f) sum += b;
    uint16_t crc = (uint16_t)(~sum);
    CHECK((crc & 0xFF) == 0x68 && (crc >> 8) == 0xFF);
  }
  uint8_t hx[2] = {0x0A, 0xBC};
  char hb[8];
  shfwHex(hx, 2, hb, sizeof(hb));
  CHECK(!strcmp(hb, "0ABC"));

  printf(fails ? "FAILED: %d\n" : "OK: all checks passed (%d items)\n", fails ? fails : (int)SHFW_ITEM_COUNT);
  return fails ? 1 : 0;
}
