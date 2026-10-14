// Логика редактирования настроек SHFW: без зависимостей от Arduino, чтобы тестировать на хосте
// (tools/test_shfw.cpp). Таблица параметров генерируется в shfw_table.h.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum ShfwKind : uint8_t { SK_NUM, SK_FLAG, SK_ENUM };
enum ShfwGroup : uint8_t { SG_POWER, SG_MODES, SG_LIGHTS, SG_DASH, SG_SYSTEM, SG_EXPERT, SG_COUNT };

struct ShfwOpt { uint16_t val; const char* label; };

struct ShfwItem {
  const char* name;
  uint8_t reg;        // индекс регистра ESC (слово)
  uint8_t len;        // сколько байт читать/писать: 1 или 2
  uint8_t group;
  uint8_t kind;
  uint16_t mask;      // маска поля в 16-битном слове (байт 0 = младший)
  int32_t minV, maxV; // границы в «сырых» единицах (для SK_NUM)
  float scale;        // отображаемое значение = raw * scale
  uint8_t decimals;
  const char* unit;
  uint16_t optFirst, optCount;   // варианты списка в SHFW_OPTS (для SK_ENUM)
};

static inline uint8_t shfwShift(uint16_t m) {
  uint8_t s = 0;
  while (m && !(m & 1)) { m >>= 1; s++; }
  return s;
}

static inline uint16_t shfwWordFromBytes(const uint8_t* b, uint8_t len) {
  return len >= 2 ? (uint16_t)(b[0] | (b[1] << 8)) : (uint16_t)b[0];
}

static inline void shfwBytesFromWord(uint16_t w, uint8_t len, uint8_t* b) {
  b[0] = (uint8_t)(w & 0xFF);
  if (len >= 2) b[1] = (uint8_t)(w >> 8);
}

// Значение поля внутри слова регистра.
static inline uint16_t shfwGet(const ShfwItem& it, uint16_t word) {
  return (uint16_t)((word & it.mask) >> shfwShift(it.mask));
}

// Новое слово регистра: меняются только биты поля, остальные сохраняются.
static inline uint16_t shfwSet(const ShfwItem& it, uint16_t word, uint32_t v) {
  return (uint16_t)((word & ~it.mask) | ((v << shfwShift(it.mask)) & it.mask));
}

static inline uint16_t shfwFieldMax(const ShfwItem& it) {
  return (uint16_t)(it.mask >> shfwShift(it.mask));
}

// Допустимое значение для записи: NUM - в границах, FLAG - 0/1, ENUM - ближайший существующий вариант.
static inline int32_t shfwClamp(const ShfwItem& it, const ShfwOpt* opts, int32_t v) {
  if (it.kind == SK_FLAG) return v ? 1 : 0;
  if (it.kind == SK_ENUM) {
    for (uint16_t i = 0; i < it.optCount; i++) if (opts[it.optFirst + i].val == v) return v;
    return opts[it.optFirst].val;
  }
  if (v < it.minV) v = it.minV;
  if (v > it.maxV) v = it.maxV;
  return v;
}

// Следующий/предыдущий вариант списка (по кругу); dir = +1 / -1.
static inline int32_t shfwEnumStep(const ShfwItem& it, const ShfwOpt* opts, int32_t cur, int dir) {
  int idx = 0;
  for (uint16_t i = 0; i < it.optCount; i++) if (opts[it.optFirst + i].val == cur) { idx = i; break; }
  idx = (idx + dir + it.optCount) % it.optCount;
  return opts[it.optFirst + idx].val;
}

static inline const char* shfwOptLabel(const ShfwItem& it, const ShfwOpt* opts, int32_t v) {
  for (uint16_t i = 0; i < it.optCount; i++) if (opts[it.optFirst + i].val == v) return opts[it.optFirst + i].label;
  return nullptr;
}

// Текст значения для экрана. ru - русские «ВКЛ/ВЫКЛ».
static inline void shfwFormat(const ShfwItem& it, const ShfwOpt* opts, int32_t raw, bool ru, char* out, size_t n) {
  if (it.kind == SK_FLAG) {
    snprintf(out, n, "%s", raw ? (ru ? "ВКЛ" : "ON") : (ru ? "ВЫКЛ" : "OFF"));
  } else if (it.kind == SK_ENUM) {
    const char* l = shfwOptLabel(it, opts, raw);
    if (l) snprintf(out, n, "%s", l); else snprintf(out, n, "?%ld", (long)raw);
  } else {
    snprintf(out, n, "%.*f%s%s", it.decimals, raw * it.scale, it.unit[0] ? " " : "", it.unit);
  }
}

// ---------------------------------------------------------------------
//  Бэкап / восстановление. Как и в регистрах Ninebot: адрес - в СЛОВАХ, длина - в БАЙТАХ,
//  за одну операцию чтения или записи не больше SHFW_MAX_XFER байт.
// ---------------------------------------------------------------------
#define SHFW_MAX_XFER 0x50
#define SHFW_VER_MAX  32          // сколько байт ответа на запрос версии хранится

struct ShfwRun { uint8_t reg; uint8_t words; };                 // диапазон: адрес и длина в словах
struct ShfwChunk { uint8_t reg; uint8_t bytes; uint16_t off; }; // блок передачи и смещение в дампе

// Режет диапазоны на блоки <= SHFW_MAX_XFER байт. Возвращает число блоков (0 при нехватке места).
static inline uint16_t shfwBuildChunks(const ShfwRun* runs, uint8_t nRuns, ShfwChunk* out, uint16_t maxOut) {
  uint16_t n = 0, off = 0;
  for (uint8_t r = 0; r < nRuns; r++) {
    uint16_t bytes = (uint16_t)runs[r].words * 2, done = 0;
    while (done < bytes) {
      uint16_t c = bytes - done;
      if (c > SHFW_MAX_XFER) c = SHFW_MAX_XFER;
      if (n >= maxOut) return 0;
      out[n].reg = (uint8_t)(runs[r].reg + done / 2);
      out[n].bytes = (uint8_t)c;
      out[n].off = off;
      n++;
      done += c;
      off += c;
    }
  }
  return n;
}

// CRC-16/CCITT-FALSE (тот же алгоритм в tools/gen_shfw_table.py).
static inline uint16_t shfwCrc16(const uint8_t* d, size_t n, uint16_t crc = 0xFFFF) {
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

static inline void shfwHex(const uint8_t* d, uint8_t n, char* out, size_t cap) {
  size_t o = 0;
  for (uint8_t i = 0; i < n && o + 3 <= cap; i++) o += snprintf(out + o, cap - o, "%02X", d[i]);
  if (!n && cap) out[0] = 0;
}
