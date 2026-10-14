#!/usr/bin/env python3
"""Генератор src/shfw_table.h из tools/shfw_registers.csv.

Формат CSV (без заголовка): reg, len, value, mask, name, tag
  reg   - индекс регистра ESC (слово, как в sendRead)
  len   - сколько байт читать/писать (1 или 2; 8 - кривые/триггеры, пропускаются)
  value - пример значения, байты в порядке памяти (первый байт = младший)
  mask  - маска поля, тоже в порядке памяти: "00FF" = поле во втором байте
  name  - описание; "Имя :Вариант" = вариант списка, "(a-b)ед" = диапазон и единица
  tag   - служебная пометка (1s/1d/1e = режим Sport/Drive/Eco)

Запуск:  python tools/gen_shfw_table.py
"""
import csv
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "tools" / "shfw_registers.csv"
DST = ROOT / "src" / "shfw_table.h"

GROUPS = ["Power & limits", "Modes & profiles", "Lights", "Dashboard", "System", "Expert"]
G_POWER, G_MODES, G_LIGHTS, G_DASH, G_SYSTEM, G_EXPERT = range(6)

TYPOS = [("baking", "braking"), ("phpase", "phase"), ("Dubble", "Double"), ("Averge", "Average"),
         ("Avarage", "Average"), ("temprature", "temperature"), ("Supress", "Suppress"),
         ("bake", "brake"), ("brightnes", "brightness"), ("Rrmaining", "Remaining"),
         ("curent", "current"), ("break", "brake"), ("with with", "with")]

# Строки, которые редактировать нельзя/небезопасно/нет понятной шкалы.
SKIP_WORDS = ["spoof", "curve", "custom profile", "off b=1", "cell voltage", "adc resistor"]

# Ручные уточнения: подстрока имени -> (scale, unit, decimals)
SCALE_RULES = [
    ("in 10mA", 0.01, "A", 2),
    ("in 0.1A", 0.1, "A", 1),
    ("10mA*km/H", 10.0, "mA*km/h", 0),
    ("in 0.1 inch", 0.1, "in", 1),
]
# Если диапазон в имени отсутствует или задан словами: подстрока -> (raw_min, raw_max)
RANGE_OVERRIDES = [
    ("min throttle", (30, 210)), ("max throttle", (30, 210)),
    ("min brake value", (30, 210)), ("max brake value", (30, 210)),
    ("wheel size", (60, 120)),            # 6.0-12.0 дюймов, защитные границы
    ("battery capacity", (0, 65535)),
]


def fix_typos(s):
    # \b...\b: чтобы уже исправленное слово (brightness) не «исправлялось» повторно
    for a, b in TYPOS:
        s = re.sub(r"\b" + re.escape(a) + r"\b", b, s)
    return s


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE - тот же алгоритм, что shfwCrc16() в src/shfw.h."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_runs(rows):
    """Сливает слова всех регистров из CSV (адрес в словах, len в байтах) в непрерывные диапазоны.
    Слова, которых нет в таблице, в бэкап не попадают (их не читаем и не пишем)."""
    spans = sorted((reg, reg + (ln + 1) // 2) for reg, ln, *_ in rows)
    runs = []
    for a, b in spans:
        if runs and a <= runs[-1][1]:
            runs[-1][1] = max(runs[-1][1], b)
        else:
            runs.append([a, b])
    return [(a, b - a) for a, b in runs]


def le(hexstr):
    b = bytes.fromhex(hexstr)
    return sum(v << (8 * i) for i, v in enumerate(b))


def ctz(m):
    return (m & -m).bit_length() - 1


def classify(name):
    n = name.lower()
    if "charging" in n:
        return G_SYSTEM
    if any(w in n for w in ("taillight", "headlight", "brake light")):
        return G_LIGHTS
    if "dashboard" in n or "battery bar" in n:
        return G_DASH
    if any(w in n for w in ("suppress", "field", "pwm", "constant k", "throttle value", "brake value",
                            "negative start", "dpc", "bms emulation", "battery capacity",
                            "new max motors", "variable field", "positive battery")):
        return G_EXPERT
    if any(w in n for w in ("mode", "cruise", "profile", "swap button", "s mode only", "beep type", "button")) \
            and "boot beep" not in n:
        return G_MODES
    if any(w in n for w in ("speed", "current", "boost", "autobraking", "braking")):
        return G_POWER
    return G_SYSTEM


def clean_name(raw):
    s = fix_typos(raw.strip().rstrip(",").strip())
    s = re.sub(r"\([^)]*\)\s*[A-Za-z%/*]*", "", s)          # диапазоны и единицы
    s = re.sub(r"\bin (10mA|0\.1A|0\.1 inch|mAh|m)\b", "", s)
    s = re.sub(r"\s*=\s*[\d.]+V", "", s)
    s = re.sub(r"10mA\*km/H", "", s)
    s = re.sub(r"\s+", " ", s).strip(" :,=")
    return s[:1].upper() + s[1:]


def parse_range(raw):
    """Возвращает (lo, hi, unit) в единицах отображения или None."""
    found = None
    for m in re.finditer(r"\(\s*([\d.]+)\s*-\s*([\d.]+)\s*\)\s*([A-Za-z%/*]*)", raw):
        try:
            found = (float(m.group(1)), float(m.group(2)), m.group(3))
        except ValueError:
            pass
    return found


def main():
    rows = []
    with open(SRC, newline="", encoding="utf-8-sig") as f:
        for r in csv.reader(f):
            if len(r) < 5 or not r[0].strip():
                continue
            reg, ln, val, mask, name = r[0].strip(), r[1].strip(), r[2].strip(), r[3].strip(), r[4]
            tag = r[5].strip() if len(r) > 5 else ""
            rows.append((int(reg, 16), int(ln, 16), val, mask, name, tag))

    skipped, warns = [], []
    nums, flags, enum_groups = [], [], {}

    for reg, ln, val, mask, name, tag in rows:
        if ln not in (1, 2) or any(w in name.lower() for w in SKIP_WORDS):
            skipped.append(f"{reg:02X} {name.strip()}")
            continue
        m, v = le(mask), le(val)
        if m == 0:
            skipped.append(f"{reg:02X} {name.strip()} (пустая маска)")
            continue
        low = fix_typos(name)
        if bin(m).count("1") == 1:
            nm = low.strip().rstrip(",").strip()
            nm = re.sub(r"\s*:\s*", ": ", nm)
            nm = re.sub(r"(\w)\(", r"\1 (", nm)
            flags.append(dict(reg=reg, len=ln, mask=m, name=nm[:1].upper() + nm[1:], tag=tag))
        elif ":" in low:
            pre, opt = low.split(":", 1)
            key = (reg, ln, clean_name(pre))
            g = enum_groups.setdefault(key, dict(opts=[], masks=[], tag=tag))
            g["opts"].append((v, m, opt.strip()))
            g["masks"].append(m)
        else:
            nums.append(dict(reg=reg, len=ln, mask=m, raw=name, low=low, val=v, tag=tag))

    items = []   # dict(reg,len,mask,kind,name,group,min,max,scale,dec,unit,opts,tag)

    for f in flags:
        items.append(dict(kind="SK_FLAG", reg=f["reg"], len=f["len"], mask=f["mask"], name=f["name"],
                          group=classify(f["name"]), mn=0, mx=1, scale=1.0, dec=0, unit="", opts=[], tag=f["tag"]))

    for (reg, ln, nm), g in enum_groups.items():
        m = min(g["masks"], key=lambda x: (bin(x).count("1"), x))
        sh = ctz(m)
        seen, opts = set(), []
        for v, om, label in g["opts"]:
            ov = (v & m) >> sh
            if ov in seen:
                continue
            seen.add(ov)
            opts.append((ov, label))
        opts.sort()
        items.append(dict(kind="SK_ENUM", reg=reg, len=ln, mask=m, name=nm, group=classify(nm),
                          mn=0, mx=0, scale=1.0, dec=0, unit="", opts=opts, tag=g["tag"]))

    for n in nums:
        nm = clean_name(n["low"])
        scale, unit, dec = 1.0, "", 0
        for key, sc, un, de in SCALE_RULES:
            if key in n["low"]:
                scale, unit, dec = sc, un, de
        rng = parse_range(n["low"])
        bits = bin(n["mask"]).count("1")
        if rng:
            lo, hi, u = rng
            if u and not unit:
                unit = u
            mn, mx = round(lo / scale), round(hi / scale)
        else:
            mn, mx = 0, (1 << bits) - 1
        for key, (a, b) in RANGE_OVERRIDES:
            if key in n["low"].lower():
                mn, mx = a, b
        if "mAh" in n["low"]:
            unit = "mAh"
        if "in 0.1 inch" in n["low"]:
            unit = "in"
        mx = min(mx, (1 << bits) - 1)
        cur = (n["val"] & n["mask"]) >> ctz(n["mask"])
        if not (mn <= cur <= mx):
            warns.append(f"{n['reg']:02X} {nm}: пример {cur} вне {mn}..{mx} (скорее всего значение из дампа при выключенной функции)")
        items.append(dict(kind="SK_NUM", reg=n["reg"], len=n["len"], mask=n["mask"], name=nm,
                          group=classify(nm), mn=mn, mx=mx, scale=scale, dec=dec, unit=unit, opts=[],
                          tag=n["tag"]))

    # Дубли по пересечению битов одного регистра: оставляем первый.
    uniq = []
    for it in items:
        if any(u["reg"] == it["reg"] and (u["mask"] & it["mask"]) for u in uniq):
            skipped.append(f"{it['reg']:02X} {it['name']} (дубль)")
            continue
        uniq.append(it)
    items = uniq

    # Флаг и число с одинаковым именем: флаг = «включено».
    num_names = {i["name"] for i in items if i["kind"] == "SK_NUM"}
    for it in items:
        if it["kind"] == "SK_FLAG" and it["name"] in num_names:
            it["name"] += " enabled"

    # Одинаковые имена в разных регистрах/битах: добавляем режим из метки 1s/1d/1e.
    from collections import Counter
    cnt = Counter((i["name"], i["kind"]) for i in items)
    mode_names = {"s": "S", "d": "D", "e": "E"}
    for it in items:
        if cnt[(it["name"], it["kind"])] > 1:
            mm = re.fullmatch(r"\d?([sde])", it["tag"])
            if mm:
                it["name"] += f" [{mode_names[mm.group(1)]}]"
    cnt = Counter((i["name"], i["kind"]) for i in items)
    for k, c in cnt.items():
        if c > 1:
            warns.append(f"неоднозначное имя: {k[0]} x{c}")

    items.sort(key=lambda i: (i["group"], i["reg"], i["mask"], i["name"]))

    # ---- вывод ----
    def cstr(s):
        return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'

    opts_out, lines = [], []
    for it in items:
        first, count = len(opts_out), len(it["opts"])
        for v, label in it["opts"]:
            opts_out.append(f'  {{{v}, {cstr(label.strip())}}},')
        lines.append(
            f'  {{{cstr(it["name"])}, 0x{it["reg"]:02X}, {it["len"]}, {it["group"]}, {it["kind"]}, '
            f'0x{it["mask"]:04X}, {it["mn"]}, {it["mx"]}, {it["scale"]}f, {it["dec"]}, {cstr(it["unit"])}, '
            f'{first}, {count}}},')

    with open(DST, "w", encoding="utf-8", newline="\n") as o:
        o.write("// АВТОГЕНЕРАЦИЯ: tools/gen_shfw_table.py из tools/shfw_registers.csv. Не править вручную.\n")
        o.write("#pragma once\n#include \"shfw.h\"\n\n")
        o.write("static const char* const SHFW_GROUP_NAMES[SG_COUNT] = {\n")
        o.write("".join(f"  {cstr(g)},\n" for g in GROUPS) + "};\n\n")
        o.write("static const ShfwOpt SHFW_OPTS[] = {\n" + "\n".join(opts_out) + "\n};\n\n")
        o.write("static const ShfwItem SHFW_ITEMS[] = {\n" + "\n".join(lines) + "\n};\n")
        o.write("static const uint16_t SHFW_ITEM_COUNT = sizeof(SHFW_ITEMS) / sizeof(SHFW_ITEMS[0]);\n")

    runs = build_runs(rows)
    assert all(a + w <= 0x100 and w <= 255 for a, w in runs)
    table_id = crc16(bytes(v for a, w in runs for v in (a, w)))
    with open(DST, "a", encoding="utf-8", newline="\n") as o:
        o.write("\n// Диапазоны регистров для бэкапа/восстановления: адрес в СЛОВАХ, длина тоже в словах.\n")
        o.write("// Читаются/пишутся блоками не больше SHFW_MAX_XFER (0x50) БАЙТ.\n")
        o.write("static const ShfwRun SHFW_RUNS[] = {\n")
        o.write("".join(f"  {{0x{a:02X}, {w}}},\n" for a, w in runs) + "};\n")
        o.write("static const uint8_t SHFW_RUN_COUNT = sizeof(SHFW_RUNS) / sizeof(SHFW_RUNS[0]);\n")
        o.write(f"static const uint16_t SHFW_DUMP_BYTES = {sum(w for a, w in runs) * 2};\n")
        o.write(f"static const uint16_t SHFW_TABLE_ID = 0x{table_id:04X};   // меняется вместе с набором регистров\n")
    print(f"Бэкап: {len(runs)} диапазонов, {sum(w for a, w in runs) * 2} байт, table id {table_id:04X}")

    by_kind = Counter(i["kind"] for i in items)
    print(f"Записано {DST.relative_to(ROOT)}: {len(items)} параметров {dict(by_kind)}, {len(opts_out)} вариантов")
    print(f"Пропущено строк: {len(skipped)}")
    for s in skipped:
        print("  skip:", s)
    for w in warns:
        print("  warn:", w)


if __name__ == "__main__":
    sys.exit(main())
