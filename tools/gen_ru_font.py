"""Generate TFT_eSPI GFX fonts covering Latin and Cyrillic characters."""

from pathlib import Path
import os
import sys

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError as exc:
    raise SystemExit(
        "Pillow is required. Install it with: python -m pip install Pillow"
    ) from exc


FIRST, LAST = 0x20, 0x451
CODES = list(range(0x20, 0x7F)) + [0x401] + list(range(0x410, 0x450)) + [0x451]
SIZES = [("RU1", 10), ("RU2", 13), ("RU4", 20)]
REQUIRED_GLYPHS = "ЁАЯёая"


def font_supports_cyrillic(path):
    try:
        font = ImageFont.truetype(str(path), 16, layout_engine=ImageFont.Layout.BASIC)
    except OSError:
        return False

    missing = [
        (font.getbbox(char), bytes(font.getmask(char)))
        for char in ("\uFFFD", "?")
    ]
    for char in REQUIRED_GLYPHS:
        signature = (font.getbbox(char), bytes(font.getmask(char)))
        if signature in missing:
            return False
    return True


def find_font_path():
    candidates = []
    configured = os.environ.get("OPENDASH_FONT_FILE")
    if configured:
        candidates.append(Path(configured).expanduser())

    candidates.extend(
        Path(path)
        for path in (
            "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
            "C:/Windows/Fonts/arial.ttf",
            "C:/Windows/Fonts/calibri.ttf",
            "C:/Windows/Fonts/segoeui.ttf",
        )
    )

    font_dirs = [
        Path("/usr/share/fonts"),
        Path("/usr/local/share/fonts"),
        Path.home() / ".local" / "share" / "fonts",
        Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts",
        Path.home() / "AppData" / "Local" / "Microsoft" / "Windows" / "Fonts",
    ]
    preferred = ("dejavu", "arial", "calibri", "liberation", "noto", "segoe")
    for directory in font_dirs:
        if not directory.is_dir():
            continue
        files = []
        for suffix in ("*.ttf", "*.otf", "*.ttc"):
            files.extend(directory.rglob(suffix))
        files.sort(key=lambda path: (not any(x in path.name.lower() for x in preferred), str(path).lower()))
        candidates.extend(files)

    seen = set()
    for candidate in candidates:
        key = str(candidate.resolve()).casefold()
        if key in seen:
            continue
        seen.add(key)
        if candidate.is_file() and font_supports_cyrillic(candidate):
            return candidate

    raise FileNotFoundError(
        "No installed TTF/OTF font with Cyrillic support was found. "
        "Install DejaVu Sans or set OPENDASH_FONT_FILE to a font file."
    )


TTF = find_font_path()


def build(size):
    font = ImageFont.truetype(str(TTF), size, layout_engine=ImageFont.Layout.BASIC)
    ascent, descent = font.getmetrics()
    bitmap = bytearray()
    glyphs = {}

    for codepoint in CODES:
        char = chr(codepoint)
        advance = int(round(font.getlength(char)))
        left, top, right, bottom = font.getbbox(char, anchor="ls")
        width, height = right - left, bottom - top
        if char == " " or width <= 0 or height <= 0:
            glyphs[codepoint] = (len(bitmap), 0, 0, advance, 0, 0)
            continue

        image = Image.new("L", (width, height), 0)
        drawer = ImageDraw.Draw(image)
        drawer.fontmode = "1"
        drawer.text((-left, -top), char, font=font, fill=255, anchor="ls")
        pixels = [
            1 if image.getpixel((x, y)) > 127 else 0
            for y in range(height)
            for x in range(width)
        ]
        offset = len(bitmap)
        for i in range(0, len(pixels), 8):
            byte = 0
            for bit in range(8):
                value = pixels[i + bit] if i + bit < len(pixels) else 0
                byte = (byte << 1) | value
            bitmap.append(byte)
        glyphs[codepoint] = (offset, width, height, advance, left, top)

    return bitmap, glyphs, ascent + descent


def emit(output, name, size):
    bitmap, glyphs, y_advance = build(size)
    output.append(f"// {name}: {TTF.name} {size}px, 1-bit")
    output.append(f"static const uint8_t {name}_bmp[] PROGMEM = {{")
    for i in range(0, len(bitmap), 16):
        output.append("  " + ",".join(f"0x{value:02X}" for value in bitmap[i:i + 16]) + ",")
    output.append("};")
    output.append(f"static const GFXglyph {name}_glyph[] PROGMEM = {{")
    for codepoint in range(FIRST, LAST + 1):
        glyph = glyphs.get(codepoint, (0, 0, 0, 0, 0, 0))
        output.append(f"  {{{glyph[0]},{glyph[1]},{glyph[2]},{glyph[3]},{glyph[4]},{glyph[5]}}},")
    output.append("};")
    output.append(
        f"static const GFXfont {name} PROGMEM = "
        f"{{(uint8_t*){name}_bmp,(GFXglyph*){name}_glyph,"
        f"0x{FIRST:02X},0x{LAST:03X},{y_advance}}};\n"
    )
    return len(bitmap)


def main():
    repo_root = Path(__file__).resolve().parents[1]
    output_path = Path(sys.argv[1]).expanduser() if len(sys.argv) > 1 else repo_root / "src" / "ru_fonts.h"
    if not output_path.is_absolute():
        output_path = Path.cwd() / output_path
    output_path.parent.mkdir(parents=True, exist_ok=True)

    output = [
        "// Generated by tools/gen_ru_font.py. Do not edit manually.",
        f"// Latin and Cyrillic GFX fonts generated from {TTF.name}.",
        "#pragma once",
        "#include <TFT_eSPI.h>",
        "",
    ]
    sizes = {name: emit(output, name, size) for name, size in SIZES}
    output_path.write_text("\n".join(output), encoding="utf-8")
    print(f"Font: {TTF}")
    print(f"Written: {output_path}")
    print(f"Bitmap bytes: {sizes}")


if __name__ == "__main__":
    main()
