# Оставлено для обратной совместимости: основной генератор живёт в tools/gen_ru_font.py.
# Запуск: python tools/gen_ru_font.py
from pathlib import Path
import runpy
import sys

script = Path(__file__).resolve().parents[1] / "tools" / "gen_ru_font.py"
if not script.exists():
    raise SystemExit(f"Не найден генератор шрифтов: {script}")

sys.argv[0] = str(script)
runpy.run_path(str(script), run_name="__main__")
