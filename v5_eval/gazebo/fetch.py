"""fetch.py N [N ...]: copy each GB10 run's program log, journal and truth poses (+ my uptime log) here."""
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
H = "promaxgb10-6c88"
for n in sys.argv[1:]:
    d = HERE / "runs" / f"v{n}"
    (d / "journal").mkdir(parents=True, exist_ok=True)
    r = f"{H}:dronehacks-2026-gb10/artifacts/gb10-sky-beta-course-v{n}"
    subprocess.run(["scp", "-q", "-o", "BatchMode=yes", f"{r}/native-program.log", f"{r}/truth-poses.csv", str(d)], check=False)
    subprocess.run(["scp", "-q", "-o", "BatchMode=yes", f"{r}/journal/journal.jsonl", str(d / "journal")], check=False)
subprocess.run(["scp", "-q", "-o", "BatchMode=yes", f"{H}:dronehacks-2026-gb10/jolie_flyx/v5_uptime.log", str(HERE / "runs")], check=False)
print("fetched", sys.argv[1:])
