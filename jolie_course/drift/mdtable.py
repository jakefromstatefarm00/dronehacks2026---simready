import json, sys
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
import analyze

files, arms = sys.argv[1].split(","), sys.argv[2].split(",")
out = analyze.summarise(analyze.load(files), arms)
names = {"base": "base", "drift": "drift (Jolie)", "S7": "safe"}
hdr = "| cell | n | " + " | ".join(f"{names.get(a, a)}: done % / contact runs / mean min clear (worst) / time s" for a in arms)
hdr += " | " + " | ".join(f"{names.get(a, a)} vs base: worse / better" for a in arms[1:]) + " | correction / drift at end, residual (" + names.get(arms[1], arms[1]) + ") |"
print(hdr)
print("|" + "---|" * (2 + len(arms) + len(arms) - 1 + 1))
for l in out:
    cells = []
    for a in arms:
        s = l["arms"][a]
        t = "-" if s["time"] != s["time"] else f"{s['time']:.0f}"
        cells.append(f"{s['complete']:.0f} / {s['any_contact']} / {s['min_clear']:.2f} ({s['worst_clear']:.2f}) / {t}")
    pair = [f"{l['arms'][a]['worse']} / {l['arms'][a]['better']}" for a in arms[1:]]
    s1 = l["arms"][arms[1]]
    corr = f"{s1['align']:.2f} / {s1['drift_end']:.2f}, {s1['resid_end']:.3f}" + (f" (capped {s1['capped']})" if s1['capped'] else "")
    print(f"| {l['cell']} | {l['n']} | " + " | ".join(cells) + " | " + " | ".join(pair) + f" | {corr} |")
