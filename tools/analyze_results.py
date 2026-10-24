#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Summarize official simulator outputs (local batch folders or files downloaded from the
Web simulator) and calibrate the timing model.

    python tools/analyze_results.py <folder>

Looks recursively for result.json (judge output) and for our program's log lines
("leg cpN ... pred=..s actual=..s level=N") inside any *.log / *.txt file.
Prints per-run results and the mean (actual - predicted) leg time: add that delta to
CONFIG["LEG_OVERHEAD_S"] (SAFE mode) or CONFIG["FAST_OVERHEAD_S"] (FAST mode).
"""
import json
import os
import re
import sys

LEG_RE = re.compile(r"leg cp(\d+)\s+(\S+)\s+pred=\s*([\d.]+)s actual=\s*([\d.]+)s level=(\d)")


def main(folder):
    runs, deltas = [], {"fast": [], "safe": []}
    for dirpath, _, files in os.walk(folder):
        for fn in files:
            path = os.path.join(dirpath, fn)
            if fn == "result.json":
                try:
                    d = json.load(open(path, encoding="utf-8"))
                    jr = d.get("JudgeReport", {})
                    cps = jr.get("checkpoint_detail", [])
                    runs.append({
                        "run": os.path.relpath(dirpath, folder),
                        "illegal": d.get("Illegal"),
                        "duration": (jr.get("mission") or {}).get("duration_sec"),
                        "passed": sum(1 for c in cps if c.get("success")),
                        "lv3": sum(1 for c in cps if c.get("level") == 3),
                        "n": len(cps),
                        "goal": (jr.get("goal") or {}).get("success"),
                    })
                except Exception as exc:
                    print("skip %s: %s" % (path, exc))
            elif fn.endswith((".log", ".txt")):
                try:
                    for line in open(path, encoding="utf-8", errors="ignore"):
                        m = LEG_RE.search(line)
                        if m:
                            mode = "fast" if m.group(2) == "fast" else "safe"
                            deltas[mode].append(float(m.group(4)) - float(m.group(3)))
                except Exception:
                    pass
    runs.sort(key=lambda r: r["run"])
    print("%-28s %8s %7s %5s %5s %6s" % ("run", "time[s]", "passed", "Lv3", "goal", "illegal"))
    for r in runs:
        print("%-28s %8s %4d/%-2d %5d %5s %6s" % (r["run"][:28], r["duration"], r["passed"], r["n"], r["lv3"], r["goal"], r["illegal"]))
    if runs:
        ok = [r for r in runs if r["duration"] is not None]
        print("\nruns=%d  mean time=%.1fs  mean Lv3 share=%.0f%%  goal=%.0f%%" % (
            len(runs), sum(r["duration"] for r in ok) / max(1, len(ok)),
            100.0 * sum(r["lv3"] / float(max(1, r["n"])) for r in runs) / len(runs),
            100.0 * sum(1 for r in runs if r["goal"]) / len(runs)))
    for mode, ds in deltas.items():
        if ds:
            mean = sum(ds) / len(ds)
            key = "FAST_OVERHEAD_S" if mode == "fast" else "LEG_OVERHEAD_S"
            print("%s legs: %d, mean(actual - predicted) = %+.2fs -> adjust CONFIG['%s'] by %+.2f" % (
                mode, len(ds), mean, key, mean))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
