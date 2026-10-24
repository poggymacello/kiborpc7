"""Check every official-sim run folder: result.json + program log criteria + score model."""
import json
import os
import re
import sys

root = sys.argv[1]
print("%-26s %-7s %-9s %-5s %-5s %-8s %-6s %-5s %-5s %-5s %-5s %s" % (
    "run", "Illegal", "cps", "Lv3", "goal", "T[s]", "score", "fastL", "SAFEsw", "SSfin", "crash", "legs a-p"))
for name in sorted(os.listdir(root)):
    d = os.path.join(root, name)
    rj = os.path.join(d, "judge_module_logs", "result.json")
    lg = os.path.join(d, "user_programs_logs", "user_program_output.log")
    if not os.path.isfile(rj):
        continue
    r = json.load(open(rj, encoding="utf-8"))
    jr = r["JudgeReport"]
    cps = jr["checkpoint_detail"]
    T = jr["mission"]["duration_sec"]
    ok = sum(1 for c in cps if c["success"])
    lv3 = sum(1 for c in cps if c["level"] == 3)
    goal = jr["goal"]["success"]
    base = sum({3: 3, 2: 2, 1: 1}.get(c["level"], 0) for c in cps if c["success"])
    if ok == len(cps) and goal:
        base += 3
    score = base * (1 + max(0.0, 300 - T) / 300.0) if T is not None and T <= 300 else 0.0
    txt = open(lg, encoding="utf-8", errors="ignore").read()
    fastlegs = len(re.findall(r"leg -> cp \d+ \(fast\)", txt))
    sw = txt.count("switching to SAFE")
    ssf = txt.count("SAFE STOP (mission finished)")
    crash = txt.count("mission crashed") + txt.count("deadline watchdog")
    d_ = [float(a) - float(p) for p, a in re.findall(r"leg cp\d+\s+fast\s+pred=\s*([\d.]+)s actual=\s*([\d.]+)s level=3", txt)]
    mean = (sum(d_) / len(d_)) if d_ else float("nan")
    print("%-26s %-7s %3d/%-5d %-5d %-5s %-8s %-6.2f %-5d %-6d %-5d %-5d %+.2f (n=%d)" % (
        name, r["Illegal"], ok, len(cps), lv3, goal, T, score, fastlegs, sw, ssf, crash, mean, len(d_)))
