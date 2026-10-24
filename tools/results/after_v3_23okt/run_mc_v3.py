# Run the 8 offline Monte Carlo scenarios of after_v2_20okt with the v3 code, split per level
# into parallel processes, then merge each scenario into <out>/<scenario>/{montecarlo.json,log.txt}
# in the same format tools/offline_sim.py writes.
import json, os, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor

ROOT = r"C:\Users\POGGY\Downloads\barengpoggy\KiboRPC7"
OUT = os.path.join(ROOT, "tools", "results", "after_v3_23okt")
SIM = os.path.join(ROOT, "tools", "offline_sim.py")
PARTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mc_parts")
ALL = "easy,normal,hard,veryhard"
SCEN = {
    "nominal":     (ALL, 25, "fast,safe,baseline", []),
    "stressA":     (ALL, 20, "fast,safe", ["--bias", "0.005", "--noise", "0.02", "--delay", "0.2", "--dist", "0.003"]),
    "bias3noise3": (ALL, 20, "fast", ["--bias", "0.006", "--noise", "0.036", "--dist", "0.0045"]),
    "fan08":       (ALL, 20, "fast,safe", ["--fan_scale", "0.8"]),
    "fan07":       ("veryhard,hard", 20, "fast,safe", ["--fan_scale", "0.7"]),
    "extreme":     ("veryhard", 20, "fast,safe", ["--bias", "0.03", "--noise", "0.156", "--dist", "0.02"]),
    "nowrench":    (ALL, 10, "fast", ["--ignore_user_wrench"]),
    "delay05":     (ALL, 20, "fast,safe", ["--delay", "0.5"]),
}


def job(args):
    name, level, seeds, strats, extra = args
    part = os.path.join(PARTS, name, level)
    os.makedirs(part, exist_ok=True)
    cmd = [sys.executable, SIM, "--levels", level, "--seeds", str(seeds), "--strategies", strats, "--out", part] + extra
    t = time.time()
    with open(os.path.join(part, "log.txt"), "w") as f:
        rc = subprocess.call(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=ROOT)
    return name, level, rc, time.time() - t, cmd


def summary(rows, levels, strats):
    out = ["", "=== SUMMARY (mean over seeds) ===",
           "%-9s %-8s %8s %8s %8s %8s %8s" % ("level", "strategy", "T[s]", "in_time", "goal%", "Lv3%", "score")]
    for level in levels:
        for strat in strats:
            rs = [r for r in rows if r["level"] == level and r["strategy"] == strat]
            if not rs:
                continue
            n = float(len(rs))
            T = sum(min(r["T"], 999) for r in rs) / n
            it = sum(1 for r in rs if r["in_time"]) / n * 100
            g = sum(1 for r in rs if r["goal"]) / n * 100
            l3 = sum(r["n_lv3"] / float(r["n_rules"]) for r in rs) / n * 100
            sc = sum(r["score"] for r in rs) / n
            out.append("%-9s %-8s %8.1f %7.0f%% %7.0f%% %7.0f%% %8.2f" % (level, strat, T, it, g, l3, sc))
    return out


def main():
    jobs = []
    for name, (levels, seeds, strats, extra) in SCEN.items():
        for lv in levels.split(","):
            jobs.append((name, lv, seeds, strats, extra))
    t0 = time.time()
    with ThreadPoolExecutor(max_workers=int(os.environ.get("MC_WORKERS", "12"))) as ex:
        res = list(ex.map(job, jobs))
    for r in res:
        print("%-12s %-9s rc=%d %.0fs" % r[:4])
    cmds = {}
    for name, (levels, seeds, strats, extra) in SCEN.items():
        rows, lines = [], []
        for lv in levels.split(","):
            part = os.path.join(PARTS, name, lv)
            rows += json.load(open(os.path.join(part, "montecarlo.json")))
            with open(os.path.join(part, "log.txt")) as f:
                for ln in f:
                    if "seed=" in ln:
                        lines.append(ln.rstrip("\n"))
        d = os.path.join(OUT, name)
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "montecarlo.json"), "w") as f:
            json.dump(rows, f, indent=1)
        with open(os.path.join(d, "log.txt"), "w") as f:
            f.write("\n".join(lines + summary(rows, levels.split(","), strats.split(","))) + "\n")
        cmds[name] = "python tools/offline_sim.py --levels %s --seeds %d --strategies %s %s" % (levels, seeds, strats, " ".join(extra))
    with open(os.path.join(OUT, "COMMANDS.txt"), "w") as f:
        f.write("# v3 (23 Oct 2026) offline Monte Carlo, same scenarios/seeds as after_v2_20okt.\n")
        f.write("# Each scenario was run split per level in parallel (scratch runner), then merged; results are\n")
        f.write("# identical to one sequential run because every (level, strategy, seed) run is independent.\n")
        for k, v in cmds.items():
            f.write("%-12s %s\n" % (k, v.strip()))
    print("total %.0fs" % (time.time() - t0))


if __name__ == "__main__":
    main()
