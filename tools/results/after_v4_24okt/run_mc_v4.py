# Offline Monte Carlo tuning harness (scratch). Runs tools/offline_sim.simulate() for a set of
# CONFIG overrides over the same seeds as tools/results/after_v3_23okt and prints mean / p10 / min.
# usage: python tune.py <name> '<json cfg overrides>' [scenario] [levels] [seeds] [user_script]
import json, os, sys, time
from multiprocessing import Pool

ROOT = r"C:\Users\POGGY\Downloads\barengpoggy\KiboRPC7"
sys.path.insert(0, os.path.join(ROOT, "tools"))
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tuning")

SCEN = {
    "nominal": {},
    "stressA": {"bias_force": 0.005, "fan_noise": 0.02, "nav_delay": 0.2, "dist_force": 0.003},
    "bias3noise3": {"bias_force": 0.006, "fan_noise": 0.036, "dist_force": 0.0045},
    "fan08": {"fan_scale": 0.8},
    "fan07": {"fan_scale": 0.7},
    "extreme": {"bias_force": 0.03, "fan_noise": 0.156, "dist_force": 0.02},
    "nowrench": {"ignore_user_wrench": True},
    "delay05": {"nav_delay": 0.5},
}
BASE_WK = {"bias_force": 0.002, "fan_noise": 0.012, "nav_delay": 0.1, "dist_force": 0.0015,
           "fan_scale": 1.0, "ignore_user_wrench": False}

_sim = None


def _init(user_script):
    global _sim
    import offline_sim as osim
    if user_script:
        osim.U = osim.load_user_module(user_script)
        osim.P = osim.U.CTL_PARAMS
    _sim = osim


def job(a):
    level, seed, cfg, wk = a
    res, api = _sim.simulate(level, seed, "fast", cfg=cfg, world_kw=wk)
    warns = [t for (_, lv, t) in api.logs if lv == "warn" and not t.startswith("SAFE STOP (mission finished)")]
    res["warns"] = warns[:12]
    res.pop("levels", None)
    return res


def main():
    name = sys.argv[1]
    cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    scen = sys.argv[3] if len(sys.argv) > 3 else "nominal"
    levels = (sys.argv[4] if len(sys.argv) > 4 else "easy,normal,hard,veryhard").split(",")
    nseeds = int(sys.argv[5]) if len(sys.argv) > 5 else 25
    script = sys.argv[6] if len(sys.argv) > 6 else None
    wk = dict(BASE_WK)
    wk.update(SCEN[scen])
    jobs = [(lv, s, cfg, wk) for lv in levels for s in range(1, nseeds + 1)]
    t0 = time.time()
    with Pool(int(os.environ.get("W", "8")), initializer=_init, initargs=(script,)) as p:
        rows = p.map(job, jobs, chunksize=1)
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "%s__%s.json" % (name, scen)), "w") as f:
        json.dump({"cfg": cfg, "scen": scen, "script": script, "rows": rows}, f, indent=1)
    print("%s [%s] cfg=%s (%.0fs)" % (name, scen, json.dumps(cfg), time.time() - t0))
    for lv in levels:
        rs = [r for r in rows if r["level"] == lv]
        sc = sorted(r["score"] for r in rs)
        n = len(sc)
        mean = sum(sc) / n
        p10 = sc[max(0, int(0.1 * n) - 1)] if n >= 10 else sc[0]
        sd = (sum((x - mean) ** 2 for x in sc) / n) ** 0.5
        T = sum(min(r["T"], 999) for r in rs) / n
        goal = sum(1 for r in rs if r["goal"]) * 100.0 / n
        lv3 = sum(r["n_lv3"] / float(r["n_rules"]) for r in rs) * 100.0 / n
        late = sum(1 for r in rs if not r["in_time"])
        fault = sum(1 for r in rs if r.get("fault"))
        warn = sum(1 for r in rs if r.get("warns"))
        print("  %-9s mean=%6.2f p10=%6.2f min=%6.2f sd=%5.2f T=%6.1f goal=%3.0f%% lv3=%5.1f%% late=%d fault=%d warnruns=%d" % (
            lv, mean, p10, sc[0], sd, T, goal, lv3, late, fault, warn))


if __name__ == "__main__":
    main()
