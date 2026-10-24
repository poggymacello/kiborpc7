import json, os
R = r"C:\Users\POGGY\Downloads\barengpoggy\KiboRPC7\tools\results"
A, B = os.path.join(R, "after_v2_20okt"), os.path.join(R, "after_v3_23okt")
tot = same = 0
lines = ["| Skenario | Level | Strategi | n | v2 skor | v3 skor | v2 goal% / Lv3% | v3 goal% / Lv3% | per-run identik |", "|---|---|---|---|---|---|---|---|---|"]
for sc in ["nominal", "stressA", "bias3noise3", "fan08", "fan07", "extreme", "nowrench", "delay05"]:
    a = json.load(open(os.path.join(A, sc, "montecarlo.json")))
    b = json.load(open(os.path.join(B, sc, "montecarlo.json")))
    key = lambda r: (r["level"], r["strategy"], r["seed"])
    da, db = dict((key(r), r) for r in a), dict((key(r), r) for r in b)
    assert set(da) == set(db), sc
    groups = []
    for r in a:
        g = (r["level"], r["strategy"])
        if g not in groups:
            groups.append(g)
    for g in groups:
        ks = [k for k in da if k[:2] == g]
        n = float(len(ks))
        def agg(d):
            return (sum(d[k]["score"] for k in ks) / n, sum(1 for k in ks if d[k]["goal"]) / n * 100,
                    sum(d[k]["n_lv3"] / float(d[k]["n_rules"]) for k in ks) / n * 100)
        sa, sb = agg(da), agg(db)
        ident = sum(1 for k in ks if all(da[k][f] == db[k][f] for f in ("T", "score", "levels", "goal", "in_time", "fault")))
        tot += len(ks); same += ident
        lines.append("| %s | %s | %s | %d | %.2f | %.2f | %.0f / %.0f | %.0f / %.0f | %d/%d |" % (
            sc, g[0], g[1], len(ks), sa[0], sb[0], sa[1], sa[2], sb[1], sb[2], ident, len(ks)))
print("\n".join(lines))
print("\nTOTAL runs identical (T, score, levels, goal, in_time, fault): %d / %d" % (same, tot))
