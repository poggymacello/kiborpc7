# -*- coding:utf-8 -*-
"""
Verification suite.  Run:  python -m pytest -q tests   (or: python tests/test_all.py)

1. judge equivalence  : our evaluate() == the OFFICIAL judge.py (loaded with ROS stubs)
2. generator equality : offline ScenarioGenerator == the OFFICIAL generate_objects.py
3. planner optimality : Held-Karp == brute force over all permutations (small n)
4. timing model       : CtlModel profile durations == emulated ctl profile durations
5. end-to-end         : FAST and SAFE missions finish in time with goal reached
6. v3 safety paths    : crash / watchdog / stop all end in the shared protective stop
7. v4                 : per-leg eta split, reference governor, optional ArUco alignment
"""
import importlib.util
import itertools
import math
import os
import random
import re
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "tools"))
import offline_sim as S  # noqa: E402

U = S.U
VENDOR = os.path.join(ROOT, "vendor", "sim", "krpc7th-simulator", "rootfs", "krpc")


# ---------------------------------------------------------------------------
# helpers: load the official judge.py with stubbed ROS modules
# ---------------------------------------------------------------------------
def _stub(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


def load_official_judge():
    class _Pub(object):
        def __init__(self, *a, **k):
            pass

        def publish(self, *a, **k):
            pass

    class _T(object):
        @staticmethod
        def now():
            return types.SimpleNamespace(to_sec=lambda: 1.0e9)

    rospy = _stub("rospy", Publisher=_Pub, Time=_T, loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
                  logerr=lambda *a, **k: None, logdebug=lambda *a, **k: None, on_shutdown=lambda *a: None)
    _stub("tf", TransformBroadcaster=object, TransformListener=object, transformations=None)
    _stub("requests")
    gm = _stub("geometry_msgs")
    _stub("geometry_msgs.msg", Pose=object)
    _stub("std_msgs")
    _stub("std_msgs.msg", Bool=object, String=object)
    _stub("ib2_msgs")
    _stub("ib2_msgs.msg", Navigation=object)
    os.environ.setdefault("HOME", ROOT)
    path = os.path.join(VENDOR, "judge_module", "judge.py")
    spec = importlib.util.spec_from_file_location("official_judge", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _Vec(object):
    def __init__(self, x, y, z, w=None):
        self.x, self.y, self.z = x, y, z
        if w is not None:
            self.w = w


class _Pose(object):
    def __init__(self, p, q):
        self.position = _Vec(*p)
        self.orientation = _Vec(*q)


def _official_referee(mod, payload):
    ref = object.__new__(mod.Referee)
    ref.sim_mode = "regular"
    ref.cheat_data = {"Checkpoints": [dict(c, checkpointid=c["checkpoint_id"]) for c in payload["checkpoints"]],
                      "Goal": {"checkpoint_id": payload["goalpoint_id"]}}
    ref._checkpoint_rules, ref._checkpoint_defaults = ref.load_checkpoint_rules()
    return ref


def _random_pose_near(rng, spec, spread):
    p = [spec.position[i] + rng.uniform(-spread, spread) for i in range(3)]
    axis = [rng.gauss(0, 1) for _ in range(3)]
    ang = math.radians(rng.uniform(0, 40.0))
    q = U.q_normalize(U.q_mul(spec.target_q, U.q_from_axis_angle(axis, ang)))
    if rng.random() < 0.5:
        q = [-c for c in q]
    return p, q


# ---------------------------------------------------------------------------
def test_judge_equivalence():
    mod = load_official_judge()
    gen = S.ScenarioGenerator()
    rng = random.Random(42)
    checked = 0
    for seed in range(1, 21):
        payload = gen.generate("veryhard", seed)
        ref = _official_referee(mod, payload)
        specs, goal = U.parse_checkpoints(payload)
        offline = S.Judge(payload)
        for spec in specs:
            for _ in range(25):
                p, q = _random_pose_near(rng, spec, 0.35)
                ref.ib2_pose = _Pose(p, q)
                ok, reason, detail = ref.evaluate_checkpoint_navigation(spec.cid)
                official = detail["selected_band"]
                ours = U.evaluate(spec, p, q)["level"]
                off, _ = offline.evaluate(spec.cid, p, q)
                assert ours == official == off, (seed, spec.cid, ours, official, off, detail)
                checked += 1
    assert checked >= 5000
    return checked


def test_generator_matches_official():
    sys.path.insert(0, os.path.join(VENDOR, "random_env_generator"))
    path = os.path.join(VENDOR, "random_env_generator", "generate_objects.py")
    spec = importlib.util.spec_from_file_location("official_gen", path)
    og = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(og)
    area = og.load_json(os.path.join(VENDOR, "random_env_generator", "files", "input", "area_config.json"))
    ours_gen = S.ScenarioGenerator(area)
    for level in ("easy", "normal", "hard", "veryhard"):
        for seed in range(1, 8):
            og.random.seed(seed)
            params = og.resolve_checkpoint_sampling_params(area, types.SimpleNamespace(
                checkpoint_distance=None, checkpoint_yaw_min_deg=None, checkpoint_yaw_max_deg=None,
                checkpoint_pitch_min_deg=None, checkpoint_pitch_max_deg=None,
                checkpoint_roll_min_deg=None, checkpoint_roll_max_deg=None))
            sel = og.select_markers_by_difficulty(area["UrdfMarkers"], level, 12)
            cps, goal = og.build_checkpoints_from_urdf_markers(sel, area, params, 12)
            mine = ours_gen.generate(level, seed)
            assert [c["checkpointid"] for c in cps] == [c["checkpoint_id"] for c in mine["checkpoints"]]
            for a, b in zip(cps, mine["checkpoints"]):
                assert max(abs(a["position"][i] - b["position"][i]) for i in range(3)) < 1e-9
                d = abs(sum(a["attitude"]["quaternion"][i] * b["attitude"]["quaternion"][i] for i in range(4)))
                assert d > 1.0 - 1e-9, (level, seed, a["checkpointid"])


def test_planner_is_optimal():
    gen = S.ScenarioGenerator()
    model = U.CtlModel()
    for leg_name in ("leg", "fast_leg"):
        planner = U.Planner(model, None, getattr(model, leg_name))
        for seed in range(1, 6):
            specs, goal_id = U.parse_checkpoints(gen.generate("normal", seed))
            goal = [s for s in specs if s.cid == goal_id][0]
            nodes = [s for s in specs if s.cid != goal_id][:6]
            start_r, start_q = [0.2, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
            plan = planner.plan(start_r, start_q, nodes, goal, 0.0)
            leg = planner.leg_fn
            best = float("inf")
            for perm in itertools.permutations(nodes):
                t, r, q = 0.0, start_r, start_q
                for n in perm:
                    t += leg(r, q, n)[0]
                    r, q = n.position, n.target_q
                t += leg(r, q, goal)[0]
                best = min(best, t)
            assert abs(plan["t_full"] - best) < 1e-6, (leg_name, seed, plan["t_full"], best)


def test_timing_model_matches_emulator():
    model = U.CtlModel()
    rng = random.Random(7)
    for _ in range(200):
        r0 = [rng.uniform(0.2, 7), rng.uniform(-0.8, 0.85), rng.uniform(0, 1.68)]
        r1 = [rng.uniform(0.2, 7), rng.uniform(-0.8, 0.85), rng.uniform(0, 1.68)]
        q0 = U.q_normalize([rng.gauss(0, 1) for _ in range(4)])
        q1 = U.q_normalize([rng.gauss(0, 1) for _ in range(4)])
        pred = model.move(r0, q0, r1, q1, need_report=False)
        prof = S.Profile(model, {"r": r0, "v": [0, 0, 0], "q": q0, "w": [0, 0, 0], "t": 0.0}, (r1, q1))
        assert abs(pred["profile"] - (prof.xtt + prof.qtt)) < 1e-9


def test_end_to_end_missions():
    for strat in ("fast", "safe"):
        for level in ("easy", "veryhard"):
            res, api = S.simulate(level, 3, strat)
            assert res["in_time"], (strat, level, res["T"])
            assert res["goal"], (strat, level)
            assert res["n_lv3"] >= 4, (strat, level, res["levels"])


def test_attitude_limit_cycle_guard():
    """v2 regression: with a 0.5 s navigation delay the v1 FAST attitude loop oscillated at
    the goal checkpoint (normal seed 16, hard seed 18) and the goal was never reported."""
    for level, seed in (("normal", 16), ("hard", 18)):
        # v4 default (per-leg split + governor): goal still reached in the two v2 regression runs
        res, api = S.simulate(level, seed, "fast", world_kw={"nav_delay": 0.5})
        assert res["goal"], (level, seed, res["levels"])
        # the guard itself, on the v3 motion (v4 changes the trajectory, so it may not be needed there)
        res, api = S.simulate(level, seed, "fast", cfg={"FAST_ETA_ADAPT": False, "FAST_GOV": False},
                              world_kw={"nav_delay": 0.5})
        assert res["goal"], (level, seed, res["levels"])
        assert any("attitude limit cycle" in text for _, _, text in api.logs), (level, seed)
    # nominal runs must not trigger the guard
    for level in ("easy", "veryhard"):
        res, api = S.simulate(level, 3, "fast")
        assert not any("attitude limit cycle" in text for _, _, text in api.logs), level


# ---------------------------------------------------------------------------
# v3 (23 Oct 2026): crash path / protective stop
# ---------------------------------------------------------------------------
def _crash_setup(strategy, level="veryhard", seed=3, crash_after_s=30.0):
    """Offline mission that raises RuntimeError mid-flight (robot moving > 5 cm/s)."""
    payload = S.ScenarioGenerator().generate(level, seed)
    world = S.World(payload, seed=seed)
    api = S.OfflineApi(world)
    mission = U.Mission(api, cfg={"MOTION_MODE": strategy})
    state = {"crashed": False, "t": None, "r": None, "user_wrench": None}
    orig_sleep = api.sleep

    def sleep(dt):
        orig_sleep(dt)
        if not state["crashed"] and world.t > crash_after_s and U.v_norm(world.v) > 0.05:
            state.update(crashed=True, t=world.t, r=list(world.r), user_wrench=world.wrench_from_user)
            raise RuntimeError("injected crash for test")
    api.sleep = sleep
    return world, api, mission, state


def _names(api):
    return [n for _, n in api.events]


def test_crash_path_fast_mode():
    """Mission.run() raises while FAST is flying the robot -> FAST off, >=3 zero wrenches,
    ctl KEEP_POSE, THEN exactly one mission_finished; the robot comes to rest under ctl."""
    world, api, mission, st = _crash_setup("fast")
    m, crashed = U.run_mission_guarded(api, lambda: mission)
    assert crashed and st["crashed"] and m is mission
    assert mission.mode == "fast" and mission.fast is not None
    assert not mission.fast.enabled, "wrench controller still active after crash"
    assert mission.finished
    names = _names(api)
    i_finish = names.index("mission_finished")
    assert names.count("mission_finished") == 1, names
    i_keep = max(i for i, n in enumerate(names) if n == "ctl_keep_pose")
    assert i_keep < i_finish, names
    zeros_before_keep = [i for i, n in enumerate(names[:i_keep]) if n == "zero_wrench"]
    assert len(zeros_before_keep) >= U.CONFIG["SAFE_STOP_ZERO_N"], names
    assert world.ctl.status == "KEEP"
    assert any("mission crashed" in t for _, _, t in api.logs)
    assert any("SAFE STOP (mission crashed)" in t for _, _, t in api.logs)
    res = world.judge.score()
    assert res["in_time"], res["T"]
    # after the stop the user wrench never reaches the fans again and ctl holds the robot
    n_user = world.wrench_from_user
    world.advance(25.0)
    assert world.wrench_from_user == n_user
    assert U.v_norm(world.v) < 0.02, world.v
    assert U.v_norm(U.v_sub(world.r, st["r"])) < 0.35, (world.r, st["r"])
    return st["t"]


def test_unguarded_v2_crash_path_leaves_fast_active():
    """Documents the v2 bug the guard fixes: the old handler only sent mission_finished."""
    world, api, mission, st = _crash_setup("fast")
    try:
        mission.run()
    except RuntimeError:
        api.mission_finished()          # what v2's except-branch did
    assert st["crashed"] and mission.fast.enabled and world.ctl.status == "STAND_BY"
    n_user = world.wrench_from_user
    world.advance(2.0)
    assert world.wrench_from_user > n_user  # our wrench controller keeps driving the fans


def test_crash_path_safe_mode():
    world, api, mission, st = _crash_setup("safe")
    m, crashed = U.run_mission_guarded(api, lambda: mission)
    assert crashed and st["crashed"] and mission.fast is None
    names = _names(api)
    assert names.count("mission_finished") == 1
    assert names.index("ctl_keep_pose") < names.index("mission_finished"), names
    assert "zero_wrench" not in names  # pure SAFE mode never touches /ctl/wrench
    assert world.ctl.status == "KEEP"
    world.advance(25.0)
    assert U.v_norm(world.v) < 0.02, world.v


def test_crash_in_constructor_and_base_exception():
    payload = S.ScenarioGenerator().generate("easy", 1)
    world = S.World(payload, seed=1)
    api = S.OfflineApi(world)

    def boom():
        raise ValueError("constructor failure")
    m, crashed = U.run_mission_guarded(api, boom)
    assert m is None and crashed
    assert "mission_finished" not in _names(api)   # mission_start was never sent
    assert "ctl_keep_pose" not in _names(api)      # never commanded the robot -> no ctl command
    # BaseException (e.g. KeyboardInterrupt / SystemExit) still gets the safe stop + finish
    world, api, mission, st = _crash_setup("fast")

    def interrupted_run():
        api.mission_start()
        mission.t0 = api.now()
        mission.fast.enable(True)
        raise KeyboardInterrupt()

    def run_interrupted():
        mission.run = interrupted_run
        return mission
    raised = False
    try:
        U.run_mission_guarded(api, run_interrupted)
    except KeyboardInterrupt:
        raised = True
    assert raised and mission.finished and not mission.fast.enabled
    assert _names(api).count("mission_finished") == 1


def test_normal_finish_and_watchdog_share_safe_stop():
    res, api = S.simulate("easy", 3, "fast")
    names = _names(api)
    assert res["in_time"] and res["goal"] and names.count("mission_finished") == 1
    tail = names[names.index("mission_finished"):]
    assert tail[:1] == ["mission_finished"] and tail.count("zero_wrench") >= 3 and tail[-1] == "ctl_keep_pose", tail
    assert any("SAFE STOP (mission finished)" in t for _, _, t in api.logs)
    # deadline watchdog decision + emergency finish (thread itself: tests/test_ros_adapter.py)
    payload = S.ScenarioGenerator().generate("easy", 2)
    world = S.World(payload, seed=2)
    api = S.OfflineApi(world)
    mission = U.Mission(api, cfg={"MOTION_MODE": "fast"})
    assert not mission.watchdog_due()                     # not started
    mission.t0 = api.now() - (U.CONFIG["T_LIMIT_S"] - U.CONFIG["DEADLINE_WATCHDOG_S"] - 0.5)
    assert not mission.watchdog_due()
    mission.t0 -= 1.0
    assert mission.watchdog_due()
    mission.fast.enable(True)
    assert mission.emergency_finish("deadline watchdog") is True
    assert mission.emergency_finish("deadline watchdog") is False   # idempotent
    mission._finish()                                               # no second report
    names = _names(api)
    assert names.count("mission_finished") == 1 and names.index("ctl_keep_pose") < names.index("mission_finished")
    assert not mission.fast.enabled and not mission.watchdog_due()



# ---------------------------------------------------------------------------
# v4 (24 Oct 2026): per-leg eta split + optional ArUco final alignment
# ---------------------------------------------------------------------------
def test_eta_split_fixed_when_disabled_and_bounded_when_enabled():
    """FAST_ETA_ADAPT=False reproduces the v1-v3 fixed split exactly; when enabled the split
    never exceeds the v3 fan budget and the flown profile equals the planner's prediction."""
    gen = S.ScenarioGenerator()
    off = U.CtlModel(None, {"FAST_ETA_ADAPT": False})
    on = U.CtlModel(None, {"FAST_ETA_ADAPT": True})
    E = U.CONFIG["FAST_ETA_T"] + U.CONFIG["FAST_ETA_R"]
    for seed in range(1, 4):
        specs, _ = U.parse_checkpoints(gen.generate("veryhard", seed))
        r, q = [0.2, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
        for s in specs:
            mv = off.fast_move(r, q, s.position, s.target_q)
            assert mv["eta_t"] == U.CONFIG["FAST_ETA_T"] and mv["eta_r"] == U.CONFIG["FAST_ETA_R"]
            assert abs(mv["acc"] - off.fast_acc(U.q_normalize(q), U.q_normalize(s.target_q),
                                                U.v_unit(U.v_sub(s.position, r)))) < 1e-15
            mv2 = on.fast_move(r, q, s.position, s.target_q)
            assert mv2["eta_t"] + mv2["eta_r"] <= E + 1e-12, mv2
            assert mv2["eta_t"] <= U.CONFIG["FAST_ETA_T_MAX_FRAC"] * E + 1e-12
            assert mv2["eta_r"] >= U.CONFIG["FAST_ETA_R_MIN_FRAC"] * E - 1e-12
            assert mv2["T"] <= mv["T"] / U.CONFIG["FAST_ROT_LEAD"] + 1e-9   # never slower than fixed
            prof = U.FastProfile(on, 0.0, r, q, s.position, s.target_q)
            assert abs(prof.duration - mv2["T"]) < 1e-9
            r, q = s.position, s.target_q


def _tvec_for(pos, q, marker_world, cfg=U.CONFIG):
    b = U.v_sub(U.q_rotate(U.q_conj(q), U.v_sub(marker_world, pos)), cfg["ARUCO_CAM_OFFSET_B"])
    return [-b[1], -b[2], b[0]]


def test_aruco_alignment_math():
    rng = random.Random(11)
    for _ in range(200):
        pos = [rng.uniform(0.2, 7), rng.uniform(-0.8, 0.85), rng.uniform(0, 1.68)]
        q = U.q_normalize([rng.gauss(0, 1) for _ in range(4)])
        m = [rng.uniform(0, 7), rng.uniform(-1, 1), rng.uniform(-0.2, 1.9)]
        back = U.marker_world_from_tvec(pos, q, _tvec_for(pos, q, m), U.CONFIG["ARUCO_CAM_OFFSET_B"])
        assert U.v_norm(U.v_sub(back, m)) < 1e-9
    cfg = U.CONFIG
    m = [1.0, 2.0, 3.0]
    off = [0.03, -0.01, 0.0]
    obs = [U.v_add(U.v_add(m, off), [0.001 * k, 0.0, 0.0]) for k in range(3)]
    got = U.aruco_offset(obs, m, cfg)
    assert got is not None and U.v_norm(U.v_sub(got, [0.031, -0.01, 0.0])) < 1e-9
    assert U.aruco_offset(obs[:2], m, cfg) is None                                   # too few
    assert U.aruco_offset(obs + [U.v_add(m, [0.06, 0, 0])], m, cfg) is None          # inconsistent
    assert U.aruco_offset([U.v_add(m, [0.08, 0, 0])] * 3, m, cfg) is None            # implausible


class _ArucoApi(S.OfflineApi):
    """Offline api whose camera sees every marker displaced by OFFSET in the navigation frame."""
    OFFSET = [0.03, 0.0, -0.02]

    def __init__(self, world, specs):
        S.OfflineApi.__init__(self, world)
        self.markers = dict((s.cid, s.marker_position) for s in specs)
        self.calls = 0

    def aruco_observe(self, cid):
        # deterministic (no draw from the world's noise stream, so "log" mode flies exactly like "off")
        self.calls += 1
        h = self.w.hist[0]
        r = U.v_add(h["r"], self.w.nav_bias)
        return _tvec_for(r, h["q"], U.v_add(self.markers[cid], self.OFFSET))


def _run_aruco(mode, level="easy", seed=3):
    payload = S.ScenarioGenerator().generate(level, seed)
    specs, _ = U.parse_checkpoints(payload)
    world = S.World(payload, seed=seed)
    api = _ArucoApi(world, specs)
    mission = U.Mission(api, cfg={"MOTION_MODE": "fast", "ARUCO_ALIGN": mode})
    mission.run()
    return world.judge.score(), api, mission


def test_aruco_align_modes():
    """off: camera never queried. log: offsets logged, flight identical to off. correct: each
    checkpoint is re-targeted by the measured offset and the mission still finishes in time."""
    r_off, api_off, _ = _run_aruco("off")
    assert api_off.calls == 0
    r_log, api_log, _ = _run_aruco("log")
    assert api_log.calls > 0
    assert abs(r_log["T"] - r_off["T"]) < 1e-9 and r_log["levels"] == r_off["levels"]
    offs = [re.findall(r"aruco offset \[([-\d.]+), ([-\d.]+), ([-\d.]+)\]", t) for _, _, t in api_log.logs]
    offs = [[float(x) for x in o[0]] for o in offs if o]
    assert len(offs) == r_off["n_rules"], offs
    assert all(U.v_norm(U.v_sub(o, _ArucoApi.OFFSET)) < 0.004 for o in offs), offs
    r_cor, api_cor, m_cor = _run_aruco("correct")
    assert r_cor["in_time"] and r_cor["goal"], r_cor
    assert len(m_cor.results) == r_cor["n_rules"]
    for cid, res in m_cor.results.items():
        assert res["dist"] < 0.06, (cid, res)   # judged against the SHIFTED checkpoint


def test_reference_governor_weak_fans():
    """v4: with fans 20% weaker than the model (easy seed 10) the per-leg split alone faults and
    falls back to SAFE; the reference governor slows the profile clock instead: no fault, faster."""
    wk = {"fan_scale": 0.8}
    r_nogov, _ = S.simulate("easy", 10, "fast", cfg={"FAST_GOV": False}, world_kw=wk)
    r_gov, api = S.simulate("easy", 10, "fast", world_kw=wk)
    assert r_nogov["fault"], r_nogov
    assert not r_gov["fault"] and r_gov["goal"] and r_gov["in_time"], r_gov
    assert r_gov["n_lv3"] == r_gov["n_rules"] and r_gov["T"] < r_nogov["T"] - 20.0, (r_gov["T"], r_nogov["T"])
    assert any("governor s=" in t for _, _, t in api.logs)
    # nominal: the governor never changes the result of a healthy run
    r_a, _ = S.simulate("veryhard", 3, "fast")
    r_b, _ = S.simulate("veryhard", 3, "fast", cfg={"FAST_GOV": False})
    assert abs(r_a["T"] - r_b["T"]) < 1e-9 and r_a["levels"] == r_b["levels"]


def test_start_settle_when_spinning():
    """v4: a robot that drifts and spins at mission start (seen in an overloaded official run)
    is first brought to rest by ctl KEEP_POSE; FAST then flies without a tracking fault.
    Without the settle step FAST faults (attitude error ~100 deg) and loses time."""
    def run(cfg):
        payload = S.ScenarioGenerator().generate("easy", 3)
        world = S.World(payload, seed=3)
        world.w, world.v = [0.3, -0.6, 0.9], [0.0, 0.08, 0.05]
        world.advance(0.3)
        api = S.OfflineApi(world)
        m = U.Mission(api, cfg=dict({"MOTION_MODE": "fast"}, **cfg))
        m.run()
        return world.judge.score(), m, api
    r1, m1, api1 = run({})
    assert any("robot moving at start" in t for _, _, t in api1.logs)
    assert not m1.fast.fault and not m1.fast_degrades and r1["goal"] and r1["n_lv3"] == r1["n_rules"], r1
    r0, m0, _ = run({"START_REST_V": 1e9, "START_REST_W_DEG": 1e9})
    assert m0.fast_degrades >= 1 and r1["T"] < r0["T"], (r0["T"], r1["T"])


if __name__ == "__main__":
    n = test_judge_equivalence()
    print("judge equivalence: %d random poses identical to official judge.py" % n)
    test_generator_matches_official()
    print("generator: identical to official generate_objects.py")
    test_planner_is_optimal()
    print("planner: Held-Karp == brute force")
    test_timing_model_matches_emulator()
    print("timing model == emulated ctl profiler")
    test_end_to_end_missions()
    print("end-to-end: OK")
    test_attitude_limit_cycle_guard()
    print("attitude limit-cycle guard (v2): OK")
    t_crash = test_crash_path_fast_mode()
    print("crash path FAST (v3): crash at t=%.1fs -> FAST off, zero wrench, KEEP_POSE, 1x mission_finished, robot at rest: OK" % t_crash)
    test_unguarded_v2_crash_path_leaves_fast_active()
    print("v2 crash handler reproduced: wrench controller stayed active (bug confirmed, fixed in v3)")
    test_crash_path_safe_mode()
    print("crash path SAFE (v3): OK")
    test_crash_in_constructor_and_base_exception()
    print("crash in constructor / BaseException (v3): OK")
    test_normal_finish_and_watchdog_share_safe_stop()
    print("normal finish + deadline watchdog share safe_stop (v3): OK")
    test_eta_split_fixed_when_disabled_and_bounded_when_enabled()
    print("eta split (v4): fixed when disabled, within the v3 fan budget, profile == prediction: OK")
    test_aruco_alignment_math()
    print("ArUco alignment math (v4): OK")
    test_aruco_align_modes()
    print("ArUco modes off/log/correct (v4): OK")
    test_reference_governor_weak_fans()
    print("reference governor (v4): weak fans without fault, nominal unchanged: OK")
    test_start_settle_when_spinning()
    print("start settle (v4): spinning start -> ctl KEEP_POSE first, FAST without fault: OK")
    print("ALL 16 TESTS PASSED")
