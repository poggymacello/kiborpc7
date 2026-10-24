#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Offline Monte-Carlo simulator for the Kibo-RPC 7th preliminary round.

Why: the official local simulator needs WSL2 + Docker (Gazebo). This tool lets the
team test the mission logic on any laptop, thousands of times, against:

  * ScenarioGenerator .. line-by-line port of rootfs/krpc/random_env_generator/generate_objects.py
  * CtlEmulator ........ port of ctl_only (pos_att_profiler.cpp, pos/att controllers,
                         thrust_allocator.cpp, reachGoal + preempt/cancel semantics of ctl.cpp)
  * Plant .............. 6-DoF rigid body (sim.yaml mass/inertia, 8 fans with sim.yaml
                         geometry, fan force noise, airflow-like OU disturbance, optional
                         constant "microgravity module" bias), navigation delay + bias
  * Judge .............. independent port of rootfs/krpc/judge_module/judge.py
  * OfflineApi ......... the `api` object expected by Mission in user_template.py

It is an approximation of Gazebo (no airflow field file, no collisions), but the
controller, profiler, tolerances and judge logic are the real ones, so mission
timing and scoring trends are representative. Calibrate LEG_OVERHEAD_S against the
local/Web simulator logs once available.
"""

import argparse
import importlib.util
import json
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
USER_SCRIPT = os.path.join(ROOT, "krpc_user", "scripts", "user_template.py")
AREA_CONFIG = os.path.join(ROOT, "vendor", "sim", "krpc7th-simulator", "rootfs", "krpc",
                           "random_env_generator", "files", "input", "area_config.json")


def load_user_module(path=USER_SCRIPT):
    spec = importlib.util.spec_from_file_location("krpc_user_program", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


U = load_user_module()
q_mul, q_conj, q_normalize, q_rotate = U.q_mul, U.q_conj, U.q_normalize, U.q_rotate
v_add, v_sub, v_scale, v_norm, v_unit, v_dot, v_cross = U.v_add, U.v_sub, U.v_scale, U.v_norm, U.v_unit, U.v_dot, U.v_cross

# Embedded copy of area_config.json (used when the vendor folder is absent)
_BANDS = [{"radius": 0.1, "level": 3}, {"radius": 0.2, "level": 2}, {"radius": 0.3, "level": 1}]
EMBEDDED_AREA = {
    "SafeArea": {"min": [0.2, -0.8, 0.0], "max": [7.0, 0.85, 1.68]},
    "UrdfMarkers": [
        {"id": 1, "position": [1.451, -1.001, 0.6806], "quaternion": [-0.5, 0.5, 0.5, 0.5]},
        {"id": 2, "position": [2.973, -1.001, 0.7176], "quaternion": [-0.5, 0.5, 0.5, 0.5]},
        {"id": 3, "position": [3.313, -1.001, 0.5226], "quaternion": [-0.5, 0.5, 0.5, 0.5]},
        {"id": 4, "position": [4.181, -1.001, 0.5226], "quaternion": [-0.5, 0.5, 0.5, 0.5]},
        {"id": 5, "position": [5.754, -1.001, 0.8376], "quaternion": [-0.5, 0.5, 0.5, 0.5]},
        {"id": 6, "position": [1.895, -0.282, -0.2084], "quaternion": [0.0, 0.0, 0.7071068, 0.7071068]},
        {"id": 7, "position": [1.451, -1.001, 0.3716], "quaternion": [-0.5, 0.5, 0.5, 0.5]},
        {"id": 8, "position": [4.779, 0.112, 1.8836], "quaternion": [0.0, 1.0, 0.0, 0.0]},
        {"id": 9, "position": [2.066, 1.046, -0.0144], "quaternion": [-0.5, 0.5, -0.5, -0.5]},
        {"id": 10, "position": [5.851, 0.01, 1.8836], "quaternion": [0.0, 1.0, 0.0, 0.0]},
        {"id": 11, "position": [7.195, 0.363, 1.2296], "quaternion": [-0.5, 0.5, 0.5, -0.5]},
        {"id": 12, "position": [0.0, -0.059, -0.1024], "quaternion": [-0.5, 0.5, -0.5, 0.5]},
    ],
}
for _m in EMBEDDED_AREA["UrdfMarkers"]:
    _m["score_bands"] = _BANDS

LEVEL_COUNT = {"easy": 6, "normal": 8, "hard": 10, "veryhard": 12}


# =============================================================================
# Scenario generator (port of generate_objects.py)
# =============================================================================
class ScenarioGenerator(object):
    DIST = 0.5
    YAW = (-20.0, 20.0)
    PITCH = (-15.0, 15.0)
    ROLL = (-180.0, 180.0)
    ATT_BANDS = [{"angle": 10.0, "level": 3}, {"angle": 20.0, "level": 2}, {"angle": 30.0, "level": 1}]
    GOAL_ID = 12

    def __init__(self, area=None):
        if area is None:
            if os.path.exists(AREA_CONFIG):
                with open(AREA_CONFIG, "r", encoding="utf-8") as f:
                    area = json.load(f)
            else:
                area = EMBEDDED_AREA
        self.area = area
        self.safe = area.get("SafeArea")

    def _inside(self, p):
        lo, hi = self.safe["min"], self.safe["max"]
        return all(lo[i] <= p[i] <= hi[i] for i in range(3))

    def _sample_pos(self, rng, mpos, mq, straight=False):
        yaw = 0.0 if straight else math.radians(rng.uniform(*self.YAW))
        pitch = 0.0 if straight else math.radians(rng.uniform(*self.PITCH))
        local = [math.cos(pitch) * math.sin(yaw), math.sin(pitch), math.cos(pitch) * math.cos(yaw)]
        d = q_rotate(mq, local)
        return [mpos[i] + self.DIST * d[i] for i in range(3)]

    def _pos(self, rng, mpos, mq):
        for _ in range(50):
            p = self._sample_pos(rng, mpos, mq)
            if self._inside(p):
                return p
        p = self._sample_pos(rng, mpos, mq, straight=True)
        if self._inside(p):
            return p
        lo, hi = self.safe["min"], self.safe["max"]
        return [min(max(p[i], lo[i]), hi[i]) for i in range(3)]

    def _attitude(self, rng, cpos, mpos):
        fwd = v_unit(v_sub(mpos, cpos), [1.0, 0.0, 0.0])
        up = [0.0, 0.0, 1.0] if abs(v_dot(fwd, [0.0, 0.0, 1.0])) <= 0.95 else [0.0, 1.0, 0.0]
        z = v_unit(v_cross(fwd, up), [0.0, 0.0, 1.0])
        y = v_unit(v_cross(z, fwd), [0.0, 1.0, 0.0])
        base = U.q_from_rotmat([[fwd[0], y[0], z[0]], [fwd[1], y[1], z[1]], [fwd[2], y[2], z[2]]])
        roll = math.radians(rng.uniform(*self.ROLL))
        return q_normalize(q_mul(U.q_from_axis_angle(fwd, roll), base))

    def generate(self, level="normal", seed=0):
        rng = random.Random(seed)
        markers = list(self.area["UrdfMarkers"])
        goal = [m for m in markers if m["id"] == self.GOAL_ID][0]
        others = [m for m in markers if m["id"] != self.GOAL_ID]
        slots = LEVEL_COUNT[level] - 1
        sel = others[:] if slots >= len(others) else rng.sample(others, slots)
        sel.append(goal)
        cps = []
        for m in sel:
            mpos = [float(x) for x in m["position"]]
            mq = q_normalize([float(x) for x in m["quaternion"]])
            cpos = self._pos(rng, mpos, mq)
            cps.append({
                "checkpoint_id": int(m["id"]),
                "position": cpos,
                "marker_pose": {"position": mpos, "quaternion": mq},
                "score_bands": m.get("score_bands", _BANDS),
                "attitude": {"quaternion": self._attitude(rng, cpos, mpos),
                             "centering_bands": self.ATT_BANDS, "roll_bands": self.ATT_BANDS},
            })
        cps.sort(key=lambda c: c["checkpoint_id"])
        return {"checkpoint_count": len(cps), "checkpoints": cps, "goalpoint_id": self.GOAL_ID, "level": level, "seed": seed}


# =============================================================================
# Judge (independent port of judge.py)
# =============================================================================
class Judge(object):
    def __init__(self, payload):
        self.rules = dict((c["checkpoint_id"], c) for c in payload["checkpoints"])
        self.goal_id = payload["goalpoint_id"]
        self.started = None
        self.finished = None
        self.cps = {}
        self.goal = None

    @staticmethod
    def _bands(objs, key):
        out = sorted((float(b[key]), int(b["level"])) for b in objs)
        return out

    @staticmethod
    def _classify(v, bands):
        for thr, lvl in bands:
            if v <= thr:
                return lvl
        return 0

    @staticmethod
    def _twist(q, axis):
        qx, qy, qz, qw = q_normalize(q)
        if qw < 0.0:
            qx, qy, qz, qw = -qx, -qy, -qz, -qw
        return math.degrees(2.0 * math.atan2(qx * axis[0] + qy * axis[1] + qz * axis[2], qw))

    def evaluate(self, cid, pos, q):
        r = self.rules[cid]
        d = v_norm(v_sub(pos, r["position"]))
        db = self._classify(d, self._bands(r["score_bands"], "radius"))
        mp = r["marker_pose"]["position"]
        view = v_unit(v_sub(mp, pos))
        cam = v_unit(q_rotate(q, [1.0, 0.0, 0.0]))
        cen = math.degrees(math.acos(max(-1.0, min(1.0, v_dot(cam, view)))))
        tq = r["attitude"]["quaternion"]
        roll = abs(((self._twist(q, view) - self._twist(tq, view)) + 180.0) % 360.0 - 180.0)
        cb = self._classify(cen, self._bands(r["attitude"]["centering_bands"], "angle"))
        rb = self._classify(roll, self._bands(r["attitude"]["roll_bands"], "angle"))
        c = [db, cb, rb]
        lvl = 0 if any(x == 0 for x in c) else min(c)
        return lvl, {"dist": d, "cen": cen, "roll": roll}

    def start(self, t):
        if self.started is None:
            self.started = t

    def checkpoint(self, t, cid, pos, q):
        if self.started is None or self.finished is not None or cid in self.cps or cid not in self.rules:
            return False
        lvl, det = self.evaluate(cid, pos, q)
        self.cps[cid] = {"level": lvl, "t": t - self.started, "detail": det}
        return True

    def finish(self, t, pos, q):
        if self.started is None or self.finished is not None:
            return False
        prev = self.cps.get(self.goal_id)
        if prev and prev["level"] > 0:
            self.goal = True
        else:
            lvl, _ = self.evaluate(self.goal_id, pos, q)
            self.goal = lvl > 0
            if lvl > 0:
                self.cps[self.goal_id] = {"level": lvl, "t": t - self.started, "detail": {}}
        self.finished = t
        return True

    def score(self, t_limit=300.0, level_points=None, all_bonus=3.0, mag_max=2.0):
        lp = level_points or {3: 3.0, 2: 2.0, 1: 1.0, 0: 0.0}
        T = (self.finished - self.started) if (self.finished is not None and self.started is not None) else float("inf")
        in_time = T <= t_limit
        base = sum(lp.get(v["level"], 0.0) for v in self.cps.values())
        all_ok = all(self.cps.get(cid, {}).get("level", 0) > 0 for cid in self.rules)
        if all_ok and self.goal:
            base += all_bonus
        mag = min(mag_max, 1.0 + max(0.0, t_limit - T) / t_limit) if in_time else 1.0
        return {"T": T, "in_time": in_time, "base": base, "mag": mag, "score": base * mag if in_time else 0.0,
                "levels": dict((k, v["level"]) for k, v in self.cps.items()), "goal": bool(self.goal),
                "n_rules": len(self.rules), "n_lv3": sum(1 for v in self.cps.values() if v["level"] == 3)}


# =============================================================================
# ctl_only emulator
# =============================================================================
P = U.CTL_PARAMS
FAN_POS = [(0.045, 0.070, 0.0555), (0.045, -0.070, 0.0555), (0.045, -0.070, -0.0555), (0.045, 0.070, -0.0555),
           (-0.045, 0.070, -0.0555), (-0.045, 0.070, 0.0555), (-0.045, -0.070, 0.0555), (-0.045, -0.070, -0.0555)]
FAN_VEC = [(-0.754180198, -0.414799109, -0.509071634), (-0.754180198, 0.414799109, -0.509071634),
           (-0.754180198, 0.414799109, 0.509071634), (-0.754180198, -0.414799109, 0.509071634),
           (0.754180198, -0.414799109, 0.509071634), (0.754180198, -0.414799109, -0.509071634),
           (0.754180198, 0.414799109, -0.509071634), (0.754180198, 0.414799109, 0.509071634)]
CG = (0.001489, 0.001363, 0.000249)


def _target(t, tcb, tce, te, thr, r, v, a):
    """pos_att_profiler.cpp::target()"""
    if r < thr:
        return r, 0.0, 0.0
    if t < 0.0:
        return 0.0, 0.0, 0.0
    if t < tcb:
        vp = a * t
        return vp * t * 0.5, vp, a
    if t < tce:
        return v * (t - 0.5 * tcb), v, 0.0
    if t < te:
        tg = te - t
        vp = a * tg
        return r - vp * tg * 0.5, vp, -a
    return r, 0.0, 0.0


class Profile(object):
    """PosAttProfiler state for one command."""

    def __init__(self, model, nav, goal=None, stopping=False, eta_max=0.4):
        r0, v0, q0, w0, t0 = nav["r"], nav["v"], nav["q"], nav["w"], nav["t"]
        self.t0, self.r0, self.v0, self.q0 = t0, list(r0), list(v0), q_normalize(q0)
        self.seq = "PARALLEL"
        self.dh, self.xm, self.amax = [1.0, 0.0, 0.0], 0.0, 0.001
        self.xcb = self.xce = self.xtt = 0.0
        self.axis, self.qm, self.wdmax = [1.0, 0.0, 0.0], 0.0, 1e-6
        self.qcb = self.qce = self.qtt = 0.0
        self.q1 = list(self.q0)
        self.r1 = list(self.r0)
        if goal is not None:
            r1g, q1g = goal
            dr = q_rotate(q_conj(self.q0), v_sub(r1g, self.r0))
            dq = q_mul(q_conj(self.q0), q_normalize(q1g))
            if dq[3] < 0.0:
                dq = [-c for c in dq]
            # setProfilePos
            self.xm = v_norm(dr)
            db = v_unit(dr) if self.xm > P["eps_rm"] else [1.0, 0.0, 0.0]
            self.dh = q_rotate(self.q0, db)
            self.r1 = v_add(self.r0, v_scale(self.dh, self.xm))
            self.amax = model.eff_acc(db)
            self.xcb, self.xce, self.xtt = U.maneuver_timing(self.xm, P["vmax"], self.amax) if self.xm > 0 else (0.0, 0.0, 0.0)
            # setProfileAtt
            self.q1 = q_normalize(q_mul(self.q0, dq))
            self.qm = 2.0 * math.acos(max(-1.0, min(1.0, dq[3])))
            self.axis = v_unit(dq[:3]) if self.qm > P["eps_qm"] else [1.0, 0.0, 0.0]
            self.wdmax = model.eff_alpha(self.axis)
            self.qcb, self.qce, self.qtt = U.maneuver_timing(self.qm, P["wmax"], self.wdmax) if self.qm > 0 else (0.0, 0.0, 0.0)
            self.seq = "POS_ATT"
        elif stopping:
            wn = v_norm(w0)
            self.axis = v_unit(w0) if wn > P["eps_qm"] else [1.0, 0.0, 0.0]
            self.wdmax = model.eff_alpha(self.axis, eta_max)
            self.qtt = wn / self.wdmax
            self.qm = 0.5 * self.wdmax * self.qtt ** 2
            self.q1 = q_normalize(q_mul(self.q0, U.q_from_axis_angle(self.axis, self.qm)))
            vn = v_norm(v0)
            self.dh = v_unit(v0) if vn > P["eps_rm"] else [1.0, 0.0, 0.0]
            db = q_rotate(q_conj(self.q1), self.dh)
            self.amax = model.eff_acc(db, eta_max)
            self.xtt = vn / self.amax
            self.xm = 0.5 * self.amax * self.xtt ** 2
            self.r1 = v_add(self.r0, v_scale(self.dh, self.xm))
            self.seq = "ATT_POS"

    def te(self):
        return self.t0 + (max(self.xtt, self.qtt) if self.seq == "PARALLEL" else self.xtt + self.qtt)

    def ref(self, t_abs):
        t = t_abs - self.t0
        # position
        r0, tp = self.r0, t
        if self.seq == "ATT_POS":
            if t <= self.qtt:
                r = v_add(self.r0, v_scale(self.v0, t))
                rr, vv, aa = r, list(self.v0), [0.0, 0.0, 0.0]
                return rr, vv, aa, self._att(t)
            r0 = v_add(self.r0, v_scale(self.v0, self.qtt))
            tp = t - self.qtt
        xp, vp, ap = _target(tp, self.xcb, self.xce, self.xtt, P["x_thr"], self.xm, P["vmax"], self.amax)
        return v_add(r0, v_scale(self.dh, xp)), v_scale(self.dh, vp), v_scale(self.dh, ap), self._att(t)

    def _att(self, t):
        if self.seq == "POS_ATT":
            if t <= self.xtt:
                return self.q0, [0.0, 0.0, 0.0]
            t -= self.xtt
        qp, wp, _ = _target(t, self.qcb, self.qce, self.qtt, P["q_thr"], self.qm, P["wmax"], self.wdmax)
        return q_normalize(q_mul(self.q0, U.q_from_axis_angle(self.axis, qp))), v_scale(self.axis, wp)


class CtlEmulator(object):
    KP_POS, KD_POS = 0.6219, 1.1152
    KP_ATT, KD_ATT = 2.6181, 3.2361

    def __init__(self, model):
        self.model = model
        self.status = "STAND_BY"       # STAND_BY | KEEP | MOVING | STOPPING
        self.profile = None
        self.goals = {}                 # gid -> state string
        self.active = None              # (gid, (r, q))
        self.queued = None
        self.preempt = False
        self.stay = False
        self.t_in = 0.0
        self.stop_deadline = 0.0

    def send_goal(self, gid, r, q, nav):
        self.goals[gid] = "PENDING"
        if self.status in ("MOVING", "STOPPING") and self.active is not None:
            self.preempt = True
            if self.queued is not None:
                self.goals[self.queued[0]] = "RECALLED"
            self.queued = (gid, (list(r), list(q)))
        else:
            self._start(gid, (r, q), nav)

    def cancel(self, nav):
        if self.status == "MOVING":
            self.preempt = True

    def set_mode(self, mode, nav):
        """ctl.cpp commandCallback for type < STOP_MOVING (STAND_BY / KEEP_POSE)."""
        self.profile = Profile(self.model, nav)   # setKeepPose(): zero-motion profile at nav
        self.status = mode
        self.active = None
        self.queued = None
        self.preempt = False

    def _start(self, gid, goal, nav):
        self.active = (gid, goal)
        self.goals[gid] = "ACTIVE"
        self.profile = Profile(self.model, nav, goal)
        self.status = "MOVING"
        self.stay = False
        self.preempt = False

    def _reach(self, nav, r1, q1, tolp, tola):
        d = v_norm(v_sub(r1, nav["r"]))
        a = U.q_angle(nav["q"], q1)
        if d < tolp and a < tola:
            if not self.stay:
                self.t_in = nav["t"]
            self.stay = True
        else:
            self.stay = False
        return self.stay and nav["t"] - self.t_in >= P["duration_goal"]

    def step(self, nav):
        """Update action state + return body wrench (F, T) for this nav sample."""
        if self.status == "MOVING":
            gid = self.active[0]
            if self.preempt:
                self.goals[gid] = "PREEMPTED"
                self.profile = Profile(self.model, nav, None, stopping=True)
                self.status = "STOPPING"
                self.stay = False
                self.stop_deadline = self.profile.te() + 10.0
            elif self._reach(nav, self.profile.r1, self.profile.q1, P["tol_pos"], P["tol_att"]):
                self.goals[gid] = "SUCCEEDED"
                self.status = "KEEP"
        elif self.status == "STOPPING":
            if self._reach(nav, self.profile.r1, self.profile.q1, 0.30, 1.0) or nav["t"] >= self.stop_deadline:
                self.status = "KEEP"
                self.preempt = False
                if self.queued is not None:
                    gid, goal = self.queued
                    self.queued = None
                    self._start(gid, goal, nav)
        if self.status in ("KEEP", "STAND_BY") and self.queued is not None:
            gid, goal = self.queued
            self.queued = None
            self._start(gid, goal, nav)
        if self.status == "STAND_BY" or self.profile is None:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        rr, vr, ar, (qr, wr) = self.profile.ref(nav["t"])
        m = P["mass"]
        acc = [ar[i] - self.KP_POS * (nav["r"][i] - rr[i]) - self.KD_POS * (nav["v"][i] - vr[i]) for i in range(3)]
        force = v_scale(q_rotate(q_conj(nav["q"]), acc), m)
        qe = q_mul(q_conj(qr), nav["q"])
        sgn = 1.0 if qe[3] >= 0.0 else -1.0
        wc = v_add(v_scale(qe[:3], -2.0 * self.KP_ATT / self.KD_ATT * sgn), q_rotate(q_conj(qe), wr))
        I = P["inertia"]
        w = nav["w"]
        torque = v_add(v_scale(v_sub(wc, w), self.KD_ATT * I), v_cross(w, v_scale(w, I)))
        return force, torque


# =============================================================================
# Plant + world
# =============================================================================
class World(object):
    def __init__(self, payload, seed=0, dt=0.01, fan_noise=0.012, dist_force=0.0015,
                 dist_torque=2e-5, bias_force=0.002, nav_delay=0.1, start=None,
                 fan_scale=1.0, ignore_user_wrench=False):
        self.fan_scale = fan_scale                    # real fans weaker/stronger than the model
        self.ignore_user_wrench = ignore_user_wrench  # failure injection: user wrench never reaches fans
        self.rng = random.Random(seed * 7919 + 17)
        self.dt = dt
        self.t = 0.0
        self.model = U.CtlModel()
        self.ctl = CtlEmulator(self.model)
        self.judge = Judge(payload)
        self.payload = payload
        self.r = list(start or [0.2, 0.0, 0.0])
        self.v = [0.0, 0.0, 0.0]
        self.q = [0.0, 0.0, 0.0, 1.0]
        self.w = [0.0, 0.0, 0.0]
        self.fan_noise = fan_noise
        self.dist_sigma_f = dist_force
        self.dist_sigma_t = dist_torque
        self.ou_f = [0.0, 0.0, 0.0]
        self.ou_t = [0.0, 0.0, 0.0]
        d = v_unit([self.rng.gauss(0, 1) for _ in range(3)])
        self.bias = v_scale(d, bias_force)
        self.delay_steps = max(0, int(round(nav_delay / dt)))
        self.hist = []
        self.nav_bias = [-0.0021, -0.0018, -0.000067]
        self.nav_std = [0.00085, 0.00085, 0.00017]
        self.fast = None            # FastController attached by the mission (FAST mode)
        self.wrench_from_user = 0   # diagnostics
        self._record()

    def _record(self):
        self.hist.append({"t": self.t, "r": list(self.r), "v": list(self.v), "q": list(self.q), "w": list(self.w)})
        if len(self.hist) > self.delay_steps + 2:
            self.hist.pop(0)

    def nav(self):
        h = self.hist[0] if len(self.hist) > self.delay_steps else self.hist[0]
        r = [h["r"][i] + self.nav_bias[i] + self.rng.gauss(0.0, self.nav_std[i]) for i in range(3)]
        return {"t": self.t, "r": r, "v": list(h["v"]), "q": list(h["q"]), "w": list(h["w"])}

    def step(self):
        nav = self.nav()
        F_cmd, T_cmd = self.ctl.step(nav)
        # ctl is silent in STAND_BY -> fsm only receives the user's /ctl/wrench
        if self.ctl.status == "STAND_BY" and self.fast is not None and self.fast.active():
            out = self.fast.wrench(nav["t"], nav["r"], nav["v"], nav["q"], nav["w"])
            if out is not None and not self.ignore_user_wrench:
                F_cmd, T_cmd = list(out[0]), list(out[1])
                self.wrench_from_user += 1
        # fan allocation (thrust_allocator::allocate) + saturation + noise
        f = self.model.alloc(F_cmd + T_cmd)
        mx = max(f)
        if mx > P["fan_fmax"]:
            f = [fi * P["fan_fmax"] / mx for fi in f]
        F = [0.0, 0.0, 0.0]
        T = [0.0, 0.0, 0.0]
        for i in range(8):
            fi = max(0.0, f[i] * self.fan_scale + (self.rng.gauss(0.0, self.fan_noise) if self.fan_noise > 0 and f[i] > 0 else 0.0))
            fv = v_scale(FAN_VEC[i], fi)
            F = v_add(F, fv)
            T = v_add(T, v_cross(v_sub(FAN_POS[i], CG), fv))
        # disturbances (world frame force, body torque) -- OU processes, tau = 5 s
        a = math.exp(-self.dt / 5.0)
        s = math.sqrt(1.0 - a * a)
        self.ou_f = [a * self.ou_f[i] + s * self.dist_sigma_f * self.rng.gauss(0, 1) for i in range(3)]
        self.ou_t = [a * self.ou_t[i] + s * self.dist_sigma_t * self.rng.gauss(0, 1) for i in range(3)]
        Fw = v_add(v_add(q_rotate(self.q, F), self.ou_f), self.bias)
        m, I, dt = P["mass"], P["inertia"], self.dt
        self.v = v_add(self.v, v_scale(Fw, dt / m))
        self.r = v_add(self.r, v_scale(self.v, dt))
        Tb = v_add(T, self.ou_t)
        # Euler's equation with spherical inertia: I dw/dt = T - w x (I w) = T
        self.w = v_add(self.w, v_scale(Tb, dt / I))
        self.q = U.q_integrate_body(self.q, self.w, dt)
        self.t += dt
        self._record()

    def advance(self, duration):
        n = max(1, int(round(duration / self.dt)))
        for _ in range(n):
            self.step()


# =============================================================================
# Offline api for Mission
# =============================================================================
class OfflineApi(object):
    def __init__(self, world, verbose=False):
        self.w = world
        self.verbose = verbose
        self.logs = []
        self._gid = 0
        self._last = None
        self.stopped = False
        self._fast = None
        self.events = []            # (t, name) of safety-relevant calls, for the crash-path tests
        self.zero_wrench_count = 0

    def now(self):
        return self.w.t

    def sleep(self, dt):
        self.w.advance(dt)

    def nav(self):
        n = self.w.nav()
        return n["r"], n["q"]

    # FAST mode hooks (mirror RosApi)
    def attach_fast_controller(self, ctrl):
        self.w.fast = ctrl
        self._fast = ctrl

    def nav_full(self):
        n = self.w.nav()
        return n["r"], n["q"], n["v"], n["w"]

    def nav_age(self):
        return 0.0

    def ctl_standby(self):
        self.events.append((self.w.t, "ctl_standby"))
        self.w.ctl.set_mode("STAND_BY", self.w.nav())
        return True

    def ctl_keep_pose(self, server_s=None, result_s=None):
        self.events.append((self.w.t, "ctl_keep_pose"))
        self.w.ctl.set_mode("KEEP", self.w.nav())
        return True

    def publish_zero_wrench(self):
        # the world only applies the user wrench while the FAST controller is active, so a
        # zero wrench needs no plant effect here; it is recorded for the safety tests
        self.zero_wrench_count += 1
        self.events.append((self.w.t, "zero_wrench"))

    def send_goal(self, pos, q):
        self._gid += 1
        self._last = self._gid
        self.w.ctl.send_goal(self._gid, pos, q, self.w.nav())
        return True

    def goal_state(self):
        if self._last is None:
            return "NONE"
        return self.w.ctl.goals.get(self._last, "LOST")

    def cancel_goal(self):
        self.w.ctl.cancel(self.w.nav())

    def report_checkpoint(self, cid):
        n = self.w.nav()
        return self.w.judge.checkpoint(self.w.t, cid, n["r"], n["q"])

    def mission_start(self):
        self.w.judge.start(self.w.t)
        return True

    def mission_finished(self):
        self.events.append((self.w.t, "mission_finished"))
        n = self.w.nav()
        return self.w.judge.finish(self.w.t, n["r"], n["q"])

    def get_checkpoints(self):
        p = dict(self.w.payload)
        p = {"checkpoint_count": p["checkpoint_count"], "checkpoints": p["checkpoints"], "goalpoint_id": p["goalpoint_id"]}
        return json.dumps(p)

    def stop_requested(self):
        return self.stopped

    def log(self, level, text):
        self.logs.append((round(self.w.t, 2), level, text))
        if self.verbose:
            print("%7.2f [%s] %s" % (self.w.t, level, text))


# =============================================================================
# Baseline: what the official sample suggests (ID order, wait SUCCESS, report)
# =============================================================================
def run_baseline(api, payload):
    api.mission_start()
    specs, goal_id = U.parse_checkpoints(api.get_checkpoints())
    order = [s for s in specs if s.cid != goal_id] + [s for s in specs if s.cid == goal_id]
    t0 = api.now()
    for s in order:
        api.send_goal(s.position, s.target_q)
        while api.goal_state() not in ("SUCCEEDED", "PREEMPTED", "ABORTED") and api.now() - t0 < 600:
            api.sleep(0.1)
        api.report_checkpoint(s.cid)
    api.mission_finished()


def simulate(level, seed, strategy="ours", cfg=None, world_kw=None, verbose=False):
    payload = ScenarioGenerator().generate(level, seed)
    world = World(payload, seed=seed, **(world_kw or {}))
    api = OfflineApi(world, verbose=verbose)
    t_wall = time.time()
    if strategy in ("fast", "safe", "ours"):
        c = dict(cfg or {})
        if strategy != "ours":
            c["MOTION_MODE"] = strategy
        mission = U.Mission(api, cfg=c)
        mission.run()
        fast = mission.fast
    else:
        run_baseline(api, payload)
        fast = None
    res = world.judge.score()
    res.update({"level": level, "seed": seed, "strategy": strategy, "wall_s": time.time() - t_wall,
                "fault": bool(fast and fast.fault),
                "max_pos_err": fast.max_pos_err if fast else None,
                "max_att_err": fast.max_att_err if fast else None})
    return res, api


def main():
    ap = argparse.ArgumentParser(description="Offline Monte Carlo for Kibo-RPC 7th")
    ap.add_argument("--levels", default="easy,normal,hard,veryhard")
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--seed0", type=int, default=1)
    ap.add_argument("--strategies", default="ours,baseline")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--bias", type=float, default=0.002, help="constant disturbance force [N]")
    ap.add_argument("--noise", type=float, default=0.012, help="fan force noise std [N]")
    ap.add_argument("--delay", type=float, default=0.1, help="navigation delay [s]")
    ap.add_argument("--dist", type=float, default=0.0015, help="random airflow force std [N]")
    ap.add_argument("--fan_scale", type=float, default=1.0, help="real fan force / modelled fan force")
    ap.add_argument("--ignore_user_wrench", action="store_true", help="failure injection test")
    ap.add_argument("--out", default=os.path.join(HERE, "results"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rows = []
    wk = {"bias_force": args.bias, "fan_noise": args.noise, "nav_delay": args.delay, "dist_force": args.dist,
          "fan_scale": args.fan_scale, "ignore_user_wrench": args.ignore_user_wrench}
    for level in args.levels.split(","):
        for strat in args.strategies.split(","):
            for k in range(args.seeds):
                seed = args.seed0 + k
                res, api = simulate(level, seed, strat, world_kw=wk, verbose=args.verbose)
                rows.append(res)
                print("%-9s %-8s seed=%3d T=%6.1fs in_time=%-5s goal=%-5s lv3=%2d/%2d base=%5.1f mag=%.3f score=%6.2f fault=%s (%.1fs wall)" % (
                    level, strat, seed, res["T"], res["in_time"], res["goal"], res["n_lv3"], res["n_rules"],
                    res["base"], res["mag"], res["score"], res.get("fault"), res["wall_s"]))
                sys.stdout.flush()
    with open(os.path.join(args.out, "montecarlo.json"), "w") as f:
        json.dump(rows, f, indent=1)
    print("\n=== SUMMARY (mean over seeds) ===")
    print("%-9s %-8s %8s %8s %8s %8s %8s" % ("level", "strategy", "T[s]", "in_time", "goal%", "Lv3%", "score"))
    for level in args.levels.split(","):
        for strat in args.strategies.split(","):
            rs = [r for r in rows if r["level"] == level and r["strategy"] == strat]
            if not rs:
                continue
            n = float(len(rs))
            T = sum(min(r["T"], 999) for r in rs) / n
            it = sum(1 for r in rs if r["in_time"]) / n * 100
            g = sum(1 for r in rs if r["goal"]) / n * 100
            l3 = sum(r["n_lv3"] / float(r["n_rules"]) for r in rs) / n * 100
            sc = sum(r["score"] for r in rs) / n
            print("%-9s %-8s %8.1f %7.0f%% %7.0f%% %7.0f%% %8.2f" % (level, strat, T, it, g, l3, sc))


if __name__ == "__main__":
    main()
