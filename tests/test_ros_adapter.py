# -*- coding:utf-8 -*-
"""Exercise every RosApi / UserTemplate code path with stub ROS modules.

Catches NameError / AttributeError / wrong call signatures in the ROS-only part of
user_template.py before it ever reaches the official simulator.
Run: python tests/test_ros_adapter.py
"""
import importlib.util
import json
import os
import sys
import threading
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCRIPT = os.path.join(ROOT, "krpc_user", "scripts", "user_template.py")

LOG = []
EVENTS = []   # ordered record of safety-relevant traffic: ("wrench", is_zero) / ("goal", name, type) / ("srv", name)


class Msg(object):
    """Generic ROS message stand-in: nested attribute bag."""

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        m = Msg()
        object.__setattr__(self, name, m)
        return m


def _mod(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


class FakeTime(object):
    @staticmethod
    def from_sec(s):
        return ("t", s)

    @staticmethod
    def now():
        return ("t", time.time())


class FakeDuration(object):
    def __init__(self, s=0.0):
        self.s = s


class Publisher(object):
    instances = []

    def __init__(self, topic, typ, queue_size=1):
        self.topic, self.msgs = topic, []
        Publisher.instances.append(self)

    def publish(self, msg):
        self.msgs.append(msg)
        if self.topic == "/ctl/wrench":
            f, t = msg.wrench.force, msg.wrench.torque
            EVENTS.append(("wrench", all(abs(v) == 0.0 for v in (f.x, f.y, f.z, t.x, t.y, t.z))))


class Subscriber(object):
    instances = {}

    def __init__(self, topic, typ, cb, queue_size=1, buff_size=None):
        Subscriber.instances[topic] = cb


class Service(object):
    instances = {}

    def __init__(self, name, typ, cb):
        Service.instances[name] = cb


class ServiceProxy(object):
    def __init__(self, name, typ):
        self.name = name

    def __call__(self):
        EVENTS.append(("srv", self.name))
        if self.name == "/mission/get_checkpoints":
            return Msg(success=True, message=json.dumps(PAYLOAD))
        return Msg(success=True, message="accepted")


class SimpleActionClient(object):
    def __init__(self, name, typ):
        self.name, self.goals, self.state = name, [], 0

    def wait_for_server(self, d=None):
        return True

    def send_goal(self, goal):
        self.goals.append(goal)
        if self.name == "/ctl/command":
            EVENTS.append(("goal", self.name, goal.type.type))
        self.state = 3  # SUCCEEDED immediately

    def get_state(self):
        return self.state

    def get_result(self):
        if self.name == "/ctl/command":
            return Msg(type=0)
        return Msg(success=True, message="accepted")

    def wait_for_result(self, d=None):
        return True

    def cancel_goal(self):
        self.state = 2


def setup_stubs():
    params = {}
    _mod("rospy", init_node=lambda *a, **k: None, Publisher=Publisher, Subscriber=Subscriber, Service=Service,
         ServiceProxy=ServiceProxy, set_param=lambda k, v: params.__setitem__(k, v), get_time=time.time,
         sleep=lambda s: time.sleep(min(s, 0.001)), wait_for_service=lambda *a, **k: None,
         Duration=FakeDuration, Time=FakeTime, is_shutdown=lambda: False, on_shutdown=lambda cb: None,
         spin=lambda: None, loginfo=lambda *a: LOG.append(("info",) + a), logwarn=lambda *a: LOG.append(("warn",) + a),
         logerr=lambda *a: LOG.append(("error",) + a))
    _mod("actionlib", SimpleActionClient=SimpleActionClient)
    _mod("actionlib_msgs")
    _mod("actionlib_msgs.msg", GoalStatus=Msg)
    _mod("geometry_msgs")
    _mod("geometry_msgs.msg", PoseStamped=Msg, WrenchStamped=Msg)
    _mod("platform_msgs")
    _mod("platform_msgs.msg", UserLogic=Msg)

    class StopResp(Msg):
        SUCCESS, ERROR = 0, 1
    _mod("platform_msgs.srv", StopProcessingUserNode=Msg, StopProcessingUserNodeResponse=StopResp)
    _mod("krpc_ib2_msgs")
    _mod("krpc_ib2_msgs.srv", MissionStart=Msg, MissionFinished=Msg, CheckPoints=Msg)
    _mod("krpc_ib2_msgs.msg", CheckpointPassedAction=Msg, CheckpointPassedGoal=Msg)

    class CtlStatusType(Msg):
        STAND_BY, KEEP_POSE, MOVE_TO_ABSOLUTE_TARGET = 0, 10, 40

    class CtlCommandResult(Msg):
        TERMINATE_SUCCESS = 0
    _mod("ib2_msgs")
    _mod("ib2_msgs.msg", CtlCommandAction=Msg, CtlCommandGoal=Msg, CtlCommandResult=CtlCommandResult,
         CtlStatusType=CtlStatusType, Navigation=Msg)
    return params


def nav_msg(p, q, v=(0, 0, 0), w=(0, 0, 0)):
    m = Msg()
    m.pose.pose.position = Msg(x=p[0], y=p[1], z=p[2])
    m.pose.pose.orientation = Msg(x=q[0], y=q[1], z=q[2], w=q[3])
    m.twist.linear = Msg(x=v[0], y=v[1], z=v[2])
    m.twist.angular = Msg(x=w[0], y=w[1], z=w[2])
    return m


PAYLOAD = None


def main():
    global PAYLOAD
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    import offline_sim as S
    PAYLOAD = {k: v for k, v in S.ScenarioGenerator().generate("easy", 4).items() if k in ("checkpoint_count", "checkpoints", "goalpoint_id")}
    params = setup_stubs()
    spec = importlib.util.spec_from_file_location("ut_ros", SCRIPT)
    U = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(U)
    assert U.ROS_AVAILABLE, "stubs not picked up"

    node = U.UserTemplate()
    assert params.get("/ib2_user/ready") is True
    api = node.api
    # nav callback (no fast controller yet)
    Subscriber.instances["/sensor_fusion/navigation"](nav_msg([0.2, 0, 0], [0, 0, 0, 1]))
    assert api.nav() is not None and api.nav_full() is not None and api.nav_age() < 1.0
    # services and actions
    assert api.mission_start() and api.mission_finished()
    specs, goal = U.parse_checkpoints(api.get_checkpoints())
    assert specs and goal == 12
    assert api.send_goal([1, 0, 0.5], [0, 0, 0, 1]) and api.goal_state() == "SUCCEEDED"
    assert api.report_checkpoint(3) is True
    assert api.ctl_standby() and api.ctl_keep_pose()
    api.cancel_goal()
    # v4: ArUco alignment is off by default -> no camera subscriber, observe() returns None
    assert U.CONFIG["ARUCO_ALIGN"] == "off" and node.camera is None and api.aruco_observe(1) is None
    assert "/camera_main/image_raw" not in Subscriber.instances
    # FAST controller path through the nav callback
    model = U.CtlModel()
    fc = U.FastController(model)
    api.attach_fast_controller(fc)
    fc.set_profile(U.FastProfile(model, time.time(), [0.2, 0, 0], [0, 0, 0, 1], [1.2, 0.3, 0.5], specs[0].target_q))
    fc.enable(True)
    wp = [p for p in Publisher.instances if p.topic == "/ctl/wrench"][0]
    n0 = len(wp.msgs)
    time.sleep(0.02)
    Subscriber.instances["/sensor_fusion/navigation"](nav_msg([0.2, 0, 0], [0, 0, 0, 1]))
    assert len(wp.msgs) == n0 + 1, "wrench not published from nav callback"
    m = wp.msgs[-1]
    assert m.header.frame_id == "body" and isinstance(m.wrench.force.x, float)
    api.publish_zero_wrench()
    assert wp.msgs[-1].wrench.force.x == 0.0
    # start callback launches the mission thread; stop service works
    th_before = threading.active_count()
    Subscriber.instances["/ib2_user/start"](Msg(id=1))
    resp = Service.instances["/ib2_user/stop"](Msg())
    assert resp.result == 0
    time.sleep(0.2)
    errors = [l for l in LOG if l[0] == "error"]
    assert not errors, errors
    print("ROS adapter: all code paths OK (%d log lines, %d wrench msgs, threads %d)" % (len(LOG), len(wp.msgs), th_before))
    n_ok = 1
    for fn in (check_crash_path, check_stop_handler, check_deadline_watchdog, check_stop_before_start):
        fn(U)
        n_ok += 1
    print("ROS adapter: ALL %d CHECKS PASSED" % n_ok)


# ---------------------------------------------------------------------------
# v3 (23 Oct 2026): crash path, /ib2_user/stop and deadline watchdog share safe_stop()
# ---------------------------------------------------------------------------
KEEP_POSE = 10
EMPTY_PLAN = {"order": [], "modes": [], "t_route": 0.0, "t_full": 0.0, "dropped": []}


def _wait(cond, timeout=5.0):
    t = time.time()
    while time.time() - t < timeout:
        if cond():
            return True
        time.sleep(0.005)
    return cond()


def _new_node(U):
    node = U.UserTemplate()
    Subscriber.instances["/sensor_fusion/navigation"](nav_msg([0.2, 0, 0], [0, 0, 0, 1]))
    return node


def _idx(ev, pred):
    return [i for i, e in enumerate(ev) if pred(e)]


def _assert_safe_stop(ev, need_finish_after=True):
    zeros = _idx(ev, lambda e: e == ("wrench", True))
    keeps = _idx(ev, lambda e: e[0] == "goal" and e[2] == KEEP_POSE)
    fins = _idx(ev, lambda e: e == ("srv", "/event/mission_finished"))
    assert keeps, ("no ctl KEEP_POSE goal", ev)
    k = keeps[0]
    assert len([i for i in zeros if i < k]) >= 3, ("<3 zero wrenches before KEEP_POSE", ev)
    assert not [e for e in ev[zeros[0]:] if e == ("wrench", False)], ("non-zero wrench after safe stop", ev)
    if need_finish_after:
        assert len(fins) == 1 and fins[0] > k, ("mission_finished must come once, after KEEP_POSE", ev)
    return fins


def check_crash_path(U):
    """Mission.run() raises while FAST is enabled -> safe stop, then one mission_finished."""
    orig = U.Planner.plan

    def boom(self, *a, **k):
        raise RuntimeError("injected planner crash")
    U.Planner.plan = boom
    try:
        node = _new_node(U)
        i0 = len(EVENTS)
        Subscriber.instances["/ib2_user/start"](Msg(id=2))
        assert _wait(lambda: node.mission is not None and node.mission.finished), "crash path never finished"
        m = node.mission
        assert m.fast is not None and not m.fast.enabled, "FAST still enabled after crash"
        _wait(lambda: False, 0.05)
        _assert_safe_stop(EVENTS[i0:])
        assert any("mission crashed" in str(l) for l in LOG)
        # a late nav sample must not produce a wrench any more
        i1 = len(EVENTS)
        Subscriber.instances["/sensor_fusion/navigation"](nav_msg([0.3, 0, 0], [0, 0, 0, 1]))
        assert not [e for e in EVENTS[i1:] if e[0] == "wrench"]
    finally:
        U.Planner.plan = orig
    print("ROS crash path: FAST off -> 3x zero wrench -> KEEP_POSE -> 1x mission_finished: OK")


def check_stop_handler(U):
    orig = U.Planner.plan
    holder = {}

    def wait_for_stop(self, *a, **k):
        _wait(lambda: holder["node"].stop_flag, 10.0)
        return dict(EMPTY_PLAN)
    U.Planner.plan = wait_for_stop
    try:
        node = _new_node(U)
        holder["node"] = node
        Subscriber.instances["/ib2_user/start"](Msg(id=3))
        assert _wait(lambda: node.mission is not None and node.mission.fast is not None and node.mission.fast.enabled)
        i0 = len(EVENTS)
        resp = Service.instances["/ib2_user/stop"](Msg())
        assert resp.result == 0
        assert not node.mission.fast.enabled, "stop handler left FAST enabled"
        _assert_safe_stop(EVENTS[i0:], need_finish_after=False)
        assert _wait(lambda: node.mission.finished)
        _wait(lambda: False, 0.05)
        fins = _idx(EVENTS[i0:], lambda e: e == ("srv", "/event/mission_finished"))
        assert len(fins) == 1, EVENTS[i0:]
    finally:
        U.Planner.plan = orig
    print("ROS /ib2_user/stop: same safe stop, response SUCCESS, mission thread finishes once: OK")


def check_deadline_watchdog(U):
    """Mission thread hangs -> the independent watchdog stops the robot and sends mission_finished."""
    orig_plan, orig_limit = U.Planner.plan, U.CONFIG["T_LIMIT_S"]
    release = threading.Event()

    def hang(self, *a, **k):
        release.wait(10.0)
        return dict(EMPTY_PLAN)
    U.Planner.plan = hang
    U.CONFIG["T_LIMIT_S"] = U.CONFIG["DEADLINE_WATCHDOG_S"] + 0.3   # watchdog fires 0.3 s after start
    try:
        node = _new_node(U)
        i0 = len(EVENTS)
        Subscriber.instances["/ib2_user/start"](Msg(id=4))
        assert _wait(lambda: node.mission is not None and node.mission.finished, 5.0), "watchdog did not finish"
        m = node.mission
        assert node.stop_flag and not m.fast.enabled
        assert any("deadline watchdog" in str(l) for l in LOG)
        t_fin = [float(str(l[1]).split("t=")[1].split("s")[0]) for l in LOG
                 if "mission_finished sent at" in str(l[1:])][-1]
        assert t_fin < U.CONFIG["T_LIMIT_S"], (t_fin, U.CONFIG["T_LIMIT_S"])
        _wait(lambda: False, 0.05)
        _assert_safe_stop(EVENTS[i0:])
        release.set()
        _wait(lambda: False, 0.2)
        fins = _idx(EVENTS[i0:], lambda e: e == ("srv", "/event/mission_finished"))
        assert len(fins) == 1, "mission thread sent a second mission_finished"
    finally:
        release.set()
        U.Planner.plan = orig_plan
        U.CONFIG["T_LIMIT_S"] = orig_limit
    print("ROS deadline watchdog: hung mission -> safe stop + 1x mission_finished before T_LIMIT: OK")


def check_stop_before_start(U):
    """/ib2_user/stop before any mission start must not send a ctl command (robot never moved)."""
    node = _new_node(U)
    i0 = len(EVENTS)
    resp = Service.instances["/ib2_user/stop"](Msg())
    assert resp.result == 0 and node.stop_flag
    assert not [e for e in EVENTS[i0:] if e[0] in ("goal", "wrench")], EVENTS[i0:]
    print("ROS /ib2_user/stop before start: no ctl goal, no wrench: OK")


if __name__ == "__main__":
    main()
