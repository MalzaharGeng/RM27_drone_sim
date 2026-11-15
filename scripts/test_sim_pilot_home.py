#!/usr/bin/env python3
"""飞手回中坐标契约的针对性测试（不启动 PX4 / DDS）。

覆盖三个阶段 3 已证实的缺陷：
  1. 回中目标被隐式当成 PX4 本地原点 (0,0)，而不是捕获的同域锚点；
  2. NED 世界系误差未按机头偏航旋到机体系，直接映射到 ch1/ch2；
  3. VIO/位置/航向失效后仍沿用上一拍的饱和摇杆。
运行：python3 -m unittest test_sim_pilot_home -v
"""
import json
import math
import time
import unittest
from types import SimpleNamespace

import sim_pilot


class _Logger:
    def __init__(self):
        self.lines = []

    def warning(self, msg):
        self.lines.append(("warning", msg))

    def info(self, msg):
        self.lines.append(("info", msg))


class _Msg:
    def __init__(self, kind, **fields):
        self._kind = kind
        self._fields = fields

    def get_type(self):
        return self._kind

    def get_srcSystem(self):
        return 1

    def __getattr__(self, name):
        return self._fields[name]


class _Mav:
    def __init__(self, messages):
        self._messages = list(messages)

    def recv_match(self, blocking=False):
        return self._messages.pop(0) if self._messages else None


class _Health:
    def __init__(self, payload):
        self.data = json.dumps(payload)


def fake_pilot(**kw):
    """构造不经过 __init__ 的 Pilot，只带被测方法需要的字段。"""
    p = object.__new__(sim_pilot.Pilot)
    p.get_logger = lambda: p._logger
    p._logger = _Logger()
    for k, v in kw.items():
        setattr(p, k, v)
    return p


class StickMappingTest(unittest.TestCase):
    """世界系误差 -> 机体系摇杆。"""

    def test_target_is_a_captured_anchor_not_the_local_origin(self):
        # 同一相对几何、整体平移后摇杆必须完全相同：把目标写死成 (0,0)
        # 的实现会在这里给出不同（或直接反向）的指令。
        home = (12.413, 14.785)
        pos = (7.259, 16.090)
        offset = (100.0, -50.0)
        base = sim_pilot.sticks_for_home(home, pos, 1.593)
        shifted = sim_pilot.sticks_for_home(
            (home[0] + offset[0], home[1] + offset[1]),
            (pos[0] + offset[0], pos[1] + offset[1]), 1.593)
        self.assertEqual(base, shifted)
        # 103614 实测：home 锚点在本地系 (12.4,14.8)，飞机在 (7.3,16.1)。
        # 机头约 91°，修正应同时向"后"和"左"打，两个通道都应饱和到低端。
        self.assertEqual(base, (1420, 1420))

    def test_yaw_rotates_world_error_into_body_frame(self):
        # 同一个世界系误差（正东，+y），机头朝北时是"右"（ch1>1500），
        # 机头朝东时是"前"（ch2>1500）。旧实现不旋转，把 +x 打到 ch2、
        # +y 打到 ch1，在 yaw≈96° 的实际航向上把作用轴搞反。
        ch1, ch2 = sim_pilot.sticks_for_home(home=(0.0, 1.0), pos=(0.0, 0.0), yaw=0.0)
        self.assertEqual((ch1, ch2), (1580, 1500))

        ch1, ch2 = sim_pilot.sticks_for_home(home=(0.0, 1.0), pos=(0.0, 0.0),
                                             yaw=math.pi / 2)
        self.assertEqual((ch1, ch2), (1500, 1580))

        # 正北误差 (+x) 在机头朝东时位于左侧，应由 roll 左（ch1<1500）修正。
        ch1, ch2 = sim_pilot.sticks_for_home(home=(1.0, 0.0), pos=(0.0, 0.0),
                                             yaw=math.pi / 2)
        self.assertEqual((ch1, ch2), (1420, 1500))

    def test_commanded_world_direction_reduces_the_error(self):
        # 用真实日志的数值检查：把摇杆指令按 PX4 的机体系语义旋回世界系，
        # 其方向必须与误差方向同向（点积 > 0）。这能抓住符号/轴交换错误。
        home = (12.413, 14.785)
        pos = (7.259, 16.090)
        yaw = 1.593
        ch1, ch2 = sim_pilot.sticks_for_home(home, pos, yaw)
        fwd = (ch2 - 1500) / sim_pilot.HOME_STICK_GAIN
        right = (ch1 - 1500) / sim_pilot.HOME_STICK_GAIN
        c, s = math.cos(yaw), math.sin(yaw)
        world = (fwd * c - right * s, fwd * s + right * c)
        n = math.hypot(*world)
        e = (home[0] - pos[0], home[1] - pos[1])
        en = math.hypot(*e)
        dot = (world[0] * e[0] + world[1] * e[1]) / (n * en)
        self.assertGreater(dot, 0.5, "commanded direction=%r error=%r" % (world, e))

    def test_deadzone_centres_and_boundary_is_exact(self):
        self.assertEqual(sim_pilot.sticks_for_home((0.3, 0.0), (0.0, 0.0), 0.0),
                         (1500, 1500))
        ch1, ch2 = sim_pilot.sticks_for_home((0.36, 0.0), (0.0, 0.0), 0.0)
        self.assertEqual((ch1, ch2), (1500, 1528))

    def test_invalid_inputs_fail_closed(self):
        for home, pos, yaw in ((None, (0.0, 0.0), 0.0),
                               ((1.0, 0.0), None, 0.0),
                               ((1.0, 0.0), (0.0, 0.0), None)):
            with self.subTest(home=home, pos=pos, yaw=yaw):
                self.assertEqual(sim_pilot.sticks_for_home(home, pos, yaw),
                                 (1500, 1500))

    def test_sticks_are_clamped(self):
        self.assertEqual(sim_pilot.sticks_for_home((50.0, 0.0), (0.0, 0.0), 0.0),
                         (1500, 1580))
        self.assertEqual(sim_pilot.sticks_for_home((-50.0, 0.0), (0.0, 0.0), 0.0),
                         (1500, 1420))


class AnchorLifecycleTest(unittest.TestCase):
    """锚点捕获与 reset 处理。"""

    def _base(self, **kw):
        # [NP-01 FIX16] a real Pilot always carries these; drain()/on_vio_health()
        # now consult fresh-readiness (vio_rx_mono) and the marker arguments.
        fields = dict(vio_ready=True, vio_reason="", home=None,
                      home_reset=None, home_pending=True,
                      vio_rx_mono=time.monotonic(), vio_seq=0, home_logged=False,
                      args=SimpleNamespace(ready_stale_s=0.8, ack_marker="",
                                           warn_marker=""))
        fields.update(kw)
        return fake_pilot(**fields)

    def test_capture_waits_for_navigation_ready(self):
        # 未就绪时收到本地位置不得捕获：那时 EKF 还没有外部位置。
        p = self._base(vio_ready=False, hx=None, hy=None, z=None,
                       mav=_Mav([_Msg("LOCAL_POSITION_NED", x=1.0, y=2.0, z=-1.0)]))
        p.drain()
        self.assertIsNone(p.home)
        self.assertTrue(p.home_pending)

        p = self._base(vio_ready=True, hx=None, hy=None, z=None,
                       mav=_Mav([_Msg("LOCAL_POSITION_NED", x=1.0, y=2.0, z=-1.0)]))
        p.drain()
        self.assertEqual(p.home, (1.0, 2.0))
        self.assertFalse(p.home_pending)

    def test_xy_reset_drops_the_old_anchor_and_recaptures(self):
        p = self._base(home=(12.413, 14.785), home_reset=2, home_pending=False,
                       hx=None, hy=None, z=None)
        p.on_vio_health(_Health({"navigation_ready": True,
                                 "px4_xy_reset_counter": 3}))
        self.assertIsNone(p.home)
        self.assertTrue(p.home_pending)
        self.assertEqual(p.home_reset, 3)
        self.assertTrue(any(kind == "warning" for kind, _ in p._logger.lines))

        p.mav = _Mav([_Msg("LOCAL_POSITION_NED", x=-2.0, y=-4.0, z=-1.0)])
        p.drain()
        self.assertEqual(p.home, (-2.0, -4.0))
        self.assertFalse(p.home_pending)

    def test_unchanged_reset_counter_keeps_the_anchor(self):
        p = self._base(home=(1.0, 2.0), home_reset=3, home_pending=False)
        p.on_vio_health(_Health({"navigation_ready": True,
                                 "px4_xy_reset_counter": 3}))
        self.assertEqual(p.home, (1.0, 2.0))
        self.assertFalse(p.home_pending)


class PositionStageTest(unittest.TestCase):
    """POSITION 状态：失效回中 + 门限仍按固定锚点。"""

    def _pilot(self, **kw):
        # [NP-01 FIX16] POSITION now goes through ready_is_fresh(); the fixture
        # must carry the fields a real Pilot always has.
        args = SimpleNamespace(no_home_gate=False, position_hold=8.0,
                               skip_ready_gate=False, slot_position=1234,
                               slot_altitude=1000, slot_offboard=1400,
                               ready_stale_s=0.8, ack_marker="", warn_marker="")
        fields = dict(state="POSITION", state_since=0.0, bt={}, args=args,
                      vio_ready=True, vio_rx_mono=time.monotonic(),
                      home=(0.0, 0.0), hx=0.0, hy=0.0,
                      yaw=0.0, yaw_time=time.monotonic(),
                      ch1=1420, ch2=1420, ch3=1500, ch5=1000,
                      home_logged=True, detent_retries=0,
                      detent_retry_pending=False, detent_retry_since=0.0)
        fields.update(kw)
        p = fake_pilot(**fields)
        p.states = []
        p.set_state = p.states.append
        return p

    def test_lost_vio_centres_the_sticks_instead_of_reusing_them(self):
        p = self._pilot(vio_ready=False)
        p.step(time.monotonic())
        self.assertEqual((p.ch1, p.ch2), (1500, 1500))
        self.assertEqual(p.states, [])

    def test_stale_yaw_centres_the_sticks(self):
        p = self._pilot(yaw_time=time.monotonic() - 5.0, hx=-2.0)
        p.step(time.monotonic())
        self.assertEqual((p.ch1, p.ch2), (1500, 1500))

    def test_gate_uses_the_anchor_not_the_origin(self):
        ready = {"offboard_ready": True}
        # 飞机在本地系 (0.2,0.1)（距本地原点 0.22 m，旧逻辑会判"到点"），
        # 但距锚点 (5,6) 仍有 7.7 m：不得放行。
        p = self._pilot(bt=ready, home=(5.0, 6.0), hx=0.2, hy=0.1,
                        state_since=time.monotonic() - 30.0)
        p.step(time.monotonic())
        self.assertNotIn("OFFBOARD_ARMED", p.states)

        # 到达锚点附近才放行。
        p = self._pilot(bt=ready, home=(5.0, 6.0), hx=5.1, hy=5.9,
                        state_since=time.monotonic() - 30.0)
        p.step(time.monotonic())
        self.assertEqual(p.states, ["OFFBOARD_ARMED"])

    def test_missing_home_never_reports_at_home(self):
        p = self._pilot(bt={"offboard_ready": True}, home=None, hx=0.0, hy=0.0,
                        state_since=time.monotonic() - 100.0)
        p.step(time.monotonic())
        self.assertNotIn("OFFBOARD_ARMED", p.states)


if __name__ == "__main__":
    unittest.main()
