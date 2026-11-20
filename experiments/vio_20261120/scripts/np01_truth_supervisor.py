#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[NP-01 FIX16] Low-latency independent TRUTH supervisor (SIM-ONLY).

Why this file exists
--------------------
`truth_safety_watchdog.py` read truth.csv and therefore inherited three serial
delays that were measured on round 14 (2026-11-15 20:20):

    truth_recorder flush every 200 rows  ->  up to 4.0 s   (50 Hz stream)
    watchdog poll 0.5 s                  ->  up to 0.5 s
    pilot FAULT dwell 2.0 s + runner 1 s ->  up to 3.0 s
    ------------------------------------------------------
    worst observed 3.98 s of continued PHYSICAL motion = 1.85 m

This process removes the disk from the safety path entirely:

    Gazebo dynamic_pose -> ros_gz_bridge -> this node (rclpy, 50 Hz)
        -> decision in the SAME callback
        -> warning: marker + ACK contract (pilot confirms receipt/action)
        -> stop: `gz service /world/<world>/control pause:true` FIRST,
                 then evidence, then (only if pause is not confirmed) a
                 precisely-targeted hard stop of the gz server process.

Hard contract
-------------
* READS truth pose only. It never publishes a position/velocity to PX4, never
  writes pilot.csv, never touches Gate / LIMITS / age / fusion / acceptance.
* Truth is an independent monitor for CANCEL/TERMINATE only.
* CSV is evidence, never the safety path: rows are buffered and flushed every
  `--flush-rows` (default 100) rows or when an event fires.
* Two clearance quantities are kept apart and never mixed:
      wall_clearance_m      = base_link origin -> nearest wall inner face
      wall_clearance_net_m  = wall_clearance_m - rotor envelope
  The warn/stop thresholds apply to `wall_clearance_net_m` (this is what the
  previous version already did; the field names now say so explicitly).
* Measured latency segments are written to supervisor.json. No worst-case
  claim is derived from a single sample; the report states sample statistics.
"""
import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

def _now_us():
    return time.monotonic_ns() // 1000


def quat_angle_deg(q0, q1):
    d = abs(sum(q0[i] * q1[i] for i in range(4)))
    return 2.0 * math.degrees(math.acos(min(1.0, max(-1.0, d))))


class TruthSupervisor(Node):
    def __init__(self, a):
        super().__init__("np01_truth_supervisor")
        self.a = a
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, )
        self.create_subscription(TFMessage, a.pose_topic, self.on_pose, qos)
        self.create_subscription(Clock, a.clock_topic, self.on_clock, qos)
        self.create_timer(0.02, self.tick)

        self.t_start_us = _now_us()
        self.datum = None
        self.prev = None
        self.n = 0
        self.lost_us = 0
        self.max_gap_ms = 0.0
        self.last_rx_us = None
        self.min_clearance_net = None
        self.min_clearance_net_event = None
        self.rate_hits = 0

        self.warned = False
        self.t_warn_us = None
        self.warn_event = None
        self.warn_ack = None
        self.stop_event = None
        self.t_stop_us = None
        self.stop_reason = None

        self.pause_proc = None
        self.pause_req = None
        self.pause_reply = None
        self.pause_attempts = 0
        self.pause_confirmed = None
        self.hard_kill = None

        self.last_sim = None
        self.pause_sim_at_reply = None
        self.sim_adv_after_pause = 0.0
        self.clock_rows_after_pause = 0
        self._pause_watch_t0 = None
        self._written = False
        self._exiting = False

        self.rows = []
        self.csv_path = os.path.join(a.run_dir, "supervisor_trace.csv")
        self._csv = open(self.csv_path, "w", newline="")
        self._wr = csv.writer(self._csv)
        self._wr.writerow([
            "t_mono_s", "t_wall_us", "x", "y", "z",
            "wall_clearance_m", "wall_clearance_net_m",
            "disp_xy_m", "dz_m", "speed_ms", "rx_interval_ms",
            "lost_us_total", "action"])
        self._flush_every = max(1, a.flush_rows)
        for stale in (a.stop_marker, a.warn_marker, a.ack_marker):
            try:
                os.unlink(stale)
            except OSError:
                pass
        self.get_logger().info(
            "[NP01_FIX16] supervisor up: pose=%s clock=%s world=%s "
            "warn_net=%.2fm stop_net=%.2fm envelope=%.6fm pause=%s geometry=%s"
            % (a.pose_topic, a.clock_topic, a.world, a.warn_clearance,
               a.stop_clearance, a.envelope_m, a.enable_pause,
               (a.geometry_used or {}).get("path", "--walls")))

    # ------------------------------------------------------------------ pose
    def on_pose(self, msg):
        t_rx = _now_us()
        if not msg.transforms:
            return
        tr = msg.transforms[0]
        t = tr.transform.translation
        q = tr.transform.rotation
        x, y, z = t.x, t.y, t.z
        if self.datum is None:
            self.datum = (t_rx, x, y, z)
        gap_us = (t_rx - self.last_rx_us) if self.last_rx_us else 0
        if self.last_rx_us and gap_us > self.a.loss_gap_us:
            self.lost_us += gap_us
        self.max_gap_ms = max(self.max_gap_ms, gap_us / 1000.0)
        self.last_rx_us = t_rx

        disp = math.hypot(x - self.datum[1], y - self.datum[2])
        dz = abs(z - self.datum[3])
        clr = min(x - self.a.west, self.a.east - x,
                  y - self.a.south, self.a.north - y)
        clr_net = clr - self.a.envelope_m
        rate = 0.0
        speed = float("nan")
        if self.prev is not None:
            dt = (t_rx - self.prev[0]) / 1e6
            if 1e-4 < dt < 1.0:
                rate = quat_angle_deg(self.prev[1], (q.w, q.x, q.y, q.z)) / dt
                speed = math.hypot(x - self.prev[2], y - self.prev[3]) / dt
        self.prev = (t_rx, (q.w, q.x, q.y, q.z), x, y)
        self.n += 1
        if self.min_clearance_net is None or clr_net < self.min_clearance_net:
            self.min_clearance_net = clr_net
            self.min_clearance_net_event = dict(
                t_mono_s=(t_rx - self.t_start_us) * 1e-6,
                wall_clearance_m=round(clr, 4),
                wall_clearance_net_m=round(clr_net, 4), x=x, y=y, z=z)
        if rate > self.a.max_rate_dps:
            self.rate_hits += 1
        else:
            self.rate_hits = 0

        action = ""
        if self.stop_reason is None:
            if clr_net < self.a.stop_clearance:
                action = "STOP"
                self._stop("truth_wall_clearance_stop", t_rx, dict(
                    wall_clearance_m=round(clr, 4),
                    wall_clearance_net_m=round(clr_net, 4),
                    disp_xy_m=round(disp, 4), dz_m=round(dz, 4),
                    angular_rate_deg_s=round(rate, 2)))
            elif clr_net < self.a.warn_clearance and not self.warned:
                action = "WARN"
                self._warn(t_rx, dict(
                    wall_clearance_m=round(clr, 4),
                    wall_clearance_net_m=round(clr_net, 4),
                    disp_xy_m=round(disp, 4), dz_m=round(dz, 4)))
            elif dz > self.a.max_z:
                action = "STOP"
                self._stop("truth_z_exceeded", t_rx, dict(dz_m=round(dz, 4)))
            elif self.rate_hits >= self.a.rate_samples:
                action = "STOP"
                self._stop("truth_angular_rate_exceeded", t_rx,
                           dict(angular_rate_deg_s=round(rate, 2)))
            elif disp > (self.a.max_xy_init
                         if (t_rx - self.datum[0]) < self.a.init_seconds * 1e6
                         else self.a.max_xy):
                action = "STOP"
                self._stop("truth_xy_exceeded", t_rx, dict(disp_xy_m=round(disp, 4)))

        self.rows.append([
            "%.6f" % ((t_rx - self.t_start_us) * 1e-6), t_rx,
            "%.4f" % x, "%.4f" % y, "%.4f" % z,
            "%.4f" % clr, "%.4f" % clr_net,
            "%.4f" % disp, "%.4f" % dz,
            ("%.4f" % speed) if math.isfinite(speed) else "nan",
            "%.1f" % (gap_us / 1000.0), self.lost_us, action])
        if len(self.rows) >= self._flush_every:
            self._flush()

    def on_clock(self, msg):
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.last_sim = sim
        if self._pause_watch_t0 is not None:
            self.clock_rows_after_pause += 1

    # ------------------------------------------------------------------ warn
    def _warn(self, t_rx, facts):
        self.warned = True
        self.t_warn_us = t_rx
        self.warn_event = dict(
            reason="truth_wall_clearance_warn",
            t_sample_mono_s=(t_rx - self.t_start_us) * 1e-6,
            thresholds=dict(warn_clearance_net_m=self.a.warn_clearance,
                            stop_clearance_net_m=self.a.stop_clearance,
                            rotor_envelope_m=self.a.envelope_m),
            **facts)
        self.warn_event["t_marker_write_mono_s"] = (self._write_warn() - self.t_start_us) * 1e-6
        self.get_logger().error(
            "[NP01_FIX16] WARNING wall_clearance_net=%.3f m < %.3f m; marker written; "
            "waiting for pilot ACK" % (facts["wall_clearance_net_m"], self.a.warn_clearance))
        self._flush()

    def _write_warn(self):
        t = _now_us()
        with open(self.a.warn_marker, "w", encoding="utf-8") as fh:
            json.dump(dict(reason="truth_wall_clearance_warn",
                           t_mono_us=t, event=self.warn_event), fh,
                      ensure_ascii=False)
        return t

    # ------------------------------------------------------------------ stop
    def _stop(self, reason, t_rx, facts):
        if self.stop_reason is not None:
            return
        t_dec = _now_us()
        self.stop_reason = reason
        self.t_stop_us = t_rx
        ev = dict(reason=reason,
                  t_sample_mono_s=(t_rx - self.t_start_us) * 1e-6,
                  t_decision_mono_s=(t_dec - self.t_start_us) * 1e-6,
                  thresholds=dict(warn_clearance_net_m=self.a.warn_clearance,
                                  stop_clearance_net_m=self.a.stop_clearance,
                                  rotor_envelope_m=self.a.envelope_m,
                                  max_xy_m=self.a.max_xy,
                                  max_xy_init_m=self.a.max_xy_init,
                                  init_seconds=self.a.init_seconds,
                                  max_z_m=self.a.max_z,
                                  max_rate_dps=self.a.max_rate_dps),
                  **facts)
        self.stop_event = ev
        # 1) physical stop request FIRST, before any file writing beyond the
        #    small marker below (evidence must never gate the stop).
        if self.a.enable_pause:
            self._request_pause(reason)
        # 2) evidence marker (audit / runner loop). Not the actuator.
        try:
            with open(self.a.stop_marker, "w", encoding="utf-8") as fh:
                json.dump(dict(reason=reason, t_mono_us=t_dec, event=ev), fh,
                          ensure_ascii=False)
            ev["t_marker_write_mono_s"] = (_now_us() - self.t_start_us) * 1e-6
        except OSError as exc:
            ev["marker_error"] = repr(exc)
        self.get_logger().error(
            "[NP01_FIX16] STOP (%s): wall_clearance_net=%.3f m; pause requested; "
            "the pilot LAND state machine is NOT waited for"
            % (reason, facts.get("wall_clearance_net_m", float("nan"))))
        self._flush()
        if not self.a.enable_pause:
            # nothing else to confirm; the report must exist immediately
            self._finalise()

    def _request_pause(self, reason):
        self.pause_attempts += 1
        cmd = ["gz", "service", "-s", "/world/%s/control" % self.a.world,
               "--reqtype", "gz.msgs.WorldControl",
               "--reptype", "gz.msgs.Boolean",
               "--timeout", "3000", "--req", "pause: true"]
        t0 = _now_us()
        try:
            self.pause_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                               stderr=subprocess.PIPE, text=True)
            self.pause_req = dict(attempt=self.pause_attempts, t_send_us=t0,
                                  t_send_mono_s=(t0 - self.t_start_us) * 1e-6,
                                  cmd=" ".join(cmd))
            self._pause_watch_t0 = t0
        except OSError as exc:
            self.pause_req = dict(attempt=self.pause_attempts, error=repr(exc))
            self._hard_stop("pause_spawn_failed")

    def _hard_stop(self, why):
        """Last-resort: stop the physics server by exact identity, never pkill."""
        killed = []
        for pid in os.listdir("/proc"):
            if not pid.isdigit() or int(pid) == os.getpid():
                continue
            try:
                with open("/proc/%s/cmdline" % pid, "rb") as fh:
                    raw = fh.read().replace(b"\0", b" ").decode("utf-8", "replace")
            except OSError:
                continue
            if self.a.world not in raw:
                continue
            # exact identity: the gz SIM SERVER binary of this world, never the
            # `gz service` CLI used above, never a wrapper or another world.
            if not ("gz-sim-main" in raw or "gz-sim-server" in raw):
                continue
            if " service " in raw or "/control" in raw or "--req" in raw:
                continue
            try:
                os.kill(int(pid), signal.SIGSTOP)
                killed.append(dict(pid=int(pid), cmdline=raw[:300], signal="SIGSTOP"))
            except OSError as exc:
                killed.append(dict(pid=int(pid), error=repr(exc)))
        self.hard_kill = dict(reason=why, targets=killed)
        self.get_logger().error("[NP01_FIX16] hard stop (%s): %s" % (why, killed))

    # ------------------------------------------------------------------ tick
    def tick(self):
        if self._exiting:
            return
        # pause reply + freeze confirmation
        p = self.pause_proc
        if (p is not None and self.pause_reply is None and p.poll() is None
                and (_now_us() - self.pause_req["t_send_us"]) > 3.5e6):
            self.pause_reply = dict(attempt=self.pause_attempts, timeout=True,
                                    latency_ms=3.5e3)
            self.stop_event["pause"] = self.pause_reply
            self.get_logger().error("[NP01_FIX16] pause request timed out; escalating")
            if self.a.hard_kill_on_fail:
                self._hard_stop("pause_timeout")
            self.pause_confirmed = False
            self._finalise()
        if p is not None and self.pause_reply is None and p.poll() is not None:
            t = _now_us()
            out = (p.stdout.read() if p.stdout else "") or ""
            err = (p.stderr.read() if p.stderr else "") or ""
            self.pause_reply = dict(
                latency_ms=(t - self.pause_req["t_send_us"]) / 1000.0,
                t_reply_mono_s=(t - self.t_start_us) * 1e-6,
                rc=p.returncode, stdout=out.strip()[:200], stderr=err.strip()[:200],
                ok=("true" in out))
            self._pause_watch_t0 = t
            self.pause_sim_at_reply = self.last_sim
        if (self.pause_reply is not None and self.pause_confirmed is None
                and self._pause_watch_t0 is not None):
            if (_now_us() - self._pause_watch_t0) > 600000:
                # source time must not advance while paused
                adv = 0.0
                if self.pause_sim_at_reply is not None and self.last_sim is not None:
                    adv = self.last_sim - self.pause_sim_at_reply
                self.sim_adv_after_pause = adv
                self.pause_confirmed = (adv < self.a.pause_max_slip_s)
                self.stop_event["pause"] = self.pause_reply
                self.stop_event["pause_sim_advance_s"] = round(adv, 4)
                self.stop_event["pause_confirmed"] = bool(self.pause_confirmed)
                self.stop_event["t_pause_confirmed_mono_s"] = (
                    (_now_us() - self.t_start_us) * 1e-6)
                if not self.pause_confirmed:
                    self.get_logger().error(
                        "[NP01_FIX16] pause NOT confirmed (source advanced %.3f s); "
                        "escalating" % adv)
                    if self.a.hard_kill_on_fail:
                        self._hard_stop("pause_unconfirmed")
                else:
                    self.get_logger().error(
                        "[NP01_FIX16] pause confirmed: source time stopped "
                        "(advance %.4f s after reply)" % adv)
                self._finalise()

        # warning ACK contract
        if self.warned and self.warn_ack is None and os.path.exists(self.a.ack_marker):
            try:
                with open(self.a.ack_marker, encoding="utf-8") as fh:
                    ack = json.load(fh)
                self.warn_ack = ack
                t_ack = float(ack.get("ack_mono_us") or 0)
                self.warn_event["ack"] = ack
                self.warn_event["ack_latency_ms"] = (
                    (t_ack - self.t_warn_us) / 1000.0 if t_ack else None)
                self.get_logger().error(
                    "[NP01_FIX16] warning ACKed by pilot: %.1f ms, state=%s, action=%s"
                    % (self.warn_event["ack_latency_ms"] or -1.0,
                       ack.get("state"), ack.get("action")))
                self._flush()
            except (OSError, ValueError, TypeError) as exc:
                self.warn_event["ack_error"] = repr(exc)
        if (self.warned and self.warn_ack is None and self.stop_reason is None
                and self.t_warn_us is not None
                and (_now_us() - self.t_warn_us) > self.a.ack_timeout_s * 1e6):
            self._stop("truth_warn_unacked", _now_us(), dict(
                note="warning was not acknowledged within %.2f s" % self.a.ack_timeout_s))

        # input loss while the monitor is the only safety net
        if (self.last_rx_us is not None and self.stop_reason is None
                and (_now_us() - self.last_rx_us) > self.a.input_timeout_s * 1e6):
            self._stop("truth_input_lost", _now_us(), dict(
                gap_ms=(_now_us() - self.last_rx_us) / 1000.0))

        # fail-closed startup: a monitor that never receives a pose must not
        # silently look healthy. The runner aborts the run on this marker.
        if (self.n == 0
                and (_now_us() - self.t_start_us) > self.a.startup_timeout_s * 1e6):
            self.get_logger().error(
                "[NP01_FIX16] no pose on %s after %.1f s; the run must not "
                "proceed without independent supervision"
                % (self.a.pose_topic, self.a.startup_timeout_s))
            try:
                with open(os.path.join(self.a.run_dir, "SUPERVISOR_NO_POSE"),
                          "w", encoding="utf-8") as fh:
                    fh.write(self.a.pose_topic + "\n")
            except OSError:
                pass
            self._finalise(exiting=True)

        # idle exit: the runner is gone / run finished
        if (self.stop_reason is None and self.n > 0 and self.last_rx_us
                and (_now_us() - self.last_rx_us) > 30e6):
            self._finalise(exiting=True)

    def _flush(self):
        if self.rows:
            self._wr.writerows(self.rows)
            self.rows = []
        try:
            self._csv.flush()
        except (OSError, ValueError):
            pass

    def _finalise(self, exiting=False):
        if self._written:
            # [NP01-FIX16] already reported: still honour an exit request,
            # otherwise SIGTERM would leave this process spinning forever
            # after the runner's cleanup (observed after round 15).
            if exiting and not self._exiting:
                self._exiting = True
                import threading
                threading.Timer(0.05, self._shutdown).start()
            return
        self._written = True
        self._flush()
        res = dict(
            scope=("independent truth supervision: direct pose subscription, "
                   "monotonic receive time, two-tier warn/stop, physical pause on stop"),
            not_a_control_input=True,
            csv_used_for_safety=False,
            rows_seen=self.n,
            datum=dict(t_mono_us=self.datum[0], x=self.datum[1], y=self.datum[2],
                       z=self.datum[3]) if self.datum else None,
            first_pose_mono_s=(self.datum[0] - self.t_start_us) * 1e-6 if self.datum else None,
            max_rx_gap_ms=self.max_gap_ms,
            lost_us_total=self.lost_us,
            min_clearance=dict(wall_clearance_m=self.min_clearance_net_event["wall_clearance_m"],
                               wall_clearance_net_m=self.min_clearance_net,
                               at=self.min_clearance_net_event) if self.min_clearance_net_event else None,
            warned=self.warned,
            warn_event=self.warn_event,
            triggered=self.stop_reason is not None,
            stop_event=self.stop_event,
            stop_reason=self.stop_reason,
            pause=self.pause_reply,
            pause_confirmed=self.pause_confirmed,
            pause_sim_advance_s=self.sim_adv_after_pause,
            hard_kill=self.hard_kill,
            config=dict(pose_topic=self.a.pose_topic, clock_topic=self.a.clock_topic,
                        world=self.a.world, walls=dict(west=self.a.west, east=self.a.east,
                                                       south=self.a.south, north=self.a.north),
                        rotor_envelope_m=self.a.envelope_m,
                        warn_clearance_net_m=self.a.warn_clearance,
                        stop_clearance_net_m=self.a.stop_clearance,
                        max_xy_m=self.a.max_xy, max_xy_init_m=self.a.max_xy_init,
                        init_seconds=self.a.init_seconds, max_z_m=self.a.max_z,
                        max_rate_dps=self.a.max_rate_dps,
                        ack_timeout_s=self.a.ack_timeout_s,
                        input_timeout_s=self.a.input_timeout_s,
                        geometry_used=self.a.geometry_used))
        tmp = self.a.json_out + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(res, fh, indent=2, ensure_ascii=False)
        os.replace(tmp, self.a.json_out)
        print("NP01_SUPERVISOR warned=%s triggered=%s rows=%d reason=%s pause=%s"
              % (self.warned, self.stop_reason is not None, self.n,
                 self.stop_reason, self.pause_confirmed), flush=True)
        if exiting and not self._exiting:
            self._exiting = True
            # Shutting rclpy down from inside a callback can deadlock; do it
            # from a helper thread after returning to the executor.
            import threading
            threading.Timer(0.05, self._shutdown).start()

    def _shutdown(self):
        try:
            rclpy.shutdown()
        except Exception:
            pass
        time.sleep(0.05)
        # the report is already on disk; do not linger as an orphan monitor
        os._exit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--world", default=os.environ.get("NP01_WORLD", "rm27_world"))
    ap.add_argument("--pose-topic", default="")
    ap.add_argument("--clock-topic", default="/clock")
    ap.add_argument("--warn-marker", default="")
    ap.add_argument("--stop-marker", default="")
    ap.add_argument("--ack-marker", default="")
    ap.add_argument("--json-out", default="")
    ap.add_argument("--flush-rows", type=int, default=100)
    ap.add_argument("--geometry-json", default="",
                    help="[NP01-FIX17] geometry generated with the world "
                         "(walls/spawn/envelope). When given it OVERRIDES --walls "
                         "so an old wall coordinate cannot be reused by accident.")
    ap.add_argument("--walls", default="west=0.349,east=27.949,south=0.264,north=15.867")
    ap.add_argument("--envelope-m", type=float, default=0.385971)
    ap.add_argument("--warn-clearance", type=float, default=2.0,
                    help="WARN when wall_clearance_net_m < this (m)")
    ap.add_argument("--stop-clearance", type=float, default=1.2,
                    help="STOP when wall_clearance_net_m < this (m)")
    ap.add_argument("--max-xy", type=float, default=6.5)
    ap.add_argument("--max-xy-init", type=float, default=6.5)
    ap.add_argument("--init-seconds", type=float, default=90.0)
    ap.add_argument("--max-z", type=float, default=4.0)
    ap.add_argument("--max-rate-dps", type=float, default=600.0)
    ap.add_argument("--rate-samples", type=int, default=3)
    ap.add_argument("--loss-gap-us", type=float, default=200000.0)
    ap.add_argument("--input-timeout-s", type=float, default=1.5)
    ap.add_argument("--startup-timeout-s", type=float, default=6.0,
                    help="fail closed if no pose arrives within this window")
    ap.add_argument("--ack-timeout-s", type=float, default=2.0)
    ap.add_argument("--enable-pause", type=int, default=1)
    ap.add_argument("--pause-max-slip-s", type=float, default=0.10)
    ap.add_argument("--hard-kill-on-fail", type=int, default=1)
    a = ap.parse_args()
    a.pose_topic = a.pose_topic or ("/world/%s/dynamic_pose/info" % a.world)
    a.warn_marker = a.warn_marker or os.path.join(a.run_dir, "TRUTH_WARN")
    a.stop_marker = a.stop_marker or os.path.join(a.run_dir, "TRUTH_BOUND_EXCEEDED")
    a.ack_marker = a.ack_marker or os.path.join(a.run_dir, "TRUTH_WARN_ACK")
    a.json_out = a.json_out or os.path.join(a.run_dir, "supervisor.json")
    a.enable_pause = bool(a.enable_pause)
    a.hard_kill_on_fail = bool(a.hard_kill_on_fail)
    walls = {}
    if a.geometry_json:
        with open(a.geometry_json, encoding="utf-8") as fh:
            geo = json.load(fh)
        if geo.get("world") != a.world:
            raise SystemExit("geometry world %r != --world %r"
                             % (geo.get("world"), a.world))
        walls = {k: float(v) for k, v in geo["walls_inner_faces"].items()}
        a.envelope_m = float(geo.get("rotor_envelope_m", a.envelope_m))
        a.spawn = geo.get("spawn")
        a.geometry_used = dict(path=a.geometry_json,
                               world=geo["world"], walls=walls,
                               envelope_m=a.envelope_m,
                               spawn=a.spawn,
                               world_md5=(geo.get("hashes") or {}).get("world"))
    else:
        a.spawn = None
        a.geometry_used = None
        for item in a.walls.split(","):
            k, v = item.split("=", 1)
            walls[k.strip()] = float(v)
    for k in ("west", "east", "south", "north"):
        if k not in walls:
            raise SystemExit("walls missing %s" % k)
        setattr(a, k, walls[k])

    rclpy.init()
    node = TruthSupervisor(a)

    def _term(_s, _f):
        try:
            node._finalise(exiting=True)
        except SystemExit:
            pass
    signal.signal(signal.SIGTERM, _term)
    signal.signal(signal.SIGINT, _term)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node._finalise()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
