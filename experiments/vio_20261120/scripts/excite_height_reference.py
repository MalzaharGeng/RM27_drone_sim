#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[NP-01 FIX20] EXCITE 高度参考在 PX4 局部 NED z 坐标系 reset 后的连续性契约。

问题（FIX19 离线定位）
----------------------
`sim_pilot.py:682-683` 进入 EXCITE 时把 `excite_ref[2] = height.z` 捕获一次；
`:708-711` 用 `abs(height.z - excite_ref[2]) > excite_max_dz` 判越界。
PX4 EKF 首次融合视觉高度时会做一次 **position reset**：`z_reset_counter`
1→2、`delta_z = ±…`，`z` 瞬时阶跃同一个量。旧参考属于旧坐标系，直接相减
就把「坐标系平移」误判成「飞机爬升」并 fatal（flight2 的 1.1929 m）。

PX4 消息语义（已用 ULog + .msg 核实）
------------------------------------
`VehicleLocalPosition.msg`：
  * `z` —— NED Down position（负高度），reset 后 `z_after = z_before + delta_z + 真实运动`
  * `delta_z` —— "Amount of vertical shift of position estimate in latest reset [m]"，
    **sticky**：reset 后每一帧都保持同一个值，直到下一次 reset
  * `z_reset_counter` —— uint8，每次 z reset +1，**回绕 255→0 合法**
  * `vz_reset_counter` —— uint8，速度 reset 计数，独立记录

因此契约是：
  1. **符号**：`ref_z += delta_z`（同号平移，不是 `ref_z -= delta_z`，
     更不是 `ref_z = 当前 z`）。后者会清掉真实累计高度偏移，属禁止行为。
  2. **一次性**：只在 `z_reset_counter` 出现一个**可确认的新事件**时应用一次。
     sticky `delta_z` 在后续帧不得重复平移。
  3. **可确认的新事件**：`(zrc_new - zrc_old) % 256 == 1`。
     `0` = 同一 counter 重复消息；`2..255` = 跨过多个事件 / 回退 / 乱序，
     累计变换无法可靠恢复 → **失败关闭**，明确报错，不猜测、不拿最后一个
     delta 代表全部 reset。
  4. **首帧同时捕获 counter**：进入 EXCITE 捕获参考时必须同时记录
     `zrc/vzrc/sample_us`，避免把进入前已存在的历史 sticky delta 误用。
  5. **per-tick 影响面**：reset 当拍只「暂缓高度位移这一项判定」；
     输入健康、角速度、倾角、水平位移、VIO 就绪、监督器、授权、
     硬上限等所有其它退出条件继续生效。下一有效样本恢复正常高度包络检查。
  6. **不重置任何状态**：不延长初始化时间、不清除安全故障、不重新开始包络、
     不重置 `disp_xy` 参考、不重置 ready streak。

本模块是纯函数（不依赖 rclpy / 不读真值 / 不读时钟），便于单测；
`sim_pilot.py` 的 EXCITE 生产路径调用 `advance_excite_height_reference()`。
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ExciteHeightRef:
    """EXCITE 高度参考 + 捕获它的坐标系身份。"""
    z: float
    zrc: int
    vzrc: int
    sample_us: int


@dataclass(frozen=True)
class ExciteResetDecision:
    """一次判定。

    action:
      "none"      —— 无新 reset，正常判定（高度包络照常生效）
      "translate" —— 单个可确认新 reset，参考已平移；本拍暂缓高度位移判定
      "fail"      —— 无法可靠恢复累计变换 / 输入不可信；调用方必须失败关闭
    """
    action: str
    ref: ExciteHeightRef
    reason: str
    applied_delta_z: float = 0.0
    old_ref_z: float = 0.0
    zrc_before: int = -1
    zrc_after: int = -1
    vzrc_before: int = -1
    vzrc_after: int = -1
    suspend_dz_this_tick: bool = False


def _finite(*values):
    for v in values:
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            return False
        if not math.isfinite(float(v)):
            return False
    return True


def _valid_counter(v):
    return (not isinstance(v, bool) and isinstance(v, int)
            and 0 <= v <= 255)


def capture_excite_height_ref(s):
    """进入 EXCITE 时捕获参考：z 与 counter/sample_us 必须同时捕获。

    s: HeightSample（或任何提供同名属性的对象）。
    返回 (ExciteHeightRef | None, reason)。
    """
    if s is None:
        return None, "missing_sample"
    if not _finite(s.z):
        return None, "non_finite_z"
    if not _valid_counter(s.z_reset_counter):
        return None, "invalid_z_reset_counter"
    if not _valid_counter(s.vz_reset_counter):
        return None, "invalid_vz_reset_counter"
    if not _finite(s.sample_us):
        return None, "non_finite_sample_us"
    return ExciteHeightRef(float(s.z), int(s.z_reset_counter),
                           int(s.vz_reset_counter), int(s.sample_us)), "captured"


def advance_excite_height_reference(s, ref, now_mono, max_age_s,
                                    max_abs_delta_z_m, last_sample_us):
    """一拍推进。绝不把参考重设为当前 z。

    s: 当前 HeightSample；ref: 已捕获的 ExciteHeightRef；
    now_mono: 主机单调时刻；max_age_s: 反馈新鲜度上限；
    max_abs_delta_z_m: 单次 reset delta 的**输入健全性界**（沿用 takeoff
                       契约 `DEFAULT_CONFIG.max_abs_delta_z_m`；不是控制门限、
                       不是包络、不是验收标准）；
    last_sample_us: 上一拍见到的源样本时间，用于检测乱序（由调用方逐帧维护）。

    返回 ExciteResetDecision；调用方必须把 action=="fail" 当作失败关闭。
    """
    if ref is None:
        return ExciteResetDecision("fail", ref, "reference_not_captured")

    # --- (1) 输入健全性：任何不可信输入都不能静默放行 ---
    if s is None:
        return ExciteResetDecision("fail", ref, "missing_sample")
    if not _finite(s.z, s.delta_z, s.sample_us, s.rx_mono, now_mono,
                   ref.z, ref.sample_us):
        return ExciteResetDecision("fail", ref, "non_finite_input")
    if not _valid_counter(s.z_reset_counter):
        return ExciteResetDecision("fail", ref, "invalid_z_reset_counter")
    if not _valid_counter(s.vz_reset_counter):
        return ExciteResetDecision("fail", ref, "invalid_vz_reset_counter")
    if not s.z_valid:
        return ExciteResetDecision("fail", ref, "z_invalid")
    age = now_mono - s.rx_mono
    # 允许 executor 在判定当拍刚更新样本造成的小负值，但拒绝明显时钟异常。
    if not math.isfinite(age) or age > max_age_s or age < -max_age_s:
        return ExciteResetDecision("fail", ref, "stale_or_clock_anomaly")
    if last_sample_us is not None and s.sample_us < last_sample_us:
        return ExciteResetDecision("fail", ref, "out_of_order_sample")

    zrc = int(s.z_reset_counter)
    vzrc = int(s.vz_reset_counter)
    step = (zrc - ref.zrc) % 256

    # --- (2) 同一 counter 反复到达：sticky delta 绝不重复应用 ---
    if step == 0:
        return ExciteResetDecision("none", ref, "no_new_reset")

    # --- (3) 跨过多个事件 / 回退 / 乱序：累计变换不可恢复，失败关闭 ---
    if step != 1:
        return ExciteResetDecision(
            "fail", ref, "unrecoverable_reset_step_%d" % step,
            zrc_before=ref.zrc, zrc_after=zrc,
            vzrc_before=ref.vzrc, vzrc_after=vzrc)

    # --- (4) 恰好一个新事件：delta 必须有限且非零、在健全性界内 ---
    # counter +1 但 delta_z=0 属「可确认事件却无变换量」：无法排除数据缺失，
    # 失败关闭，不静默放行、不猜测为零平移。
    dz = float(s.delta_z)
    if not math.isfinite(dz):
        return ExciteResetDecision("fail", ref, "non_finite_reset_delta",
                                   zrc_before=ref.zrc, zrc_after=zrc)
    if dz == 0.0:
        return ExciteResetDecision("fail", ref, "missing_reset_delta",
                                   zrc_before=ref.zrc, zrc_after=zrc)
    if abs(dz) > max_abs_delta_z_m:
        return ExciteResetDecision(
            "fail", ref, "reset_delta_out_of_bounds",
            zrc_before=ref.zrc, zrc_after=zrc)

    # --- (5) 同向平移一次，保留真实累计高度偏移 ---
    new_ref = ExciteHeightRef(ref.z + dz, zrc, vzrc, int(s.sample_us))
    return ExciteResetDecision(
        "translate", new_ref, "height_reference_translated",
        applied_delta_z=dz, old_ref_z=ref.z,
        zrc_before=ref.zrc, zrc_after=zrc,
        vzrc_before=ref.vzrc, vzrc_after=vzrc,
        suspend_dz_this_tick=True)
