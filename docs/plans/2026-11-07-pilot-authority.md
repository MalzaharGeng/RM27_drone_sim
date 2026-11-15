# Pilot authority and ten-metre navigation implementation plan

**Goal:** Pilot flies manually, validates VIO in Position, selects Offboard to authorize one task, and can cancel it by leaving Offboard.

**Architecture:** Preserve the localization-only C++ bridge in ORB_SLAM3. Replace the Python square BT's automatic arm/mode/land path in drone_sim with a pilot-authorized task and a ROS-independent authority gate. Gate both task ticks and final publication. Bind authorization to fresh RC switches, RC-source manual input, actual/user-intended Offboard mode, fresh VIO health and the complete reference token. Revoke on takeover, stale telemetry, fault, estimator reset or reference change.

**Tech Stack:** Python 3, ROS 2 Lyrical, px4_msgs (VehicleStatus v1), Eigen/OpenCV C++17, CTest/unittest.

## Design choices

1. Checking actual Offboard alone is insufficient: GCS/another companion could select it. Also require RC-source manual input and a mapped physical Offboard switch or explicitly configured mode slot. This is the selected approach.
2. Relying on `nav_state_user_intention` alone is compatible with stock DDS but cannot prove the physical selector. Rejected for this requirement.
3. Moving all autonomy into PX4 would need a separate firmware task; keep existing PX4 failsafes and remove companion arm/mode/land commands from the real-flight BT.

User confirmed: manual takeoff, Position check, then Offboard; default trial is ten metres forward from acquisition heading and hold. No obstacle avoidance is claimed for this trial. A* planner remains separate.

## Implementation steps

1. Preserve existing dirty changes and baseline copies outside both projects (flight_control_review_20261107).
2. Add drone_sim/scripts/pilot_authority.py with freshness, physical selector, reference binding, takeover latch and local discontinuity checks; test boundary scenarios without ROS.
3. Replace drone_sim/scripts/px4_square_bt.py with guarded Sequence (capture current anchor/yaw -> line or square -> hold). Pre-entry sends OffboardControlMode proof-of-life only, never trajectory, arm or mode commands. Require seeing a non-Offboard/unselected state before first acquisition. After cancellation require restarting the task and a new selector transition. Mission completion holds until pilot takeover.
4. Block rm27_nav.py and rm27_waypoints.py MAVSDK auto-flight entries by default; retain an explicitly named simulation-only opt-in. The former changes PX4 parameters, and both send automatic flight commands. Do not route physical flights through them.
5. Add C++ angular discontinuity gates in Px4Vio.h/Px4Vio.cc and explicit YAML defaults; retain position/velocity and epoch latches. Angular frame faults require disarm. Extend px4_vio_regression.
6. Add an accuracy report tool using independently timestamped VIO/truth/target samples; report drift, endpoint error, dropouts and jumps without fitting away bias or claiming truth from the controller's own estimate.
7. Document physical switch DDS publication requirements, RC slot mapping, Offboard pre-entry semantics, takeover behavior, relocation boundaries and staged ten-metre measurements.
8. Build targeted C++ targets; run existing VIO/recording/sortie tests and isolated ROS integration; run authority/BT/accuracy/planner tests. Record commands and limitations. No physical aircraft commands, firmware flash or GitHub push.

## Acceptance

- No arm/disarm, mode, takeoff or land command publisher exists in the real-flight BT.
- No trajectory outside confirmed physically authorized Offboard; takeovers bypass fallback actions and clear targets.
- Position retains independent VIO publication; localization validity never grants actuation authority.
- No stale/duplicate message can renew a lease. Frame/relocalization changes invalidate the task even with matching numeric coordinates.
- A ten-metre flight's absolute accuracy is reported only against independent measurements. ROS mock tests establish software behavior, not physical flight performance.
