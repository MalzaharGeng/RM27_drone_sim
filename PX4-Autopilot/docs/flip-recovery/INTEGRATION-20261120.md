# 2026-11-20 local software integration

This document supersedes the older statements that automatic DShot direction switching is unimplemented. Those documents remain historical background. No ground-dynamics simulation or hardware test has been performed for this update.

## Production changes

- `DirectionSequencer.hpp` is the shared, platform-independent stop/normal/reverse/wait/drive/restore state machine. Every direction change first holds all outputs at STOP for 300 ms. Commands 20/21 are submitted for ten complete frames per group; throttle resumes after a 100 ms guard and a fresh output slew ramp. These are explicit timing assumptions, not measured guarantees that a loaded rotor has stopped.
- DShot resolves logical Motor1..4 through output-function assignments, rather than assuming hardware outputs 1..4. All four must exist uniquely in the same DShot instance. Mapping changes during recovery cancel drive and restore using the saved mapping.
- STOP, a stale/future request, invalid controls, session changes, mode exit and safety loss stop drive. Restoration owns the output path until normal-direction commands and their guard finish. While armed/kill/lockdown/termination/calibration/3D-configured, the owned path emits STOP only and defers direction commands. An unsafe state must be cleared while disarmed to finish restoration. Driver shutdown requests restoration before normal cleanup; power loss and forced process termination cannot restore ESC state.
- Recovery does not use the normal CLI/configuration command slot. Those commands wait while recovery owns the bus. Command repetitions advance only after all requested logical motors have had a frame submitted and the DShot trigger has been called. There is no ESC direction acknowledgement.
- Recovery throttle is mapped from unipolar `[0,1]` to the bipolar input used by the output limiter. The old implementation interpreted 0.20 as approximately 60% of the configured min/max interval. The new implementation means 20% of that interval (including configured DSHOT_MIN). Zero remains the disarmed STOP value. Previously chosen throttle numbers therefore are NOT calibrated equivalents of the old output.
- Planner height now negates NED down coordinates. Its default quaternion is wxyz identity. Directed progress uses an unwrapped atan2 angle in the initial recovery plane; the expected direction is frozen at session start, so crossing 180 degrees does not turn valid progress negative. Off-plane motion, 180-degree ties, dynamically changing motor groups and actual contact mechanics still require simulation validation.
- Crossing FLIP_STOP_ANG cuts throttle but does not report success. Success requires estimated tilt <=15 degrees and total angular rate <=10 degrees/s continuously for one second; failure reason 6 means no settling within three seconds after cut. Mode exit during settling is failure, not success. This estimated-state test never substitutes for independent physics truth scoring.
- Invalid attitude/geometry/gyro, safety loss and mode exit retain terminal failure reasons. A new session clears old failures. The module validates all exposed settings, observes parameter updates without the old one-second interval, and retries a rejected exit-mode request once per second while remaining stopped.
- SITL default builds and starts the module, dormant with FLIP_EN=0. Both flip topics are included in optional ULog subscriptions and exported through uXRCE-DDS. ROS2 clients must regenerate matching px4_msgs, including the added control.slew_rate field and REASON_NOT_SETTLED value.

## Important simulation boundary

Enabling the module is not a complete ground-physics backend. Standard Gazebo ESC/PWM simulation does not consume the DShot recovery sequencer, and may discard signed motor speed or disarmed outputs. The next verifier must create an explicit simulation adapter with the same `DirectionSequencer` semantics, real sensor/estimator inputs, actuator gates, and a motor model that actually reverses thrust. It must record that this adapter substitutes for physical DShot, and separately test the actual DShot code. Do not set FLIP_OUT_DIRECT=1 and assume it proves the hardware output path or timing.

## Reproduction

Host regressions (no simulator):

```bash
bash test/flip_recovery/run_host_tests.sh /tmp/px4_flip_host
```

Build only (does not launch PX4 or Gazebo):

```bash
git submodule update --init --recursive
make px4_sitl_default -j4
make micoair_h743-v2_default -j4
```

For a partial Ubuntu ARM toolchain without libstdc++ linker archives, the recorded build uses `CMAKE_ARGS=-DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY` to avoid CMake's standalone C++ executable probe; the actual NuttX firmware still must compile and link. This is a build-environment option, not a controller change.

For Gazebo from ROS Lyrical vendors, source `/opt/ros/lyrical/setup.bash` before CMake configuration. If a previous configure disabled Gazebo for missing dependencies, rerun CMake explicitly; merely sourcing the environment after configuration does not re-enable the missing targets.

For unarmed recovery logging the verifier must enable boot-time ULog logging (e.g. the applicable SDLOG_MODE setting) and verify that both flip topics actually occur in the ULog. Logger topic registration alone does not prove that logging was active.

## Untested and not promised

No physical success rate, tuned parameter set, Bluejay response under propeller load, loaded motor stopping time, contact/friction fidelity, ROS2 live transport or full mode-to-physics-to-exit acceptance has been established by this integration. Use the delivery verification logs to distinguish executed host/build checks from future work. Do not flash or touch hardware as part of the independent simulation handoff.
