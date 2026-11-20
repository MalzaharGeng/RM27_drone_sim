# Flip recovery integration implementation plan

**Goal:** Complete software direction sequencing and correct confirmed planner defects before independent simulation validation.

**Architecture:** A platform-independent direction sequencer owns stop/command/wait/drive/restore transitions. DShot maps logical motor IDs to hardware outputs and acknowledges transmitted bursts. The planner uses NED-consistent height, strategy-aware progress and a real settling interval. SITL enables the module but never pretends an ordinary single-direction motor model implements reversal.

**Tech stack:** PX4 v1.17 C++, DShot 20/21, uORB, standalone host regression tests, Linux compiler and PX4 build checks.

1. Freeze upstream 60889745af001515a62872420de2d32fb602803b in an isolated codex branch. Inspect applicable instructions and build dependencies.
2. Add regression cases against actual C++ planner for NED grouping, over-180 progress, settling, invalid attitude and exit reason; establish failures before fixes.
3. Implement direction sequencer and regression cases: initial normal/reverse bursts, motor mapping, no drive during switching, STOP, stale request, interruption and restoration. DShot integration must preserve safety gates and block normal throttle until restoration completes.
4. Correct planner defects and module validation/result/exit handling without changing independent physical acceptance thresholds or claiming tuned parameters.
5. Enable module build/start for SITL; document simulation adapter boundary and direction-delay assumptions. Compile modified production code where toolchain is available. Do not start simulator or touch hardware.
6. Deliver source/patches, verification logs, exact manifests, concise change report and full independent-validation prompt with absolute delivery paths. Preserve any unverified build/hardware/model limitations.
