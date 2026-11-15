# SITL findings: publication leases and pilot feedback

User supplied logs/bt_sitl_20261108. Scope: fix normal PX4 message timing and completion feedback in the existing manually launched navigation task. Autonomous takeoff/VIO bootstrap is not established by this GPS + health-stub test and is outside this regression fix.

Evidence: rc_update republishes unchanged input after 300ms; ManualControl republishes timestamp with the selected timestamp_sample unchanged. Status at 2Hz, land at 1Hz make the prior 0.5s leases insufficient. Preserve baseline in logs/bt_fix_20261108/baseline.

Chosen design: separate publication and sample clocks; permit repeated sample timestamps within an independent sample-age bound. Exact publication duplicates do not renew leases; backwards/conflicting timestamps invalidate. Both source age and monotonic receipt age count toward deadlines. Fixed bounded jitter margins cover normal cadence; do not add an unbounded retry or extra grace beyond these deadlines. Reject confirmed faults and takeover immediately.

Alternatives: increasing all leases to 1.5–3s hides loss of fast position/health streams; ignore sample age indefinitely would permit frozen RC input. Neither is selected.

Steps:
1. Add failing timing tests for measured cadence, constant sticks, source-age + receipt-age budget, frozen samples, duplicate/reversed publication, fault/takeover without new samples.
2. Update PilotAuthority dual timestamp ingestion, bounded TTLs and diagnostic ages. Preserve reference identity and physical RC requirements.
3. Make ROS callbacks pass timestamp + timestamp_sample independently; add reliable status topic with readiness, prestream duration, task completion and revocation. Hold keeps controlling after completion; never terminate the hold by silently cutting off valid Offboard.
4. Require >1s continuous eligible heartbeat before acquisition; advertise ready and require physical selector cycle if selected too soon. Maintain heartbeat through the normal selector-to-commander transition after readiness, with bounded wait.
5. Test the node using actual 2/5/1Hz cadence and repeated samples, mixed with immediate override/invalid/frozen inputs; extend flight geometry and completion tests.
6. Run a new PX4/Gazebo SITL task with constant RC (no artificial jitter), original evidence preserved, explicit simulation-only health stub. Record navigation, completion, selector takeover and shutdown. If environment prevents it, report the exact gap.
7. Document new deadlines, measured results, and limits: GPS SITL does not verify ORB-SLAM3 initialization, relocation accuracy, or physical RC latency.
