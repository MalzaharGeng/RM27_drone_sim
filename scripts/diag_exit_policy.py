#!/usr/bin/env python3
"""[DIAG-ONLY] Bounded diagnostic exit policy for the SITL virtual pilot.

The diagnostic collection window and process termination are deliberately
SEPARATE concerns. When the window elapses the pilot must ENTER the existing
LAND state machine (not be killed by the runner), then wait for landed and
disarm before the simulation is torn down. This works even when VIO never
becomes ready and OFFBOARD was never reached.

No ROS / MAVLink dependency: intentionally isolated so the exit contract can
be unit-tested without a running simulation.
"""

# States that are already in an exit or terminal phase. The diagnostic cutoff
# must never re-trigger or interrupt them.
EXIT_STATES = frozenset(("FAULT", "LAND", "LANDED", "DONE"))


def diagnostic_exit_due(state, elapsed_s, cutoff_s):
    """Return True iff a bounded diagnostic window has elapsed and the pilot
    is not already in an exit/terminal state.

    ``cutoff_s <= 0`` (or None) DISABLES the feature completely, so the normal
    pilot state machine is unaffected. The function deliberately has no
    ``vio_ready`` / ``armed`` / ``offboard`` parameter: reaching the diagnostic
    cutoff must not depend on VIO recovery or on having reached OFFBOARD.
    """
    if cutoff_s is None or cutoff_s <= 0:
        return False
    return elapsed_s >= cutoff_s and state not in EXIT_STATES
