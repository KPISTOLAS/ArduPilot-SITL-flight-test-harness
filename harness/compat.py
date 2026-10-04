"""Compatibility shims loaded before DroneKit / pymavlink use."""

from __future__ import annotations

import collections
import collections.abc


def patch_collections_abc() -> None:
    """DroneKit 2.9 still references collections.MutableMapping (removed in 3.10+)."""
    if not hasattr(collections, "MutableMapping"):
        collections.MutableMapping = collections.abc.MutableMapping  # type: ignore[attr-defined]


def patch_pymavlink_set_mode_apm() -> None:
    """
    dronekit-sitl ships Copter 3.3, which does NOT support MAV_CMD_DO_SET_MODE.

    pymavlink >= 2.4.9 switched set_mode_apm() to command_long DO_SET_MODE, so
    mode changes silently fail and the vehicle stays in STABILIZE.
    Restore the classic SET_MODE message used by older pymavlink.
    """
    try:
        from pymavlink import mavutil
    except ImportError:
        return

    def set_mode_apm_legacy(self, mode):
        if isinstance(mode, str):
            mode_map = self.mode_mapping()
            if mode_map is None or mode not in mode_map:
                print("Unknown mode '%s'" % mode)
                return
            mode = mode_map[mode]
        self.mav.set_mode_send(
            self.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode,
        )

    # Patch the base mavfile class used by TCP/UDP connections.
    mavutil.mavfile.set_mode_apm = set_mode_apm_legacy  # type: ignore[assignment]


patch_collections_abc()
patch_pymavlink_set_mode_apm()
