#!/usr/bin/env python3
"""Compatibility entrypoint: pilot-authorized navigation (no auto arm/mode/land)."""
from pilot_navigation_bt import main

if __name__ == "__main__":
    raise SystemExit(main())
