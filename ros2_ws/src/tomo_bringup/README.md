# tomo_bringup

Version: 1.3 (final for phase 1)

Launch files for full or partial system bringup.

## Launch
- `bringup.launch.py` - all-in-one (PS4 + Web + Factory + Motion + turtlesim)
- `tomo_system.launch.py` - control, motion, PS4, web, engine + steering managers
- `turtlesim.launch.py` - standalone turtlesim for testing

## Notes
- `bringup.launch.py` starts `joy_node` from the `joy` package.
- `tomo_system.launch.py` includes launch files from other packages.
