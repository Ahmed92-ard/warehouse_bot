# MARS Warehouse Rover

ROS 2 Jazzy workspace project containing the rover base, SO-101 follower arm, and SO-101 leader GUI packages.

## Packages

- `warehouse_bot/` — rover bringup, Nav2/SLAM configuration, maps, teleop, and warehouse task bridge
- `so101_lerobot_follower/` — follower arm driver, camera/audio streaming, and messages
- `so101_lerobot_leader/` — leader arm driver, GUI, camera viewer, and audio controls
- `STARTUP_COMMANDS.txt` — launch commands only

The warehouse backend's `.env` file contains local credentials and is intentionally excluded from Git. Copy `.env.example` to `.env` and fill in local values when setting up another machine.

Build from the ROS workspace root with `colcon build` so the workspace dependencies outside this project are discovered too. Source `install/setup.bash` (or the shell-specific equivalent) before launching.
