# AGENTS.md

ROS 2 (Jazzy) workspace for a mecanum-wheeled robot. Only two packages are tracked in this repo, under `src/mecanum/`; the rest of the workspace is imported externally.

## Partial checkout: external repos are required

- `src/.repos` is a `vcs` manifest. It pulls `mecanum_jacobian_controller`, `mecanum_hardware_interface`, `mecanum_microros_firmware`, `mecanum_udp_firmware`, `kinect_ros2`, `micro_ros_setup`, and `sllidar_ros2` into `src/control/`, `src/firmware/`, `src/third_party/`.
- Those directories are gitignored and **not present in this checkout**. Run `vcs import < src/.repos` from `src/` before building the full system. Launch files reference packages from these repos (`micro_ros_agent`, `sllidar_node`, `kinect_ros2_node`, `mecanum_hardware_interface`, `mecanum_jacobian`).
- The two tracked packages contain only assets (launch/xacro/mesh/config), no compiled code, so `colcon build --packages-select mecanum_description mecanum_bringup` works standalone (verified).

## Build & run

Jazzy is installed at `/opt/ros/jazzy`; source `/opt/ros/jazzy/setup.bash` first. Workspace root is the repo root.

```bash
cd src && vcs import < .repos && cd ..
rosdep update
rosdep install -i --from-paths src/ -y
colcon build
source install/setup.bash
ros2 launch mecanum_bringup system_bringup.launch.py   # main entrypoint (also Docker CMD)
```

`system_bringup.launch.py` accepts boolean args: `use_kinect`, `use_lidar`, `use_micro`, `use_control`, `use_display`.

## Tests / lint

- `colcon test` runs ament linters (flake8, pep257, xmllint, lint_cmake) via `ament_lint_auto`. The full suite passes (22 tests, 0 failures).
- Use `--packages-select <pkg> --event-handlers console_direct+` to run and see output for a single package.
- Environment gotcha: if `/opt/miniconda3/bin` is on `PATH`, CMake picks the conda python and `colcon build` fails (`ModuleNotFoundError: No module named 'catkin_pkg'`). Build with a clean PATH: `PATH=/usr/bin:/bin` and unset `CONDA_PREFIX`/`CONDA_PYTHON_EXE`/`CONDA_DEFAULT_ENV`.

## Architecture & naming — keep names in sync

- `urdf/robot.urdf.xacro` includes `robot_core.xacro` (chassis + 4 wheels), `lidar.xacro`, `imu.xacro`, `kinect.xacro`, `ros2_control.xacro`.
- Wheel joints are `chassis_{fr,fl,rl,rr}_wheel_joint`. The command interface is `voltage`; state interfaces are `position`/`velocity`. These names must stay consistent across `robot_core.xacro`, `ros2_control.xacro`, and `config/controllers.yaml`.
- `ros2_control.xacro` declares a custom system hardware plugin `mecanum_hardware_interface/MicroRosInterface` (external repo); IMU state interfaces there are commented out.
- `config/controllers.yaml`: controller_manager at 70 Hz; `joint_state_broadcaster`, `forward_voltage_controller`, `jacobian`, `inverse_jacobian`. Note the `forward_voltage_controller` spawner is **not spawned** — it was removed from `ros2_control_bringup.launch.py` (dead code); the `jacobian`/`inverse_jacobian` controllers own voltage commands.
- Hardware defaults: LiDAR on `/dev/ttyUSB0` (sllidar_ros2), micro-ROS agent serial on `/dev/ttyUSB1`. Kinect frame ids: `kinect_rgb_optical_link`, `kinect_depth_optical_link`. `ROS_DOMAIN_ID=1` is set in the Dockerfile.

## Known inconsistency

- Default branch is `jazzy` (upstream also has `main`; README says `git clone -b jazzy`). The Dockerfile now targets `jazzy`; earlier it was `ros:humble` — if you see `humble` anywhere, it is stale.
- CI: `.github/workflows/build.yml` builds and lints only the two tracked packages (self-contained, no external repos). It is green; the full workspace (external repos) is not built in CI.
