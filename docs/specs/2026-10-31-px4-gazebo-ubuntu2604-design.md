# PX4 SITL + Gazebo on Ubuntu 26.04 (WSL2) — Design

Date: 2026-10-31
Status: Approved design (pre-implementation)
Target host: Windows 11 + WSL2 Ubuntu 26.04.1 LTS
Project root: `~/drone_sim` inside Ubuntu-26.04

## Objective

Build a reproducible drone simulation environment inside WSL2 Ubuntu 26.04:

- Gazebo Jetty (gz-sim 10.5) from ROS 2 Lyrical vendor packages
- PX4-Autopilot v1.17.0 SITL built from source
- Gazebo rendering on the local NVIDIA RTX 5070 Ti Laptop GPU
- QGroundControl / MAVSDK connection and a first offboard waypoint flight

Primary route is native Ubuntu 26.04 (Plan A). Docker with the official
`px4io/px4-sitl-gazebo` image is the time-boxed fallback (Plan B).

## Verified Environment Facts

- OS: Ubuntu 26.04.1 LTS, glibc 2.43, x86_64, 32 CPU, 15 GiB RAM, 940 GiB free
- ROS 2 Lyrical installed at `/opt/ros/lyrical` (`ros2` requires sourcing)
- `ros-lyrical-ros-gz` 3.0.10 available from packages.ros.org (resolute)
- It vendors Gazebo Jetty: gz-sim 10.5.0, gz-transport, gz-gui, gz-rendering,
  gz-sensors, gz-tools, gz-physics, gz-msgs, sdformat, ogre-next
- No system `gz-harmonic` / `gz-jetty` apt packages are configured
- WSLg works: `DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0`, `/tmp/.X11-unix/X0`
- Networking: `.wslconfig` uses `[wsl2] networkingMode=mirrored`
- GPU: `nvidia-smi` sees RTX 5070 Ti Laptop GPU, 12 GiB, driver 615.71.08,
  CUDA 13.4; `/dev/dxg` and `/usr/lib/wsl/lib` are present
- Default OpenGL renderer is `llvmpipe` (software). Forcing D3D12 works:
  `GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`
  yields `D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)`,
  `Accelerated: yes`, OpenGL 4.6
- The machine also has an AMD Radeon 610M iGPU; D3D12 selects it by default,
  so the NVIDIA adapter must be selected explicitly
- PX4 v1.17 supports Gazebo Jetty (upstream PR #25521); PX4 officially
  supports Ubuntu 22.04/24.04, not 26.04 yet (issue #27086 open)
- `github.com` is reachable; `raw.githubusercontent.com` HEAD request timed out
  and may need workarounds during PX4 dependency setup
- Docker Desktop is installed on Windows; fallback image is 2.2 GB

## Architecture and Components

```
Windows 11
├─ QGroundControl (UDP 14550)          [manual verification]
├─ MAVSDK C++ examples (UDP 14540)     [automation]
└─ WSL2 Ubuntu-26.04
   ├─ ROS 2 Lyrical
   │  └─ ros_gz 3.0.10 (vendored Gazebo Jetty 10.5)
   ├─ ~/drone_sim/
   │  ├─ env.sh            GPU + ROS/PX4 environment
   │  ├─ start_sim.sh      one-command launch
   │  ├─ stop_sim.sh       cleanup
   │  ├─ scripts/          MAVSDK offboard example
   │  ├─ logs/             PX4/Gazebo logs
   │  ├─ docs/
   │  └─ PX4-Autopilot/    v1.17.0 source and build/
   └─ Gazebo GUI via WSLg on RTX 5070 Ti
```

- Gazebo is installed only through the ROS vendor packages (no system-wide
  OSRF repo, no Harmonic source build).
- PX4 is built for SITL only; no NuttX hardware toolchain is required.
- PX4's own Gazebo models/worlds (`gz_x500`) are used first. The RM27 world is
  not reused in this milestone.
- `env.sh` exports:
  - `GALLIUM_DRIVER=d3d12`
  - `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`
  - `QT_QPA_PLATFORM=xcb`
  - `QT_XCB_GL_INTEGRATION=xcb_glx`
  - `QT_OPENGL=desktop`
  - ROS 2 Lyrical setup and PX4 paths
- `start_sim.sh` refuses to launch unless `glxinfo -B` reports the RTX 5070 Ti
  and `Accelerated: yes`.

## Data Flow and Networking

- PX4 SITL runs on the host WSL network namespace (mirrored mode).
- PX4 <-> Gazebo: PX4 `gz_bridge` plugin over gz-transport; actuator commands
  out, IMU/GPS/baro/mag/camera in.
- PX4 -> GCS: MAVLink UDP 14550 (QGC).
- GCS/MAVSDK -> PX4: MAVLink UDP 14540.
- Windows QGC connects to `localhost:14550`; MAVSDK uses `udp://:14540`.
- Startup check: `ss -lunp | grep -E '14540|14550'`.
- If QGC cannot connect: try the WSL IP, set an explicit UDP endpoint in QGC,
  and check the Windows firewall.
- Phase 2 (out of scope here): `MicroXRCEAgent udp4 -p 8888` + `px4_msgs`,
  plus `ros_gz_bridge` for Gazebo clock/camera topics.

## GPU Rendering

- Rendering path: Gazebo Ogre2 -> OpenGL 4.6 -> Mesa D3D12 -> WSLg ->
  RTX 5070 Ti.
- GPU preflight is mandatory before every launch:
  `glxinfo -B | grep -E 'Device|Accelerated'`
- Expected: `D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)`,
  `Accelerated: yes`.
- Target: >= 30 FPS at 1280x720 in the `gz_x500` world.
- If software rendering persists: try `EGL_PLATFORM=x11`, `QT_OPENGL=desktop`,
  reduce resolution/shadows; if still no GPU within the time box, switch to
  Plan B or run headless.

## Milestones and Time Box

Plan A gets at most about 4 working hours to reach a flying simulation.

- M0 (~30 min): `ros-lyrical-ros-gz` installed; `gz sim` demo opens on the
  5070 Ti; GPU preflight passes.
- M1 (~2 h): PX4 v1.17.0 cloned, dependencies installed, SITL build succeeds,
  `make px4_sitl gz_x500` launches PX4 + Gazebo.
- M2 (~1 h): QGC or MAVSDK connects; arm, takeoff, land succeed.
- M3 (~1 h): MAVSDK offboard script flies a rectangular waypoint path and
  returns to start; position log is saved.

## Acceptance Criteria

- `glxinfo -B` shows the RTX 5070 Ti and `Accelerated: yes`.
- Gazebo GUI shows the x500; >= 30 FPS at 1280x720.
- PX4 console has no fatal errors; UDP 14540/14550 are listening.
- QGC or MAVSDK completes arm/takeoff/land.
- Offboard script reaches 3 waypoints and returns.
- A fresh shell can reproduce everything with `~/drone_sim/start_sim.sh`.
- `stop_sim.sh` leaves no PX4/Gazebo processes or occupied ports.

## Fallback Plan B

Trigger any of:

- PX4 CMake cannot find `gz-transport`/`gz-sim` after two explicit
  `CMAKE_PREFIX_PATH` attempts (1 h limit).
- Gazebo never leaves `llvmpipe` after Qt/EGL variants (1 h limit).
- PX4 Python 3.14 build scripts cannot be worked around within 1 h.

Plan B:

- `docker pull px4io/px4-sitl-gazebo`
- Run with `--network host`, WSLg X11 socket mount, `/dev/dxg`, and the same
  GPU environment variables; add `--gpus all` only if CUDA access is needed
  inside the container (Gazebo GL still goes through WSLg's D3D12 path).
- Acceptance criteria unchanged, except Gazebo is Harmonic 8.
- Deleting the container rolls back everything.

## Risks and Mitigations

- PX4 is not officially supported on Ubuntu 26.04: pin v1.17.0, keep changes
  in the project directory, and stop at the time box.
- ROS vendor Gazebo paths: set `CMAKE_PREFIX_PATH`, `PATH`, and Gazebo
  resource/plugin paths explicitly; patch PX4 CMake only if unavoidable.
- `raw.githubusercontent.com` may be unreachable: pre-download or mirror
  dependency scripts; verify early.
- Python 3.14 may break PX4 helper scripts: use a pinned interpreter/venv if
  needed.
- WSLg GPU selection: always force the NVIDIA adapter; never rely on defaults.
- Docker Desktop containers run in Docker's own WSL distro, not inside
  Ubuntu-26.04; this is accepted only for the fallback.

## Out of Scope

- ROS 2 bridge and uXRCE-DDS integration (phase 2)
- VIO/VINS-Fusion, camera calibration, YOLO/TensorRT deployment
- RM27 world/model reuse
- Real flight-controller hardware and NuttX builds
- Multi-drone/swarm simulation

## Deliverables

- `~/drone_sim/env.sh`, `start_sim.sh`, `stop_sim.sh`
- `~/drone_sim/scripts/` with an offboard waypoint example
- `~/drone_sim/docs/` with this spec and a README of known pitfalls
- `~/drone_sim/logs/` with PX4 console and offboard position logs
