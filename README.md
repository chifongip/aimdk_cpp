# aimdk_cpp

`aimdk_cpp` is the native ROS 2 and pybind11 backend used by RoboJuDo to communicate with an AgiBot X2 through AimDK. It subscribes to joint state, IMU, and optional odometry topics, while publishing grouped joint commands for the legs, waist, arms, and head.

The backend owns the real-time command publication loop and the command/state timeout safety state machine. Python supplies PD targets; it does not need to publish every actuator message itself.

## Requirements

- Python 3.10 or newer
- CMake 3.16 or newer
- ROS 2 with `rclcpp`, `sensor_msgs`, and `nav_msgs`
- An AimDK workspace that provides `aimdk_msgs`
- A C++17 compiler

Source ROS 2 and the AimDK workspace before building or importing the extension:

```bash
source /opt/ros/humble/setup.bash
source /path/to/aimdk/install/setup.bash
python -m pip install .
```

For development, install it in editable mode from this directory:

```bash
python -m pip install -e .
```

The build uses `scikit-build-core`, CMake, and pybind11. The generated Python package is named `aimdk_cpp`.

## Configuration

Construct `AimdkController` with either an `AimdkConfig` instance or a Python dictionary. The dictionary must provide the configured joint names, their leg/waist/arm/head grouping, and equally sized stiffness and damping vectors.

```python
from aimdk_cpp import AimdkController

controller = AimdkController(
    {
        "act": True,
        "joint_names": joint_names,
        "leg_joint_names": leg_joint_names,
        "waist_joint_names": waist_joint_names,
        "arm_joint_names": arm_joint_names,
        "head_joint_names": head_joint_names,
        "stiffness": stiffness,
        "damping": damping,
        "control_dt": 0.02,
        "publish_dt": 0.002,
        "command_timeout": 0.1,
        "command_damping_timeout": 0.5,
        "state_timeout": 0.1,
        "state_damping_timeout": 0.5,
        "odometry_timeout": 0.1,
        "odometry_damping_timeout": 0.5,
    }
)
```

All timing values must be finite and positive. Each damping timeout must be at least as long as its corresponding hold timeout. The generic C++ defaults retain legacy behavior by making each damping timeout equal to its hold timeout; the X2 RoboJuDo configuration uses a 100 ms hold deadline and a 500 ms damping deadline.

`control_dt` describes the upstream target cadence. `publish_dt` is the backend actuator publication cadence. For X2 deployment, Python should produce one target every 20 ms (50 Hz), while the backend continues to republish the latest target every 2 ms (500 Hz). Do not lower `publish_dt` merely to match the policy rate.

## Safety State Machine

When position control is active, the backend has three safety states:

| State | Meaning | Published command |
| --- | --- | --- |
| `ACTIVE` | Recent command and required state are fresh. | Latest requested PD target. |
| `HOLD` | A soft command, joint/IMU, or required odometry timeout occurred. | Latest measured joint positions with active PD gains. |
| `DAMPING` | A hard timeout or invalid hold state occurred. This state is latched. | Zero stiffness with damping. |

The required state streams are IMU plus every configured joint. When odometry is enabled, valid and fresh odometry is also required. Invalid or non-representable joint, IMU, and odometry samples are ignored and do not refresh their corresponding state timestamps.

The previous command age is checked before accepting a new target, so a late Python command cannot hide a host scheduling stall. A temporary hold recovers only after fresh required state is observed and a subsequent position command is received. A state-stream hold uses a two-phase recovery: all required state must first be fresh, then a command generated after that recovery is required. This avoids resuming on a command that was calculated from stale state.

Hard damping is intentionally latched. Recover it explicitly:

```python
controller.arm_position_control()
controller.step(fresh_pd_targets)
```

`set_passive()` and `set_damping(value)` select intentional non-position modes. They clear a non-latched hold, but they cannot clear a watchdog latch. `shutdown()` publishes damping for `shutdown_publish_duration` before stopping the executor.

Use `get_safety_status()` to inspect `ACTIVE`, `HOLD`, or `DAMPING`, the fault reason, whether damping is latched, and timing metadata. Use `get_state_freshness_report()` to diagnose missing, stale, or rejected state streams and inspect message-rate telemetry.

## Normal Control Sequence

1. Construct the controller with correct topics, joint names, gains, and optional odometry frames.
2. Call `self_check()` before enabling motion. It waits for required state streams.
3. Call `arm_position_control()` before the first position target or after a latched safety fault has been investigated and cleared.
4. Call `step(targets)` at the configured control rate. Targets must be finite and match `joint_names` exactly.
5. Call `shutdown()` during application teardown. It is safe to call more than once.

Before unattended deployment, validate the configured soft and hard deadlines on the actual robot and host workload. In particular, test CPU contention from perception or manipulation workloads, confirm measured-position hold is stable while standing, and verify damping occurs within the intended hard deadline.

## Development Checks

With ROS 2 and AimDK sourced, build a wheel without installing it:

```bash
python -m pip wheel . --no-deps --no-build-isolation --wheel-dir /tmp/aimdk_cpp-wheel
```

RoboJuDo contains integration tests for the binding and its configuration under `tests/test_x2_integration.py` and state-report formatting tests under `tests/test_x2_state_monitor.py`.
