# Robot runtime and dependency setup

## Available runtime

The primary scripts use the existing `/arm/robot_control` and `/hand/hand_control` service nodes. The source snapshot includes all four relevant packages: `marvin_robot`, `marvin_interfaces`, `inspire_hand_robot`, and `inspire_hand_interface`. It also includes `jibot_kine`, vendor Python wrappers, YAML/CSV model parameters, the M6 configuration, and native libraries. This avoids distributing only clients with missing local imports.

The included `.so` files are **Linux aarch64 ELF** libraries. They cannot run on macOS merely because the host has an ARM CPU. A Linux aarch64 robot computer with the compatible controller is required. Other Linux architectures need matching libraries from the vendor. The actual controller firmware used for the recorded experiments is not established by the supplied evidence.

The classic offset client uses `Marvin_Kine.ik` through `arm_ik_node`; it does not need the optional Pinocchio service. The available Pinocchio implementation depends on the robotics Pinocchio library and its own CSV model. Some supplementary platform Modbus drivers import `pymodbus`; the direct hand-force Modbus TCP application uses standard-library `socket` and `struct` instead. Neither route is required for the demonstrated ROS hand-service path. Versions for the optional platform integrations have not been verified here.

## Obtain and build

On an existing Ubuntu/ROS 2 Humble robot development system with `colcon`, ROS message generators, NumPy, and PyYAML available:

```bash
# Run from this repository root using the system Python appropriate for ROS 2.
python3 tools/fetch_sources.py
source /opt/ros/humble/setup.bash
cd .runtime/ros_ws
colcon build --symlink-install --packages-select marvin_interfaces inspire_hand_interface marvin_robot inspire_hand_robot
source install/setup.bash
cd ../..
```

The fetcher relocates fixed native-library paths to their module directory. Use a sourced system ROS environment rather than the separate offline-analysis venv. `--symlink-install` preserves the sibling-source layout expected by the optional Pinocchio service. This build recipe is derived from the supplied package metadata; it has **not** been built on this macOS preparation host.

## Existing robot services

Prefer the platform's already-running nodes. Do not start competing native SDK connections. Check the available graph before running a client:

```bash
ros2 node list
ros2 service list
ros2 topic list
```

If the normal controller services are absent, an operator familiar with the platform can start the included nodes in separate sourced terminals:

```bash
ros2 run marvin_robot robot_node --ros-args -r __ns:=/arm -r __node:=robot_control
ros2 run inspire_hand_robot hand_node --ros-args -r __ns:=/hand -r __node:=hand_control
ros2 run marvin_robot frame_kinematics_node
ros2 run marvin_robot arm_ik_node
```

After a manual controller-node startup, establish its controller connection before expecting arm feedback (use the actual LAN address):

```bash
ros2 service call /arm/connect_robot marvin_interfaces/srv/Connect "{robot_ip: '192.168.1.190'}"
```

The camera-frame path also requires the platform's body/head controller to publish fresh `/joint_states` in radians. The included frame node subscribes to that topic plus `/arm/joint_states` (or `/arm/joint_positions_deg`); starting the four nodes alone does not create body/head feedback or camera detections. Those hardware drivers are prerequisites from the existing robot stack, outside this selected arm/hand source bundle.

These nodes connect software to real hardware and are not an offline simulation. The inherited supervised launch is preserved as reference, but is not the recommended startup entry here: it can enable position/torque modes and contains a machine-specific shutdown path. No controller update, automatic motion, or launch was executed during preparation.

## Read-only examples

With the existing robot stack running and the workspace sourced:

```bash
python3 examples/ros2/camera_to_ground_test.py
# Alternative tag-pose client, requiring an existing tag detector:
python3 examples/ros2/camera_to_ground_pose_test.py
# B-hand acquisition, each in its own terminal:
bash examples/hand/read_touch_loop.sh
bash examples/hand/read_force_loop.sh
```

Camera clients additionally require the upstream perception publisher (`vision_msgs` or `geometry_msgs`) and `/frame/head_ground_frames`. A camera driver or complete perception stack is not implemented by these clients.

## Classic arm calculation

```bash
export ARM_IP=192.168.1.190  # replace with the controller's actual LAN address
python3 examples/ros2/cartesian_offset_ik.py --arm A --dx 10 --dy 0 --dz 0 --dry-run
```

Even dry-run requires robot state and a running IK service. The client uses generated ROS interfaces and the sourced service nodes; frame conversion occurs on the server. This classic example is distinct from the later full-body test records. Drag-mode and hand-motion examples change real actuator state; inspect their parameters and the platform's operating procedure before use.

## Missing later backend

The archived later clients import `marvin_interfaces.srv.PinocchioFullBodyIK` and call `/ik/full_body_ik`. The supplied August interface package defines arm/leg IK and FK services, but not that full-body service. No matching implementation or exact dated client cited in the transcripts was supplied. A replacement has not been invented during preparation. Restore the matching interface, server, model/configuration, and dated client before attempting to rerun those formal tests.
