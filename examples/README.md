# ROS 2 integration examples

These are English reading copies of selected supplied scripts. Service names, register addresses, joint values, and control flow are preserved. Chinese explanatory comments/docstrings were replaced by English module descriptions and this guide. Minor portability and display changes are listed in [validation](../docs/validation.md). Original source remains available in the dependency snapshots.

| File | Purpose | Hardware effect |
| --- | --- | --- |
| [camera_to_ground_test.py](ros2/camera_to_ground_test.py) | Transform detection points using the current camera-to-ground matrix | Read-only subscriptions |
| [camera_to_ground_pose_test.py](ros2/camera_to_ground_pose_test.py) | Transform tag positions and orientations | Read-only subscriptions |
| [cartesian_offset_ik.py](ros2/cartesian_offset_ik.py) | Send a body-frame offset target to the arm IK service; the server converts it into the vendor IK frame | Calculates by default; `--execute` sends motion |
| [arm_ros2_common.py](ros2/arm_ros2_common.py) | Shared controller-service calls, A/B indexing, shutdown | Uses the existing controller connection |
| [drag_arm_a.py](ros2/drag_arm_a.py), [drag_arm_b.py](ros2/drag_arm_b.py) | Enable drag mode and observe arm feedback | Changes arm control mode |
| [hand_open_close.py](ros2/hand_open_close.py) | Hand speeds, force limits, and angle sequences | Moves fingers unless `--dry-run` |
| [read_touch_loop.sh](hand/read_touch_loop.sh) | Repeated `/hand/read_touch` calls, B/COM3/ID1 | Read-only |
| [read_force_loop.sh](hand/read_force_loop.sh) | Repeated `/hand/read_actual_forces` calls, B/COM3/ID1 | Read-only |
| [11_hand_read_forces_485.py](hand/11_hand_read_forces_485.py) | Parse six signed register channels through `/arm/transfer_485` | Read-only, A/COM2/ID1 |
| [13_hand_read_touch_485.py](hand/13_hand_read_touch_485.py) | Read five fingertip modules and decode float32 fields | Read-only, A/COM2/ID1 |

The B-hand polling scripts reflect the diary's tested wiring. The direct-transfer Python diagnostics retain their original A-hand wiring; they are alternative examples, not interchangeable hardware settings. Polling uses timeouts of 5/7 seconds (service/shell) for touch and 8/12 seconds for actual force, with 0.5/1.5 second intervals respectively. The scripts now require an explicitly sourced environment rather than silently using an old home-directory workspace.

The original force diagnostic displayed `g`; the English copy uses `device units` because the supplied evidence does not establish a calibration for the tested device. Tactile normal/tangential names are protocol fields, not a calibration claim.

The [runtime guide](../docs/robot-setup.md) explains how to obtain and build all four ROS packages, including the generated interfaces, helper modules, YAML/CSV configurations, and native libraries. Supplementary hand status, free-position, grasp, Modbus, and SDK examples are preserved in the hand reference release bundle; their execution is outside the diary-backed demonstration scope.

## Additional application development

The published hand reference bundle also contains single-finger, grasp-sequence and six-channel position applications with two existing offline test files. [Hand-control development](../docs/hand-control-development.md) maps those sources, explains the differing completion rules and write verification, and provides the 14-test command. The last supplied grasp deployment reached a pre-motion RS485 timeout; no subsequent completed grasp is documented.
