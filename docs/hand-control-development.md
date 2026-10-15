# Dexterous-hand application development

The supplementary work notes add the development story behind the hand scripts already preserved in the [hand source release](https://github.com/dddddddddl/robotics-internship-integration/releases/tag/source-snapshots-v1). All 12 top-level Python files in the supplied hand inventory match that published snapshot by SHA-256. These notes add explanation and evidence context rather than a second code version.

## From observation to task execution

The applications build on the existing MARVIN controller, ROS 2 services and Inspire hand protocol. They cover read-only monitoring, single-channel adjustments, a grasp sequence, and six-channel target input. The user's documented role includes reporting field behavior, participating in coding-assisted iteration, deploying a revised script, running it on the robot computer, and returning terminal feedback. The recorded grasp changes and regression checks were developed with Codex assistance; the SDK and platform drivers remain attributed dependencies.

| Application in the source bundle | Implementation to inspect |
| --- | --- |
| `10_hand_sensor_monitor_ros2_fixed.py` | Existing `/hand` services for angles and combined state |
| `10_hand_sensor_monitor_ros2_direct_fixed.py` | Read angles through `/arm/transfer_485` to isolate the lower communication path |
| `11_hand_read_forces_modbus_tcp.py` | Standard-library socket/struct client, MBAP checks, exact-length receive and reconnect handling |
| `11_hand_read_forces_485.py` | Signed six-channel force decoding over ROS 2/RS485 |
| `12_hand_read_status_485.py` | Packed error/status/temperature bytes and fault-bit interpretation |
| `13_hand_read_touch_485.py` | Five-finger normal/tangential float32 decoding |
| `14_hand_single_finger_control_485.py` | Small channel changes with an attempted return |
| `15_hand_grasp_demo_485.py` | Precheck, configuration, opening, object-placement wait, grasp, hold, release and attempted parameter restoration |
| `16_hand_free_position_control_485.py` | Full or named partial targets, interactive input and position-specific completion checks |

The similar `11_hand_read_forces_rs485_ros2.py` remains an alternative reference. These paths describe different entry points, not an automatic backend-switching system.

## A stale status is not completion of a new command

The field report described fingers moving briefly rather than completing the intended grasp. The code review identified a completion rule that could accept an arrival/stop status left over from the preceding open command. That is a software failure mode supported by the regression tests; the later field run does not prove it was the sole cause of the physical symptom.

In `motion_finished()` the revised grasp logic checks each active channel. Actual angle feedback within 30 normalized command units of the target can finish that channel. Otherwise, force-stop completion requires the corresponding stop status plus evidence of movement greater than 10 command units or a load magnitude of at least `max(50, force_threshold // 2)`. Those thresholds are implementation settings, not measured positioning or force accuracy.

The grasp regression suite includes the counterexample “old arrival status, actual angles still 1000, new target 250.” It must remain unfinished. Another test accepts actual arrival even when the status value is not recognized.

## Verify the setting separately from the motion

Some writes can lack an acknowledgment. Treating such a timeout as proof that the setting was applied can hide an ignored command. The grasp client's `write_registers_verified()` writes the setting, waits 0.08 seconds, reads that register group back, and rejects a mismatch. A mock test supplies the old values after a write and checks that the mismatch raises an error.

Readback confirms the requested setting, while actual angle feedback is still needed to judge motion. The grasp client applies readback to its configuration/target writes. The free-position client does not implement equivalent verification for every setting; this is a remaining implementation gap.

## Grasping and positioning need different completion rules

An object can stop a grasp before the fingers reach their fully closed target. The grasp workflow can therefore accept an evidenced force stop. A position task instead requires the selected channels to reach their requested values; `wait_until_reached()` reports a premature force stop as `MotionError` while checking faults and temperatures during polling.

The six channels are pinky, ring, middle, index, thumb flexion and thumb rotation. Target values use the device's 0–1000 normalized angle convention, not degrees or the separate actuator-position register range. The free-position parser accepts six values or named partial updates; `-` fills a channel from its current feedback. English aliases are available, and the original Chinese aliases remain input-compatibility features.

## Protocol and diagnostic details

RS485 applications construct vendor `EB 90` frames with read/write opcodes `0x11`/`0x12`, little-endian address/data fields, and an additive checksum. Parsers check headers, device ID, length, checksum, opcode and address. Error/status/temperature groups contain six bytes packed into three 16-bit values; they must be unpacked low byte first. Force values use signed 16-bit conversion, while tactile values use float32 reconstruction.

The direct Modbus TCP application uses network byte order and validates transaction/protocol IDs, device ID, function code and payload size. It uses the Python standard library; optional platform Modbus drivers have their own dependencies. TCP and RS485 are alternative communication routes with different framing.

## Field evidence and current limit

The supplied debugging excerpts record a completed file transfer and a robot-side launch of the revised A-arm/COM2 grasp script. The controller connection responded, but the next fault-register read at address 1606 timed out with an empty RS485 response before the motion sequence began. No subsequent complete grasp success is supplied. This localizes that recorded failure to a hand-read request; it does not establish whether power, wiring, addressing, bus contention or another cause was responsible.

The earlier B-arm/COM3 read-only sensing observations in the internship diary are a separate setup and are retained in the [case study](technical-case-study.md). They do not establish that the later A-hand grasp sequence completed.

## Repeat the existing offline checks

On a clean snapshot extraction directory, from the repository root:

```bash
python tools/fetch_sources.py --all
python -m unittest discover -s .runtime/hand_references/hand_diagnostics -p 'test_*.py' -v
```

The fetcher refuses to overwrite existing extraction targets. If other snapshots have already been extracted, download only `hand_diagnostics_source_snapshot.zip` from the release, verify it against [source_manifest.json](../third_party/source_manifest.json), and extract it into `.runtime/hand_references/` before running the second command.

The **14 existing hand tests passed again during this documentation update**. They cover framing/checksum and byte unpacking, target parsing/range checks, position completion, stale grasp status, force-stop evidence and ignored-write rejection. Fake feedback, ROS import stubs and mocks make these offline checks; they do not model actual bus timing or mechanical response.

Cleanup actions require a working communication link. The single-finger example lacks continuous arrival verification and complete parameter restoration; grasp polling does not continuously check temperature; the free-position workflow lacks all of the grasp client's readback checks. None of these clients is a replacement for device-side stopping or a validated safety system.
