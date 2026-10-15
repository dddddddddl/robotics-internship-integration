# Fingertip-target IK client integration

The supplementary task notes and selected historical excerpts explain why the later client differs from the classic Cartesian TCP-offset example. Both client files in [later_full_body_clients.zip](../references/later_full_body_clients.zip) match the supplement's source hashes. The numbered test results remain the same [70 formal records and one repeat](../data/ik/README.md); this update adds their engineering context.

## Define the requested control point

The task targeted the left index finger's `sensor_2` frame in ground coordinates. The arm solver expects a TCP target, so the client uses the supplied hand FK transform to compute:

```text
T_ground_tcp_target = T_ground_sensor_target @ inverse(T_tcp_sensor)
T_ground_sensor_solved = T_ground_tcp_solved @ T_tcp_sensor
```

The selected finger and modeled hand command (`index`, `1000`) must remain consistent when forming and checking a target. A model command is not a measurement of the physical finger pose. The Python arm visualization is another offline aid; it is not the backend solver or an independent fingertip metrology system.

## Diagnose a missing frame-chain input

The historical task excerpts show the IK service and arm feedback being available while `/frame/arm_ground_frames` returned `Waiting for fresh /joint_states` and empty transforms. A topic query reported no publisher for the body state. Arm joints alone cannot supply the body-to-ground part of that chain.

The later client offers an explicit zero-body-state calculation mode: publish the named zero-valued body joints needed by the frame computation without commanding or enabling body actuators. It checks for an external state publisher and stops the assumed source if a real publisher appears. This is a constrained calculation assumption for a body physically fixed at the assumed pose, not a repair of the body driver or an estimate of its unknown state.

Client adaptation and diagnosis were carried out with Codex assistance. The operator clarified the target frame and task, reported the missing body feedback, executed robot-side commands, and returned physical observations and terminal results.

## Separate calculation from execution

The supplied single-target client supports relative XYZ changes, optional skipping of preliminary motion, explicit confirmation for execution under the assumed-body mode, joint feedback checks and normal disable-after-motion behavior. Later full-body calculations can return nonzero leg solutions and body-frame changes. An arm-only execution path would then fail to realize the configuration used by the solver, so those results are not executed through that path.

The final local client has `LEG_ENABLE=True`, whereas earlier arm executions used the arm-only stage. Historical commands are therefore not a universal recipe for rerunning every record against the final file. The exact dated client used in the robot transcripts and the matching full-body backend are still missing.

## Individual runs and batch tooling

| Source | Responsibility |
| --- | --- |
| `20_ik_hand_move_target_matrix.py` in the reference archive | Single sensor target, hand/TCP composition, state-source assumption, service feedback and optional arm execution |
| `21_arm_ik_line_boundary.py` in the reference archive | Line/grid targets, per-point requests and result export; line execution has waypoint/joint-change checks, grid mode calculates only |
| [ik_experiment_core.py](../experiments/ik/ik_experiment_core.py) | Rigid-transform validation, line spacing, inclusive axes, snake-order grids and pose residuals |
| [view_ik_result.py](../experiments/ik/view_ik_result.py) | Read recorded arm joints and display offline model FK |

The formal records mainly came from individual runs with terminal feedback returned for assisted analysis and tabulation. The existence of a batch exporter does not mean all 70 tests were collected automatically through it.

## Interpret the evidence at the right level

Tests 001–006 cover six fixed-orientation X+ targets over 0–50 mm. Tests 007–011 record five executed 10 mm steps; the largest arrival joint error in those five summaries is 0.038 degrees. That is a recorded controller-feedback discrepancy, not measured Cartesian accuracy or a continuous straight-line tracking result.

The later X+, Y+ and Z+ brackets each narrow the recorded service outcome interval to 6.25 mm. Some failures reach iteration limits; others have small numerical residuals but are rejected by leg-change constraints. These are different reasons, and neither should be relabeled as a success merely because one residual is small. The Z− bracket stays 350–400 mm: the repeat is another 350 mm run, not a missing 375 mm sample.

Test 028 adds a separate physical criterion: numerical success and arrival were followed by suspected waist contact reported by the operator. Test 029 records retraction and clearance confirmation. The client was not thereby shown to contain complete collision checking or autonomous contact recovery.

Detailed brackets and caveats are in the [evidence guide](../data/ik/README.md). Joint arrival, model IK/FK consistency, service acceptance and physical clearance are different measurements. Full-body searches remained calculation only; no executed whole-body coordination or global workspace benchmark is claimed.

## Reproduction boundary

The archived ROS clients depend on `PinocchioFullBodyIK`, `/ik/full_body_ik`, the corresponding model/configuration, and the exact deployed client version. The available runtime provides earlier arm/leg services, not that later backend. The English notes therefore explain the existing implementation and recorded decisions without supplying a replacement and assigning it to the internship.

Local user paths, SSH details and private conversation identifiers from the supplied evidence are not published. The supplemental reports are later summaries of existing work, not additional robot experiments. The earlier [runtime guide](robot-setup.md) remains the setup entry point for the available classic arm-service path.
