# Preparation changes and validation

## Changes made for this portfolio

- Organized the supplied material by workstream and wrote English reader guides from the CV, diary, source, and test records.
- Produced an English CSV, machine-readable counts, and an outcome plot from the 71 supplied IK summaries. Removed local attachment/user paths from JSON and an embedded workbook source path in the public evidence copy; kept the unmodified inputs locally.
- Translated selected examples' messages, argument help, and explanatory text. Kept joint values, service names, register addresses, and control flow. Original source remains in the release snapshots.
- Repaired the IK viewer's model import path, replaced the head-metrology wrapper's stale Windows filename with a CLI input, removed an unused machine-specific path from the offset client, and added test discovery paths.
- Made the B-hand loops require a sourced ROS environment, preserving their wiring, timeout, and interval settings. Changed the force diagnostic's unverified `g` display label to `device units`.
- Excluded an unrelated controller maintenance/reboot script with an embedded credential from the public runtime bundle.
- Added a checksum-verifying source fetcher. In the extracted working copy it relocates three files' native-library paths. It leaves release archives unchanged and records the patched filenames. Added missing empty ament resource markers when assembling the runtime snapshot.

These are repository-preparation changes, not claims about software originally written or tests originally executed during the internship.

## Offline checks

Checks were performed on macOS using Python 3.12. NumPy, Pandas, Matplotlib, and Pytest versions observed in that environment are recorded in [validated-environment.txt](validated-environment.txt). Requirements list direct dependencies without guessing the original robot's package versions.

- **19 tests passed.** Existing IK utility tests cover transform rejection, line spacing, endpoint inclusion, grid order, pose residuals, sampled boundary summaries, and joint CSV loading. Additional tests reconcile the supplied records and documented brackets, distinguish requested from recorded motion, decode synthetic RS485 force/tactile frames, reject malformed frames, check camera-transform composition and degree/radian equivalence, and reject unsafe ZIP paths.
- Ran the README's record-summary command, camera-ground CLI, object-ground CLI, left-arm FK CLI, and recorded-joint viewer with `--no-plot`. These calculated model outputs successfully without ROS or hardware.
- Download-bundle extraction was tested using the local release files with `--all --from-directory`; all three SHA-256/size checks passed. Relocated Python source parsed successfully.
- Statically checked that every MARVIN/Inspire service or message type imported by the primary clients has a definition in the bundled interface packages. This checks dependency availability, not live ROS service compatibility.
- Parsed selected Python files and the included ROS runtime, checked local Markdown links, reviewed the generated plot, and checked the selected/public text for credential patterns and accidental CV contact details.

Synthetic protocol frames are test fixtures, not recorded measurements. The normalized data reproduce existing record summaries, not a new hardware test.

## Unverified or unavailable

ROS 2, the Linux aarch64 libraries, controller firmware compatibility, colcon builds, perception topics, native SDK loading, and real robot motion were not exercised on this host. The hardware setup commands are documented from source inspection and require validation on the actual robot.

The later full-body interface/server and exact dated client from the transcripts are missing. No attempt is made to replace them with the earlier arm/leg backend. Original camera logs and the reported 568-record CSV dataset are missing. Raw hand acquisition logs, surveyed localization ground truth, a force calibration, navigation exports, and the original metrology workbook are also unavailable.

One supplied numerical-model README is unreadable binary data. It was retained locally and replaced with a fresh English guide. Original Chinese narratives/workbooks and multilingual third-party snapshots remain optional evidence/reference downloads; the repository is not claimed to be a translation of every vendor file.
