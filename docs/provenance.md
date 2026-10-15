# Source provenance and attribution

## Internship materials

The owner supplied a CV, a narrative internship log, a local source folder, a large ROS workspace archive, and a smaller hand-workspace snapshot. The CV/log support the role and practical activities. The diary's inconsistent date order was not used to invent a chronology. Its content is organized by technical workstream in the case study. The private CV and its contact details are not published.

The supplied Python files are integration/reference material with mixed or unestablished authorship. They are identified as supplied scripts, adapted examples, or dependencies. Personal contributions are described as test setup, parameter adaptation, execution, diagnostic acquisition, and interpretation supported by the diary. This portfolio does not establish original authorship of every script or assign the vendor's controller/solver implementation to the intern.

The smaller hand snapshot was selected for the relevant runtime because it includes the arm/hand service packages together. The larger archive fills license/resource omissions and supplies camera/arm examples. Backup files remain within the compact runtime snapshot where they were already part of that package; primary examples use the non-backup files. Neither timestamp nor a filename alone establishes that the snapshot matches the later full-body experiment backend.

## MARVIN upstream

The official source reference supplied by the owner is [cynthia-you/TJ_FX_ROBOT_CONTRL_SDK](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK). At inspection, `main` contained the license while the implementation was on `master`. The inspected implementation revision was [`53ebaa809c2ee5d90b9bed1cafd22c970bd24228`](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/tree/53ebaa809c2ee5d90b9bed1cafd22c970bd24228).

| Upstream directory | Role |
| --- | --- |
| [contrlSDK100343](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/tree/53ebaa809c2ee5d90b9bed1cafd22c970bd24228/contrlSDK100343) | Control SDK described by the supplied links as requiring controller version 100343 or newer |
| [contrlSDK](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/tree/53ebaa809c2ee5d90b9bed1cafd22c970bd24228/contrlSDK) | Older control SDK described as supporting controllers no newer than 100341 |
| [kinematicsSDK](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/tree/53ebaa809c2ee5d90b9bed1cafd22c970bd24228/kinematicsSDK) | Native kinematic calculations |
| [SDK_PYTHON](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/tree/53ebaa809c2ee5d90b9bed1cafd22c970bd24228/SDK_PYTHON) | Python SDK wrappers and native libraries |
| [DEMO_PYTHON](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/tree/53ebaa809c2ee5d90b9bed1cafd22c970bd24228/DEMO_PYTHON), [DEMO_C++](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/tree/53ebaa809c2ee5d90b9bed1cafd22c970bd24228/DEMO_C%2B%2B) | Usage examples |
| [MarvinPlatform_EN](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/tree/53ebaa809c2ee5d90b9bed1cafd22c970bd24228/MarvinPlatform_EN) | English platform/debugging distribution supplied upstream |

The provided local SDK differs from the inspected `master` revision in some files. The release preserves the local snapshot instead of silently upgrading its controller-facing code. Controller 100342 compatibility is not established by the supplied version descriptions; check with the vendor before selecting a control library.

The supplied URDF link pointed to a branch using a `blob` URL. The actual [documentation branch at revision `2ae22e4bec5ea1ce8bebbfcd4c15713bc56414b6`](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/tree/2ae22e4bec5ea1ce8bebbfcd4c15713bc56414b6/URDF) contains M3/M6 arm URDF archives. Those are upstream arm resources; they do not provide the complete wheeled robot's calibrated model or replace the missing full-body service.

## Licenses and publication scope

The MARVIN SDK includes an Apache 2.0 license with the original vendor copyright. That notice is preserved verbatim, including its Chinese legal entity name. `marvin_interfaces` also supplies an Apache 2.0 notice. The hand packages declare Apache 2.0 in their package metadata. The supplied `marvin_robot` metadata still says `TODO: License declaration`; this repository does not infer a blanket Apache license for all internship source. No new repository-wide license has been added.

An unrelated controller maintenance/reboot client containing an embedded login credential is excluded from the runtime release; its original remains local. It is not imported by the selected examples. 

Unrelated voice/AI packages, proprietary-marked force-sensor packages outside the demonstrated dependency path, navigation assets with no connection to the documented work, caches, installed dependencies, and build logs are excluded. Manual summaries in the optional hand reference archive remain attributed reference material; they are not recast as an original hardware manual.

A complete source-to-destination inventory and unmodified input backups are kept locally for the owner. The public evidence archive redacts local user/attachment paths while retaining numerical test values and original narrative evidence.

## Supplementary work summaries

Two later owner-supplied folders describe hand application development and fingertip-target IK integration, with source inventories and selected historical debugging excerpts. They add context for task requirements, client adaptation, deployment, software decisions and observed failures. The published English development notes synthesize those materials and the matching source already in the release/reference archives; they do not duplicate the private conversation excerpts, local paths or host identifiers.

The excerpts explicitly show Codex assistance in code changes, analysis and regression testing. The personal role is described through requirements, field feedback, deployment, execution and interpretation. Claims of independent authorship of the SDK, hand model or IK solver are not added. Later documentation/export utilities are preparation artifacts, not evidence of extra internship experiments.

## How to read the evidence

| Claim type | Source and boundary |
| --- | --- |
| Implemented functions and dependencies | Supplied source and attributed snapshots establish that the code exists. They do not establish who wrote every function or whether it ran successfully on hardware. |
| IK counts, brackets and joint-arrival discrepancies | Recomputed from supplied JSON summaries, with the original-language evidence preserved. Most records are summaries rather than raw controller logs. |
| Vision drift, hand-sensing observations and navigation work | Reported in the owner's CV, diary or supplied work notes. Missing raw vision logs, calibrated hand measurements and navigation exports prevent independent reproduction of those observations. |
| Grasp-client debugging and deployment | Supplied excerpts document assisted iteration and a field run ending at a pre-motion read timeout. No later successful complete grasp is inferred. |
| English guides, normalized data, figure, fetcher and added checks | Created during portfolio preparation. Offline test passes validate the checked software behavior, not new robot experiments. |
