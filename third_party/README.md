# Dependency source snapshots

The [source-snapshots-v1 release](https://github.com/dddddddddl/robotics-internship-integration/releases/tag/source-snapshots-v1) carries the supplied source dependencies. Large vendor files are release assets so a normal clone stays small. All asset sizes and SHA-256 hashes are in [source_manifest.json](source_manifest.json).

| Archive | Contents |
| --- | --- |
| `robot_runtime_source_snapshot.zip` | Relevant August ROS packages: `marvin_robot`, `marvin_interfaces`, `inspire_hand_robot`, `inspire_hand_interface`; arm and camera demo directories; native libraries and configurations. Missing ament resource markers were restored as empty marker files. |
| `marvin_vendor_sdk_source_snapshot.zip` | Complete user-provided MARVIN vendor SDK ZIP, byte-for-byte. Includes control/kinematics SDKs, Python/C++ examples, configurations, and GUI material. |
| `hand_diagnostics_source_snapshot.zip` | Additional supplied hand scripts, local hand SDK helpers, tests, and original-language manual summaries. |

Retrieve the runtime only, or all snapshots:

```bash
python tools/fetch_sources.py
# On a clean extraction directory, use this instead for all three bundles:
python tools/fetch_sources.py --all
```

The fetcher checks hashes, rejects path traversal and symlink entries, and refuses to overwrite an existing extracted target. It relocates three runtime files' native-library paths in the extracted working copy. The release archives stay unchanged; `.runtime/relocation_changes.json` records the edits. It does not install dependencies, start services, connect to a controller, or issue motion commands.

Original vendor licenses are retained under [licenses/](licenses/). Attribution and package-level licensing gaps are explained in [provenance](../docs/provenance.md). Availability here does not turn third-party code into personally authored code or supply a new blanket license.
