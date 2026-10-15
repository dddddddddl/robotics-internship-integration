#!/usr/bin/env python3
"""Download, verify and unpack the supplied dependency snapshots. Never starts a robot."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import tempfile
import urllib.request
import zipfile

ROOT=Path(__file__).resolve().parents[1]
RELEASE='https://github.com/dddddddddl/robotics-internship-integration/releases/download/source-snapshots-v1'


def safe_extract(archive: Path, destination: Path):
    with zipfile.ZipFile(archive) as z:
        for member in z.infolist():
            path=PurePosixPath(member.filename.replace('\\','/'))
            if path.is_absolute() or '..' in path.parts or (path.parts and ':' in path.parts[0]):
                raise ValueError(f'Unsafe archive path: {member.filename}')
            if stat.S_ISLNK(member.external_attr >> 16):
                raise ValueError(f'Symlinks are not supported: {member.filename}')
        z.extractall(destination)


def patch_runtime(runtime: Path):
    """Relocate native-library paths in the extracted reading copy only."""
    package=runtime/'ros_ws/src/marvin_robot'
    old='/home/user/jibot_ros2_opensource/30_ros_ws/src/drivers/marvin_robot/marvin_robot'
    changed=[]
    for name in ['robot_control_node.py','fx_robot.py','fx_kine.py']:
        p=package/'marvin_robot'/name
        s=p.read_text()
        if old in s:
            s=s.replace(repr(old), 'os.path.dirname(os.path.abspath(__file__))')
            for lib in ['libMarvinSDK.so','libKine.so']:
                s=s.replace(repr(old+'/'+lib), f"os.path.join(os.path.dirname(os.path.abspath(__file__)), '{lib}')")
            p.write_text(s);changed.append(str(p.relative_to(runtime)))
    # Pinocchio service locates its sibling source directory; use --symlink-install.
    (runtime/'relocation_changes.json').write_text(json.dumps(changed,indent=2)+'\n')
    return changed


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--all',action='store_true',help='Also fetch full vendor SDK and supplementary hand references')
    parser.add_argument('--from-directory',type=Path,help='Use previously downloaded archives (offline verification)')
    args=parser.parse_args()
    manifest=json.loads((ROOT/'third_party/source_manifest.json').read_text())
    destination=ROOT/'.runtime';destination.mkdir(exist_ok=True)
    names=list(manifest) if args.all else ['robot_runtime_source_snapshot.zip']
    for name in names:
        target=destination/({'robot_runtime_source_snapshot.zip':'ros_ws', 'marvin_vendor_sdk_source_snapshot.zip':'vendor_sdk','hand_diagnostics_source_snapshot.zip':'hand_references'}[name])
        if target.exists():raise SystemExit(f'{target} already exists; move it aside before a clean extraction.')
        with tempfile.TemporaryDirectory(dir=destination) as temp:
            temp=Path(temp);archive=temp/name
            if args.from_directory:shutil.copyfile(args.from_directory/name,archive)
            else:
                print(f'Downloading {name} ...')
                with urllib.request.urlopen(RELEASE+'/'+name,timeout=120) as response,archive.open('wb') as out:shutil.copyfileobj(response,out)
            metadata=manifest[name]
            if archive.stat().st_size!=metadata['bytes'] or hashlib.sha256(archive.read_bytes()).hexdigest()!=metadata['sha256']:
                raise SystemExit(f'Checksum or size mismatch: {name}')
            unpack=temp/'unpacked';unpack.mkdir();safe_extract(archive,unpack)
            if name=='robot_runtime_source_snapshot.zip':
                shutil.move(str(unpack/'ros_ws'),target)
                print('Relocated:',patch_runtime(destination))
            else:shutil.move(str(unpack),target)
        print(f'Verified and extracted {name} to {target.relative_to(ROOT)}')

if __name__=='__main__':main()
