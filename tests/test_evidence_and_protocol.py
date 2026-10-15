"""Preparation checks using supplied records and synthetic protocol frames; no ROS/hardware."""
import ast
import math
from pathlib import Path
import struct
import zipfile

import numpy as np
import pytest
from summarize_ik import load_records, normalize, summary
from fetch_sources import safe_extract
from jibot_kinematics import camera_to_ground, object_to_ground, body_to_camera, leg_to_body

ROOT=Path(__file__).resolve().parents[1]


def pure_protocol(name):
    """Load only constants and pure codec functions from the actual ROS script AST."""
    tree=ast.parse((ROOT/'examples/hand'/name).read_text())
    names={'bytes_to_hex','hex_to_bytes','build_read_request','find_frame_start',
           'unsigned_to_signed','parse_read_response','parse_register_response',
           'registers_to_float','parse_touch_values'}
    nodes=[n for n in tree.body if isinstance(n,ast.Assign) or isinstance(n,ast.FunctionDef) and n.name in names]
    namespace={'struct':struct,'math':math}
    exec(compile(ast.Module(body=nodes,type_ignores=[]),name,'exec'),namespace)
    return namespace


def response(address,values,device=1):
    payload=struct.pack('<'+'H'*len(values),*[v & 0xffff for v in values])
    frame=bytearray([0xeb,0x90,device,len(payload)+3,0x11,address & 255,address >> 8])+payload
    frame.append(sum(frame[2:]) & 255)
    return frame


def test_reconcile_records_and_separate_repeat():
    rows=[normalize(d) for d in load_records()]
    counts=summary(rows)
    assert (counts['formal_records'],counts['repeat_records'],counts['recorded_ik_success'],counts['recorded_ik_failure'],counts['recorded_motion'])==(70,1,54,16,29)
    assert sorted(r['point_index'] for r in rows if r['record_type']=='formal')==list(range(1,71))
    assert counts['suspected_contact_records']==[28]
    for r in rows:
        if r['point_index']>=46:assert not r['recorded_motion']
    assert next(r for r in rows if r['point_index']==31)['execute_requested']
    assert not next(r for r in rows if r['point_index']==31)['recorded_motion']


@pytest.mark.parametrize('ok,bad,gap',[(52,50,6.25),(58,56,6.25),(66,65,6.25),(70,69,50)])
def test_documented_brackets_have_record_evidence(ok,bad,gap):
    records={int(d['test_id']):d for d in load_records() if 'test_id' in d}
    assert records[ok]['ik_success'] is True
    assert records[bad]['ik_success'] is False
    assert abs(records[ok]['distance_mm']-records[bad]['distance_mm'])==gap


def test_force_signed_values_and_checksum_rejection():
    codec=pure_protocol('11_hand_read_forces_485.py')
    values=[55,64,160,193,-32,-554]
    frame=response(1582,values)
    assert codec['parse_read_response'](frame.hex())==values
    frame[-1]^=1
    with pytest.raises(ValueError,match='checksum'):codec['parse_read_response'](frame.hex())


def test_force_wrong_device_and_truncated_frame_rejected():
    codec=pure_protocol('11_hand_read_forces_485.py')
    with pytest.raises(ValueError,match='Device ID'):codec['parse_read_response'](response(1582,[0]*6,2).hex())
    with pytest.raises(ValueError,match='Incomplete'):codec['parse_read_response'](response(1582,[0]*6)[:-2].hex())


def test_tactile_float_fields_and_nonfinite_rejection():
    codec=pure_protocol('13_hand_read_touch_485.py')
    registers=[0]*25
    registers[16:18]=struct.unpack('<HH',struct.pack('<f',1.25))
    registers[20:22]=struct.unpack('<HH',struct.pack('<f',-.5))
    raw=response(3174,registers)
    decoded=codec['parse_register_response'](raw.hex(),3174,25)
    assert codec['parse_touch_values'](decoded)=={'normal_force':1.25,'tangent_force':-.5}
    registers[16:18]=struct.unpack('<HH',struct.pack('<f',float('nan')))
    with pytest.raises(ValueError,match='Non-finite'):codec['parse_touch_values'](registers)


def test_camera_transform_composition_and_units():
    leg=[10,-5,3,2];head=[-9,4];point=[-.16,.02,.98]
    result=camera_to_ground(leg,head,unit="deg")
    expected=leg_to_body(leg)['T_ground_body'] @ body_to_camera(head)['T_body_camera']
    np.testing.assert_allclose(result['T_ground_camera'],expected)
    np.testing.assert_allclose(camera_to_ground(np.deg2rad(leg),np.deg2rad(head),unit='rad')['T_ground_camera'],expected)
    np.testing.assert_allclose(object_to_ground(point,leg,head,unit='deg')['object_xyz_ground'],(expected @ np.r_[point,1])[:3])
    np.testing.assert_allclose(expected[:3,:3].T @ expected[:3,:3],np.eye(3),atol=1e-12)
    assert np.linalg.det(expected[:3,:3])==pytest.approx(1)


def test_extract_rejects_traversal_before_writing(tmp_path):
    archive=tmp_path/'bad.zip'
    with zipfile.ZipFile(archive,'w') as z:z.writestr('../escape','bad')
    with pytest.raises(ValueError,match='Unsafe'):safe_extract(archive,tmp_path/'target')
    assert not (tmp_path/'escape').exists()
