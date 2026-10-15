#!/usr/bin/env python3
"""Generate English tables from supplied test summaries, without executing robot code."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def load_records(path=ROOT / 'data/ik/source_evidence.zip'):
    records = []
    with zipfile.ZipFile(path) as z:
        for name in sorted(z.namelist()):
            if name.endswith('.json'):
                raw = z.read(name)
                d = json.loads(raw)
                d['_file'] = name
                d['_sha256'] = hashlib.sha256(raw).hexdigest()
                records.append(d)
    return records


def normalize(d):
    repeat = d.get('record_type') == 'repeat' or 'repeat' in d['_file']
    point = int(d.get('test_id', d.get('related_test_id', '070')))
    # Older summaries encode observed motion in the English status rather than a field.
    status = d['status']
    observed = d.get('actual_motion', d.get('motion_executed'))
    if observed is None:
        observed = status.startswith(('executed_success', 'retraction_executed')) or status == 'ik_and_motion_success_but_physical_contact_unsafe'
    row = dict(point_index=point, record_type='repeat' if repeat else 'formal',
               success=d['ik_success'], recorded_motion=bool(observed),
               execute_requested=d.get('execute', d.get('execute_requested', False)),
               status=status, direction=d.get('direction',d.get('direction_ground','none')).replace('（远离腰部）',' (away from waist)').replace('并改变姿态',' with orientation change'),
               distance_mm=d.get('distance_mm',''),
               cumulative_distance_mm=d.get('cumulative_distance_from_common_baseline_mm',d.get('cumulative_distance_mm','')),
               position_error_m=d.get('position_error_m'),
               orientation_error_rad=d.get('orientation_error_rad'),
               iterations=d.get('iterations',d.get('arm_iterations','')),
               leg_solution_enabled=d.get('leg_enable',False),
               suspected_contact=bool(d.get('field_safety_event')),
               source_file=d['_file'], source_sha256=d['_sha256'])
    for label, keys in [('baseline',('baseline_xyz_m','baseline_sensor_2_xyz_m')),('target',('target_xyz_m','target_sensor_2_xyz_m'))]:
        values = next((d[k] for k in keys if k in d),[None]*3)
        for axis,val in zip('xyz', values):row[f'{label}_{axis}_m']=val
    joints=next((d[k] for k in ['joint_angles_deg','ik_joint_angles_deg','returned_joint_angles_deg'] if d.get(k)),[])
    for i in range(7):row[f'joint_{i+1}_deg']=joints[i] if i<len(joints) else ''
    return row


def summary(rows):
    formal=[r for r in rows if r['record_type']=='formal']
    return dict(formal_records=len(formal),repeat_records=len(rows)-len(formal),
                recorded_ik_success=sum(r['success'] for r in formal),
                recorded_ik_failure=sum(not r['success'] for r in formal),
                recorded_motion=sum(r['recorded_motion'] for r in formal),
                suspected_contact_records=[r['point_index'] for r in formal if r['suspected_contact']],
                evidence_type='Supplied structured test summaries; only test 001 includes raw terminal output.',
                hardware_reexecuted=False)


def plot_summary(rows,dest):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    formal=[r for r in rows if r['record_type']=='formal']
    fig,ax=plt.subplots(figsize=(11,3.3),layout='constrained')
    for label,y,color,select in [('Recorded IK success',1,'#157b74',lambda r:r['success']),('Recorded IK failure',0,'#bd4b3e',lambda r:not r['success'])]:
        group=[r for r in formal if select(r)]
        ax.scatter([r['point_index'] for r in group],[y]*len(group),color=color,s=40,label=label)
    moving=[r for r in formal if r['recorded_motion']]
    ax.scatter([r['point_index'] for r in moving],[1.22]*len(moving),marker='|',s=100,color='#335b8e',label='Motion recorded')
    ax.scatter([28],[1],s=170,facecolors='none',edgecolors='#bd4b3e',linewidths=2)
    ax.annotate('028: suspected waist contact',(28,1),xytext=(17,1.64),arrowprops={'arrowstyle':'->'},fontsize=9)
    ax.set(xlabel='Formal test ID (record sequence; not a time axis)',yticks=[0,1],yticklabels=['Failure','Success'],ylim=(-.45,1.95),xlim=(-1,72),title='Recorded IK outcomes: 70 tests, 54 successes, 16 failures')
    ax.legend(loc='upper center',bbox_to_anchor=(.69,1.01),frameon=False,fontsize=8)
    ax.grid(axis='x',alpha=.15)
    fig.savefig(dest,dpi=180);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,default=ROOT/'data/ik')
    parser.add_argument('--plot',type=Path)
    args=parser.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    rows=[normalize(d) for d in load_records()]
    with (args.output_dir/'records.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    result=summary(rows)
    (args.output_dir/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    if args.plot:args.plot.parent.mkdir(parents=True,exist_ok=True);plot_summary(rows,args.plot)
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
