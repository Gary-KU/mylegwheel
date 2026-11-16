"""Summarize one neutral parking run; exclude STOP rows and show raw metrics.

No hardware calls or binary claim of physical stability. A low final speed
or small net displacement alone does not establish stationary parking.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics
from read_lqr_startup_record import active_run_summaries


def summary(rows):
    def stats(values):
        values = [float(v) for v in values if v is not None and math.isfinite(v)]
        if not values:
            return None
        absolute = sorted(abs(v) for v in values)
        return dict(count=len(values), minimum=min(values), maximum=max(values),
                    mean=statistics.mean(values), standard_deviation=statistics.pstdev(values),
                    peak_to_peak=max(values)-min(values), mean_absolute=statistics.mean(absolute),
                    p95_absolute=absolute[math.ceil(.95*len(absolute))-1], max_absolute=max(absolute))
    speed = [r['state'][3] for r in rows]
    return dict(first_elapsed_ms=rows[0]['elapsed_ms'], last_elapsed_ms=rows[-1]['elapsed_ms'],
                observed_duration_ms=(rows[-1]['tick_ms']-rows[0]['tick_ms']) & 0xffffffff,
                count=len(rows), phases=dict(Counter(str(r['phase']) for r in rows)),
                speed_m_s=stats(speed), position_error_m=stats(r['position_error_m'] for r in rows),
                true_parking_goal_m=stats(r['reference'][2] for r in rows),
                net_travel_m=rows[-1]['state'][2]-rows[0]['state'][2],
                body_pitch_deg=stats(r['body_pitch_deg'] for r in rows),
                relative_leg_angle_deg=stats(math.degrees(sum(r['leg_angle_rad'])/2)-90 for r in rows),
                fraction_abs_speed_le_5mm_s=sum(abs(v)<=.005 for v in speed)/len(rows),
                fraction_abs_speed_le_20mm_s=sum(abs(v)<=.020 for v in speed)/len(rows),
                nonzero_forward_target_rows=sum(abs(r.get('forward_target_m_s',0))>1e-6 for r in rows),
                nonzero_yaw_target_rows=sum(abs(r.get('yaw_target_rad_s',0))>1e-6 for r in rows),
                wheel_saturation_rows=sum(r['wheel_voltage_saturated'] or r['wheel_Nm_saturated'] for r in rows),
                joint_saturation_rows=sum(r['joint_output_saturated'] for r in rows))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--run',type=int,help='Zero-based active run; required for mixed historical logs')
    parser.add_argument('--context',default='Contact/support conditions not specified to this analyzer')
    args=parser.parse_args()
    raw=args.input.read_bytes();data=json.loads(raw);records=data['records']
    runs=active_run_summaries(records,data['header']['trigger_index'])
    if not runs:
        parser.error('No active control run in this record')
    if args.run is None and len(runs)!=1:
        parser.error('Multiple launches: select --run explicitly; do not combine them')
    index=0 if args.run is None else args.run
    if not 0<=index<len(runs):parser.error('--run is outside the active-run list')
    run=runs[index];rows=records[run['start_index']:run['end_index']+1]
    windows={}
    for seconds in (1,2,3,5):
        selected=[r for r in rows if r['elapsed_ms']>=rows[-1]['elapsed_ms']-1000*seconds]
        windows[f'last_{seconds}s']=summary(selected)
    result=dict(source=str(args.input.resolve()),source_sha256=hashlib.sha256(raw).hexdigest(),
                build_id=data['header']['build_id'],generation=data['header']['generation'],
                context=args.context,active_run_count=len(runs),selected_run=index,
                all_active=summary(rows),trailing_windows=windows,
                stop_observed=run['stop_observed'],start_row=run['start_index'],end_row=run['end_index'],
                excluded_stop_rows=sum(not r['active'] for r in records),
                limitations='STOP rows excluded. Different initial pose/contact/trajectory is not a controlled A/B test. Encoder travel may differ from ground motion under slip. Speed thresholds are descriptive, not user-approved acceptance limits. No physical-stability pass/fail inferred.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    last=windows['last_2s']
    print(json.dumps(dict(build_id=result['build_id'],generation=result['generation'],
        active_rows=len(rows),last_2s_speed_m_s=last['speed_m_s'],
        last_2s_position_range_m=last['position_error_m']['peak_to_peak'],
        last_2s_net_travel_m=last['net_travel_m']),indent=2))


if __name__=='__main__':main()
