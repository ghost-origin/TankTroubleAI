# -*- coding: utf-8 -*-
"""Independent turn benchmark for the current Virtual Tube follower.

This benchmark intentionally removes A*, enemies and tactical replanning.  It
asks one question only: can the current follower execute a known smooth turn
without overshoot/collision?

Default suite:
- 30/45/60/90/120/135 deg, left + right (12 cases)
- 90 deg tight corridor, left + right
- 90 deg outer-wall regression, left + right
- S curve, left-right + right-left
"""
from __future__ import annotations

from pathlib import Path
import sys
PROJECT_ROOT = Path(__file__).resolve().parents[2]
PYTHON_ROOT = PROJECT_ROOT / 'python'
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

import argparse
import csv
import json
import math
import os
from dataclasses import asdict
from typing import List, Sequence, Tuple

import numpy as np

from benchmark_sim import (
    DT, GOAL_RADIUS_PX, TankState, WaypointFollower, add_outer_wall,
    body_collides, corridor_wall_mask, free_distance, make_open_wall_mask,
    min_turn_radius_from_track, moving_stats, nearest_polyline_distance,
    step_car, trajectory_distance, trajectory_smoothness_per_100, wrap,
)
from navigation_mvp import MAP_PX, Point, catmull_rom_smooth


def turn_path(angle_deg: float, direction: str, sample_step: float = 3.0) -> Tuple[List[Point], float, int]:
    sign = -1 if direction == 'left' else 1   # screen y grows down
    start = (105.0, 285.0)
    corner = (285.0, 285.0)
    th = math.radians(sign*angle_deg)
    end = (corner[0] + 180.0*math.cos(th), corner[1] + 180.0*math.sin(th))
    path = catmull_rom_smooth([start, corner, end], sample_step)
    return path, th, sign


def s_curve_path(first: str, sample_step: float = 3.0) -> Tuple[List[Point], float, int]:
    sign = -1 if first == 'left' else 1
    pts = [
        (80.0, 285.0),
        (210.0, 285.0),
        (305.0, 285.0 + sign*95.0),
        (400.0, 285.0),
        (520.0, 285.0),
    ]
    return catmull_rom_smooth(pts, sample_step), 0.0, sign


def build_cases():
    cases=[]
    for deg in (30,45,60,90,120,135):
        for direction in ('left','right'):
            cases.append({'name':f'T{deg}_{direction}','kind':'open','angle_deg':deg,'direction':direction})
    for direction in ('left','right'):
        cases.append({'name':f'T90_tight_{direction}','kind':'tight','angle_deg':90,'direction':direction})
        cases.append({'name':f'T90_outer_wall_{direction}','kind':'outer_wall','angle_deg':90,'direction':direction})
    cases.append({'name':'S_left_right','kind':'s_curve','direction':'left'})
    cases.append({'name':'S_right_left','kind':'s_curve','direction':'right'})
    return cases


def simulate_case(case: dict, out_dir: str, max_time: float = 8.0) -> dict:
    if case['kind']=='s_curve':
        path, desired_heading, sign = s_curve_path(case['direction'])
    else:
        path, desired_heading, sign = turn_path(case['angle_deg'],case['direction'])

    if case['kind']=='tight':
        raw = corridor_wall_mask(path, 34.0)
    else:
        raw = make_open_wall_mask()
    if case['kind']=='outer_wall':
        raw = add_outer_wall(raw,path,sign,offset=42.0,width=5)
    fd = free_distance(raw)

    start=path[0]
    # Initial tangent rather than blindly assuming zero; S-curve and ordinary turns both start straight.
    initial_heading=math.atan2(path[1][1]-path[0][1],path[1][0]-path[0][0])
    st=TankState(start[0],start[1],initial_heading)
    follower=WaypointFollower(path)
    rows=[]
    collision=False
    reached=False
    min_clear=float('inf')
    max_cross=0.0
    sum_cross=0.0
    progress_samples=[]
    heading_settle=None
    settle_band=math.radians(5.0)
    t=0.0
    while t <= max_time+1e-9:
        cross=nearest_polyline_distance((st.x,st.y),path)
        clear=float(fd[min(MAP_PX-1,max(0,int(round(st.y)))),min(MAP_PX-1,max(0,int(round(st.x))))])
        min_clear=min(min_clear,clear);max_cross=max(max_cross,cross);sum_cross+=cross
        rows.append({'t':t,'x':st.x,'y':st.y,'angle':st.angle,'move_angle':st.move_angle,
                     'speed':st.speed,'cross_track_px':cross,'wall_clearance_px':clear,
                     'wp_idx':follower.wp_idx})
        if math.hypot(st.x-path[-1][0],st.y-path[-1][1]) <= GOAL_RADIUS_PX:
            reached=True;break
        action=follower.action(st)
        nxt=step_car(st,action,DT)
        if body_collides(raw,nxt.x,nxt.y,nxt.angle):
            collision=True
            # Record first contact pose approximately at the attempted pose.
            st=nxt;t+=DT
            rows.append({'t':t,'x':st.x,'y':st.y,'angle':st.angle,'move_angle':st.move_angle,
                         'speed':st.speed,'cross_track_px':nearest_polyline_distance((st.x,st.y),path),
                         'wall_clearance_px':0.0,'wp_idx':follower.wp_idx})
            break
        st=nxt;t+=DT
        if case['kind']!='s_curve':
            progress=sign*math.degrees(wrap(st.angle-initial_heading))
            progress_samples.append(progress)
            if heading_settle is None and abs(wrap(st.angle-desired_heading)) <= settle_band:
                heading_settle=t

    total_d=trajectory_distance(rows)
    avg_speed,moving_speed,idle_ratio=moving_stats(rows)
    smooth=trajectory_smoothness_per_100(rows)
    heading_exit=abs(math.degrees(wrap(st.angle-desired_heading)))
    if case['kind']=='s_curve':
        overshoot=0.0
    else:
        overshoot=max(0.0,(max(progress_samples) if progress_samples else 0.0)-float(case['angle_deg']))
    min_r=min_turn_radius_from_track(rows)
    mean_cross=sum_cross/max(1,len(rows))

    os.makedirs(out_dir,exist_ok=True)
    csv_path=os.path.join(out_dir,case['name']+'.csv')
    with open(csv_path,'w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys()));w.writeheader();w.writerows(rows)

    # Plot is diagnostic, not part of score.
    try:
        import matplotlib.pyplot as plt
        fig,ax=plt.subplots(figsize=(6,6))
        ax.imshow(raw,cmap='gray_r',origin='upper',extent=(0,MAP_PX,MAP_PX,0),alpha=.35)
        ax.plot([p[0] for p in path],[p[1] for p in path],label='planned centerline')
        ax.plot([r['x'] for r in rows],[r['y'] for r in rows],label='actual tank center')
        ax.scatter([path[-1][0]],[path[-1][1]],marker='*',s=70,label='goal')
        ax.set_xlim(0,MAP_PX);ax.set_ylim(MAP_PX,0);ax.set_aspect('equal')
        ax.set_title(case['name']);ax.set_xlabel('x (px)');ax.set_ylabel('y (px)');ax.legend(fontsize=8)
        fig.tight_layout();fig.savefig(os.path.join(out_dir,case['name']+'.png'),dpi=130);plt.close(fig)
    except Exception as e:
        print('plot warning',case['name'],e)

    return {
        'case':case['name'],'kind':case['kind'],'angle_deg':case.get('angle_deg',''),'direction':case['direction'],
        'success':int(reached and not collision),'collision':int(collision),'time_s':t,
        'total_distance_px':total_d,'average_speed_px_s':avg_speed,'moving_speed_px_s':moving_speed,
        'idle_ratio':idle_ratio,'smooth_rad_per_100px':smooth,'mean_cross_track_px':mean_cross,
        'max_cross_track_px':max_cross,'min_wall_clearance_px':min_clear,
        'heading_exit_error_deg':heading_exit,'overshoot_deg':overshoot,
        'heading_settle_time_s':'' if heading_settle is None else heading_settle,
        'steer_flip_count':follower.steer_flips,
        'min_turn_radius_px':'' if min_r is None else min_r,
    }


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--out-dir',default='turn_benchmark_results')
    ap.add_argument('--max-time',type=float,default=8.0)
    a=ap.parse_args();os.makedirs(a.out_dir,exist_ok=True)
    results=[]
    for case in build_cases():
        r=simulate_case(case,a.out_dir,a.max_time);results.append(r)
        print(case['name'],'success',r['success'],'collision',r['collision'],
              'overshoot',round(r['overshoot_deg'],2),'exit_err',round(r['heading_exit_error_deg'],2),
              'cross_max',round(r['max_cross_track_px'],2))
    summary_csv=os.path.join(a.out_dir,'turn_summary.csv')
    with open(summary_csv,'w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(results[0].keys()));w.writeheader();w.writerows(results)
    valid=[r for r in results if r['success']]
    summary={
        'n_cases':len(results),'n_success':sum(r['success'] for r in results),
        'n_collision':sum(r['collision'] for r in results),
        'success_rate':sum(r['success'] for r in results)/len(results),
        'median_overshoot_deg':float(np.median([r['overshoot_deg'] for r in results])),
        'median_heading_exit_error_deg':float(np.median([r['heading_exit_error_deg'] for r in results])),
        'median_max_cross_track_px':float(np.median([r['max_cross_track_px'] for r in results])),
        'note':'Independent turn benchmark. Not part of combat score.',
    }
    with open(os.path.join(a.out_dir,'turn_summary.json'),'w',encoding='utf-8') as f:
        json.dump(summary,f,ensure_ascii=False,indent=2)
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
