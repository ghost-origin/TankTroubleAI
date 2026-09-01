# -*- coding: utf-8 -*-
"""Navigation Arena: enemy-free deterministic "snake goal" benchmark.

A tank reaches a target, then the next target in a seeded sequence becomes active.
The target sequence is pre-generated from the maze, so two navigation versions with
same maze+seed receive exactly the same task sequence.

This is a fast kinematic benchmark layer.  Combat/headless real-game tests remain the
final integration benchmark.
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
import random
from collections import Counter
from typing import List, Optional, Sequence, Tuple

import numpy as np

from benchmark_sim import (
    DT, GOAL_RADIUS_PX, TankState, WaypointFollower, body_collides,
    maze_from_assets, min_turn_radius_from_track, moving_stats,
    nearest_polyline_distance, shortest_map_distance, step_car,
    trajectory_distance, trajectory_smoothness_per_100,
)
from navigation_mvp import (
    A_STAR_TOPOLOGY_RADIUS_PX, GRID_PX, MAP_PX, WINDOW_REPLAN_REMAINING_PX,
    astar, build_maps,
    cell_to_point, load_polys, nearest_free, path_length, plan_to_goal,
    point_to_cell, simplify_cells, catmull_rom_smooth,
)

GOAL_CLEARANCE_PX = 28.0
BANDS = {
    'short': (100.0, 210.0),
    'medium': (210.0, 360.0),
    'long': (360.0, 700.0),
}
BAND_WEIGHTS = [('short',0.30),('medium',0.40),('long',0.30)]
LONG_BANDS = {
    '400_600': (400.0, 600.0),
    '600_900': (600.0, 900.0),
    '900_plus': (900.0, float('inf')),
}
LONG_BAND_WEIGHTS = [('400_600', 0.45), ('600_900', 0.40), ('900_plus', 0.15)]


def weighted_band(rng: random.Random, weights=BAND_WEIGHTS) -> str:
    x=rng.random();s=0.0
    for name,w in weights:
        s+=w
        if x<=s:return name
    return weights[-1][0]


def safe_points(blocked: np.ndarray, free_dist: np.ndarray, clearance: float = GOAL_CLEARANCE_PX):
    pts=[]
    h,w=blocked.shape
    for gy in range(h):
        for gx in range(w):
            if blocked[gy,gx]:continue
            p=cell_to_point((gx,gy),GRID_PX)
            ix=min(MAP_PX-1,max(0,int(round(p[0]))));iy=min(MAP_PX-1,max(0,int(round(p[1]))))
            if free_dist[iy,ix] >= clearance:
                pts.append(p)
    return pts


def route_complexity(start,goal,blocked):
    sc=nearest_free(point_to_cell(start,GRID_PX,blocked),blocked,max_r=5)
    gc=nearest_free(point_to_cell(goal,GRID_PX,blocked),blocked,max_r=5)
    if sc is None or gc is None:return 'unreachable',0
    cells=astar(sc,gc,blocked)
    if not cells:return 'unreachable',0
    simp=simplify_cells(cells,blocked)
    turns=max(0,len(simp)-2)
    if turns==0:return 'straight',turns
    if turns==1:return '1_turn',turns
    if turns==2:return '2_turn',turns
    return 'complex',turns


def pre_generate_goals(points: Sequence[Tuple[float,float]], blocked: np.ndarray,
                       seed: int, n_goals: int, bands=BANDS, weights=BAND_WEIGHTS,
                       min_path_px: float = 0.0):
    rng=random.Random(seed)
    if len(points)<2:raise RuntimeError('not enough safe points')
    start=points[rng.randrange(len(points))]
    seq=[];prev=start
    for i in range(n_goals):
        band=weighted_band(rng, weights);lo,hi=bands[band]
        indices=list(range(len(points)));rng.shuffle(indices)
        chosen=None;shortest=None
        # exact-band pass
        for idx in indices[:800]:
            p=points[idx]
            if math.hypot(p[0]-prev[0],p[1]-prev[1])<60:continue
            d=shortest_map_distance(prev,p,blocked)
            if d is not None and d >= min_path_px and lo<=d<hi:
                chosen=p;shortest=d;break
        # Fallback keeps the task reachable. For the open-ended 900+ band,
        # choose the longest route sampled rather than inventing an upper bound.
        if chosen is None:
            mid=(lo+hi)/2 if math.isfinite(hi) else None;best=None
            for idx in indices[:800]:
                p=points[idx];d=shortest_map_distance(prev,p,blocked)
                if d is None or d < max(80.0, min_path_px):continue
                key=-d if mid is None else abs(d-mid)
                if best is None or key<best[0]:best=(key,p,d)
            if best is None:
                raise RuntimeError(f'could not generate a reachable goal >= {min_path_px:.0f}px')
            _,chosen,shortest=best
        complexity,turns=route_complexity(prev,chosen,blocked)
        seq.append({'index':i+1,'x':chosen[0],'y':chosen[1],'band':band,
                    'nominal_shortest_px':shortest,'complexity':complexity,'turns':turns,
                    'start_x':prev[0],'start_y':prev[1]})
        prev=chosen
    return start,seq


def diagnostic_goal_path(start, goal, blocked):
    """Permissive benchmark-only path when the strict Virtual Tube planner rejects.

    This does NOT change online navigation.  It lets the Arena measure what happens
    if the current Catmull-Rom smoothing is executed anyway, which is useful for
    reproducing overshoot/collision regressions instead of ending every test at
    the planning gate.
    """
    sc=nearest_free(point_to_cell(start,GRID_PX,blocked),blocked,max_r=5)
    gc=nearest_free(point_to_cell(goal,GRID_PX,blocked),blocked,max_r=5)
    if sc is None or gc is None:
        return None
    cells=astar(sc,gc,blocked)
    if not cells:
        return None
    cells=simplify_cells(cells,blocked)
    poly=[start]+[cell_to_point(c,GRID_PX) for c in cells[1:-1]]+[goal]
    return catmull_rom_smooth(poly,5.0)


def simulate_segment(st: TankState, goal, plan_path, raw_wall, free_dist, max_time: float):
    follower=WaypointFollower(plan_path)
    rows=[];collision=False;reached=False;t=0.0;max_cross=0.0;min_clear=float('inf')
    last_safe=TankState(st.x,st.y,st.angle,st.speed,st.move_angle)
    while t<=max_time+1e-9:
        cross=nearest_polyline_distance((st.x,st.y),plan_path)
        ix=min(MAP_PX-1,max(0,int(round(st.x))));iy=min(MAP_PX-1,max(0,int(round(st.y))))
        clear=float(free_dist[iy,ix])
        max_cross=max(max_cross,cross);min_clear=min(min_clear,clear)
        rows.append({'t':t,'x':st.x,'y':st.y,'angle':st.angle,'move_angle':st.move_angle,
                     'speed':st.speed,'cross_track_px':cross,'wall_clearance_px':clear,
                     'wp_idx':follower.wp_idx})
        if math.hypot(st.x-goal[0],st.y-goal[1])<=GOAL_RADIUS_PX:
            reached=True;break
        action=follower.action(st)
        nxt=step_car(st,action,DT)
        if body_collides(raw_wall,nxt.x,nxt.y,nxt.angle):
            collision=True;break
        last_safe=TankState(st.x,st.y,st.angle,st.speed,st.move_angle)
        st=nxt;t+=DT
    return st,last_safe,rows,reached,collision,follower,max_cross,min_clear


def follower_remaining_path(follower: WaypointFollower, st: TankState) -> float:
    if follower.wp_idx >= len(follower.path):
        return 0.0
    pts = [(st.x, st.y)] + list(follower.path[follower.wp_idx:])
    return path_length(pts)


def simulate_receding_segment(st: TankState, goal, initial_plan, raw_wall, blocked,
                              blocked_turn, free_dist, max_time: float):
    """Execute validated windows and replan before the current one is exhausted."""
    follower = WaypointFollower(initial_plan.path)
    current_path = initial_plan.path
    current_covers_goal = initial_plan.window_goal_reached
    current_replan_remaining = min(
        WINDOW_REPLAN_REMAINING_PX,
        max(12.0, initial_plan.window_target_length * 0.35),
    )
    rows = []
    collision = False
    reached = False
    t = 0.0
    max_cross = 0.0
    min_clear = float('inf')
    last_safe = TankState(st.x, st.y, st.angle, st.speed, st.move_angle)
    plan_attempts = 1
    plan_successes = 1
    plan_failures = 0
    windows_executed = 1
    steer_flips = 0
    planned_path_px = path_length(initial_plan.path)
    next_replan_t = 0.0
    stop_reason = ''

    while t <= max_time + 1e-9:
        cross = nearest_polyline_distance((st.x, st.y), current_path)
        ix = min(MAP_PX - 1, max(0, int(round(st.x))))
        iy = min(MAP_PX - 1, max(0, int(round(st.y))))
        clear = float(free_dist[iy, ix])
        max_cross = max(max_cross, cross)
        min_clear = min(min_clear, clear)
        remaining = follower_remaining_path(follower, st)
        rows.append({
            't': t, 'x': st.x, 'y': st.y, 'angle': st.angle,
            'move_angle': st.move_angle, 'speed': st.speed,
            'cross_track_px': cross, 'wall_clearance_px': clear,
            'wp_idx': follower.wp_idx, 'window_index': windows_executed,
            'window_remaining_px': remaining,
        })
        if math.hypot(st.x - goal[0], st.y - goal[1]) <= GOAL_RADIUS_PX:
            reached = True
            stop_reason = 'goal_reached'
            break

        exhausted = follower.wp_idx >= len(follower.path)
        needs_replan = exhausted or (
            not current_covers_goal and remaining <= current_replan_remaining
        )
        if needs_replan and t + 1e-9 >= next_replan_t:
            pr = plan_to_goal(
                (st.x, st.y), goal, raw_wall, blocked,
                blocked_turn=blocked_turn, free_dist=free_dist,
                start_heading=st.angle,
            )
            plan_attempts += 1
            if pr.success and pr.path_validated and len(pr.path) >= 2:
                steer_flips += follower.steer_flips
                follower = WaypointFollower(pr.path)
                current_path = pr.path
                current_covers_goal = pr.window_goal_reached
                current_replan_remaining = min(
                    WINDOW_REPLAN_REMAINING_PX,
                    max(12.0, pr.window_target_length * 0.35),
                )
                planned_path_px += path_length(pr.path)
                plan_successes += 1
                windows_executed += 1
                remaining = follower_remaining_path(follower, st)
                exhausted = False
                next_replan_t = t
            else:
                plan_failures += 1
                next_replan_t = t + 0.2
                if exhausted:
                    stop_reason = pr.reason or 'window_replan_failed'
                    break

        action = follower.action(st)
        nxt = step_car(st, action, DT)
        if body_collides(raw_wall, nxt.x, nxt.y, nxt.angle):
            collision = True
            stop_reason = 'collision'
            break
        last_safe = TankState(st.x, st.y, st.angle, st.speed, st.move_angle)
        st = nxt
        t += DT

    if not stop_reason:
        stop_reason = 'timeout'
    steer_flips += follower.steer_flips
    stats = {
        'window_plan_attempts': plan_attempts,
        'window_plan_successes': plan_successes,
        'window_plan_failures': plan_failures,
        'window_plan_coverage': plan_successes / max(1, plan_attempts),
        'windows_executed': windows_executed,
        'planned_path_px': planned_path_px,
        'steer_flip_count': steer_flips,
        'stop_reason': stop_reason,
    }
    return st,last_safe,rows,reached,collision,max_cross,min_clear,stats


def main():
    ap=argparse.ArgumentParser()
    here=os.path.dirname(os.path.abspath(__file__))
    repo=os.path.abspath(os.path.join(here,'..'))
    ap.add_argument('--repo',default=repo)
    ap.add_argument('--maze-index',type=int,default=0)
    ap.add_argument('--seed',type=int,default=1001)
    ap.add_argument('--goals',type=int,default=20)
    ap.add_argument('--profile',choices=('default','long'),default='default',
                    help='default preserves the original fixture; long uses only 400px+ A* routes')
    ap.add_argument('--goal-sequence',default='',
                    help='reuse an existing goal_sequence.json for exact A/B comparison')
    ap.add_argument('--topology-radius',type=float,default=A_STAR_TOPOLOGY_RADIUS_PX,
                    help='coarse A* wall padding; final swept footprint remains unchanged')
    ap.add_argument('--out-dir',default='navigation_arena_results')
    ap.add_argument('--segment-timeout',type=float,default=22.0)
    a=ap.parse_args();os.makedirs(a.out_dir,exist_ok=True)

    maze=maze_from_assets(os.path.join(a.repo,'assets','mazes.json'),a.maze_index)
    polys=load_polys(os.path.join(a.repo,'data log','tile_polys.json'))
    raw,_,blocked,blocked_turn,free_dist=build_maps(maze,polys,10,GRID_PX,
                                                    turn_radius_px=18.45,
                                                    straight_radius_px=a.topology_radius)
    pts=safe_points(blocked,free_dist)
    bands, weights = (LONG_BANDS, LONG_BAND_WEIGHTS) if a.profile == 'long' else (BANDS, BAND_WEIGHTS)
    min_path_px = 400.0 if a.profile == 'long' else 0.0
    if a.goal_sequence:
        with open(a.goal_sequence,encoding='utf-8') as f:
            fixture=json.load(f)
        start=tuple(fixture['start'])
        goals=fixture['goals'][:a.goals]
    else:
        start,goals=pre_generate_goals(pts,blocked,a.seed,a.goals,bands=bands,weights=weights,
                                       min_path_px=min_path_px)
    with open(os.path.join(a.out_dir,'goal_sequence.json'),'w',encoding='utf-8') as f:
        json.dump({'maze_index':a.maze_index,'seed':a.seed,'profile':a.profile,
                   'topology_radius_px':a.topology_radius,
                   'min_path_px':min_path_px,
                   'start':start,'goals':goals},f,ensure_ascii=False,indent=2)

    st=TankState(start[0],start[1],0.0)
    reset_to_nominal=True
    segment_rows=[];all_track=[];global_t=0.0;collisions=0;plan_failures=0;completed=0
    total_shortest=0.0;completed_actual=0.0;all_steer_flips=0
    window_plan_attempts=0;window_plan_successes=0;window_plan_failures=0;windows_executed=0
    complexity_counter=Counter();success_by_complexity=Counter();reason_counter=Counter()

    for g in goals:
        goal=(g['x'],g['y'])
        if reset_to_nominal:
            st=TankState(g['start_x'],g['start_y'],0.0)
        pr=plan_to_goal(
            (st.x,st.y),goal,raw,blocked,blocked_turn=blocked_turn,free_dist=free_dist,
            start_heading=None if reset_to_nominal else st.angle,
        )
        strict_plan_success=int(pr.success)
        execution_mode='strict_tube'
        if pr.success:
            exec_path=pr.path
            execution_mode='strict_windows'
        else:
            plan_failures+=1;reason_counter[pr.reason]+=1
            exec_path=diagnostic_goal_path((st.x,st.y),goal,blocked)
            execution_mode='diagnostic_spline'
            if not exec_path:
                window_plan_attempts+=1;window_plan_failures+=1
                segment_rows.append({**g,'success':0,'collision':0,'plan_success':0,'strict_plan_success':0,
                                     'execution_mode':'no_executable_path','plan_reason':pr.reason,
                                     'planned_path_px':0.0,'actual_distance_px':0.0,'map_efficiency':0.0,
                                     'window_plan_attempts':1,'window_plan_successes':0,
                                     'window_plan_failures':1,'window_plan_coverage':0.0,
                                     'windows_executed':0,'stop_reason':'no_executable_path',
                                     'time_s':0.0,'average_speed_px_s':0.0,'moving_speed_px_s':0.0,'idle_ratio':1.0,
                                     'smooth_rad_per_100px':0.0,'max_cross_track_px':0.0,'min_wall_clearance_px':'',
                                     'steer_flip_count':0,'min_turn_radius_px':''})
                reset_to_nominal=True
                continue
        if reset_to_nominal and len(exec_path) >= 2:
            initial_heading=math.atan2(exec_path[1][1]-exec_path[0][1],exec_path[1][0]-exec_path[0][0])
            st=TankState(st.x,st.y,initial_heading)
        complexity_counter[g['complexity']]+=1
        timeout=min(a.segment_timeout,max(8.0,g['nominal_shortest_px']/35.0+5.0))
        if execution_mode == 'strict_windows':
            end,last_safe,rows,reached,collision,max_cross,min_clear,window_stats=simulate_receding_segment(
                st,goal,pr,raw,blocked,blocked_turn,free_dist,timeout)
            segment_steer_flips=window_stats['steer_flip_count']
            planned_path_px=window_stats['planned_path_px']
        else:
            end,last_safe,rows,reached,collision,follower,max_cross,min_clear=simulate_segment(
                st,goal,exec_path,raw,free_dist,timeout)
            segment_steer_flips=follower.steer_flips
            planned_path_px=path_length(exec_path)
            window_stats={
                'window_plan_attempts':1,
                'window_plan_successes':0,
                'window_plan_failures':1,
                'window_plan_coverage':0.0,
                'windows_executed':0,
                'stop_reason':'diagnostic_fallback',
            }
        window_plan_attempts+=window_stats['window_plan_attempts']
        window_plan_successes+=window_stats['window_plan_successes']
        window_plan_failures+=window_stats['window_plan_failures']
        windows_executed+=window_stats['windows_executed']
        for r in rows:
            rr=dict(r);rr['t_global']=global_t+r['t'];rr['goal_index']=g['index'];all_track.append(rr)
        seg_time=rows[-1]['t'] if rows else 0.0;global_t+=seg_time
        actual=trajectory_distance(rows);avg,moving,idle=moving_stats(rows)
        smooth=trajectory_smoothness_per_100(rows);min_r=min_turn_radius_from_track(rows)
        success=int(reached and not collision)
        if collision:collisions+=1;reason_counter['collision']+=1
        elif not reached:reason_counter['timeout']+=1
        if success:
            completed+=1;success_by_complexity[g['complexity']]+=1
            total_shortest+=g['nominal_shortest_px'];completed_actual+=actual
            st=end; reset_to_nominal=False
        else:
            # Benchmark recovery only: next fixed task restarts from its nominal safe
            # predecessor goal. This prevents one collision from poisoning all later cases.
            reset_to_nominal=True
        all_steer_flips+=segment_steer_flips
        eff=min(1.0,g['nominal_shortest_px']/actual) if success and actual>1e-9 else 0.0
        segment_rows.append({**g,'success':success,'collision':int(collision),'plan_success':1,
                             'strict_plan_success':strict_plan_success,'execution_mode':execution_mode,
                             'plan_reason':pr.reason,'planned_path_px':planned_path_px,
                             'window_plan_attempts':window_stats['window_plan_attempts'],
                             'window_plan_successes':window_stats['window_plan_successes'],
                             'window_plan_failures':window_stats['window_plan_failures'],
                             'window_plan_coverage':window_stats['window_plan_coverage'],
                             'windows_executed':window_stats['windows_executed'],
                             'stop_reason':window_stats['stop_reason'],
                             'actual_distance_px':actual,'map_efficiency':eff,'time_s':seg_time,
                             'average_speed_px_s':avg,'moving_speed_px_s':moving,'idle_ratio':idle,
                             'smooth_rad_per_100px':smooth,'max_cross_track_px':max_cross,
                             'min_wall_clearance_px':min_clear,'steer_flip_count':segment_steer_flips,
                             'min_turn_radius_px':'' if min_r is None else min_r})

    fields=list(segment_rows[0].keys()) if segment_rows else []
    with open(os.path.join(a.out_dir,'arena_segments.csv'),'w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(segment_rows)
    if all_track:
        with open(os.path.join(a.out_dir,'arena_track.csv'),'w',newline='',encoding='utf-8-sig') as f:
            track_fields=list(dict.fromkeys(k for row in all_track for k in row.keys()))
            w=csv.DictWriter(f,fieldnames=track_fields);w.writeheader();w.writerows(all_track)

    total_distance=sum(float(r['actual_distance_px']) for r in segment_rows)
    total_time=sum(float(r['time_s']) for r in segment_rows)
    moving_dist=0.0;moving_time=0.0
    for r in segment_rows:
        if float(r['moving_speed_px_s'])>0:
            # derive moving time approximately from segment distance / moving speed
            mt=float(r['actual_distance_px'])/float(r['moving_speed_px_s']) if float(r['moving_speed_px_s'])>1e-9 else 0.0
            moving_time+=mt;moving_dist+=float(r['actual_distance_px'])
    nominal_lengths = [float(g['nominal_shortest_px']) for g in goals]
    summary={
        'benchmark':'Navigation Arena','profile':a.profile,'maze_index':a.maze_index,'seed':a.seed,
        'topology_radius_px':a.topology_radius,
        'goals_attempted':len(goals),'goals_completed':completed,'goal_success_rate':completed/max(1,len(goals)),
        'goals_per_minute':completed/(total_time/60.0) if total_time>1e-9 else 0.0,
        'strict_plan_successes':len(goals)-plan_failures,'strict_plan_success_rate':(len(goals)-plan_failures)/max(1,len(goals)),
        'nominal_a_star_path_px': {
            'min': min(nominal_lengths, default=0.0),
            'mean': float(np.mean(nominal_lengths)) if nominal_lengths else 0.0,
            'median': float(np.median(nominal_lengths)) if nominal_lengths else 0.0,
            'max': max(nominal_lengths, default=0.0),
        },
        'plan_failures':plan_failures,'collisions':collisions,
        'window_plan_attempts':window_plan_attempts,
        'window_plan_successes':window_plan_successes,
        'window_plan_failures':window_plan_failures,
        'strict_executable_window_coverage':window_plan_successes/max(1,window_plan_attempts),
        'windows_executed':windows_executed,
        'total_distance_px':total_distance,'total_time_s':total_time,
        'average_speed_px_s':total_distance/total_time if total_time>1e-9 else 0.0,
        'moving_speed_px_s':moving_dist/moving_time if moving_time>1e-9 else 0.0,
        'map_path_efficiency':min(1.0,total_shortest/completed_actual) if completed_actual>1e-9 else 0.0,
        'mean_smooth_rad_per_100px':float(np.mean([r['smooth_rad_per_100px'] for r in segment_rows if r['success']])) if completed else 0.0,
        'median_max_cross_track_px':float(np.median([r['max_cross_track_px'] for r in segment_rows if r.get('plan_success') and r['max_cross_track_px']!=''])) if any(r.get('plan_success') for r in segment_rows) else 0.0,
        'min_wall_clearance_px':min([float(r['min_wall_clearance_px']) for r in segment_rows if r['min_wall_clearance_px']!=''],default=0.0),
        'steer_flip_count':all_steer_flips,
        'diagnostic_fallback_segments':sum(1 for r in segment_rows if r.get('execution_mode')=='diagnostic_spline'),
        'diagnostic_fallback_collisions':sum(int(r['collision']) for r in segment_rows if r.get('execution_mode')=='diagnostic_spline'),
        'map_path_efficiency_scope':'successful goal segments only',
        'failure_reasons':dict(reason_counter),
        'success_by_complexity':{k:{'attempts':complexity_counter[k],'success':success_by_complexity[k],
                                    'rate':success_by_complexity[k]/complexity_counter[k] if complexity_counter[k] else 0.0}
                                 for k in sorted(complexity_counter)},
        'note':'Fast enemy-free kinematic benchmark; use real-game Combat benchmark for final validation.'
    }
    with open(os.path.join(a.out_dir,'arena_summary.json'),'w',encoding='utf-8') as f:
        json.dump(summary,f,ensure_ascii=False,indent=2)

    try:
        import matplotlib.pyplot as plt
        fig,ax=plt.subplots(figsize=(8,8))
        ax.imshow(raw,cmap='gray_r',origin='upper',extent=(0,MAP_PX,MAP_PX,0),alpha=.45)
        if all_track:
            # Each failed benchmark task may reset to the next task's nominal
            # start. Plot each goal segment independently so that those test
            # resets are never drawn as fake cross-maze trajectories.
            by_goal={}
            for r in all_track:
                by_goal.setdefault(int(r['goal_index']),[]).append(r)
            first=True
            for goal_idx in sorted(by_goal):
                seg=by_goal[goal_idx]
                ax.plot([r['x'] for r in seg],[r['y'] for r in seg],lw=1.0,
                        label='actual navigation track' if first else None)
                first=False
        ax.scatter([start[0]],[start[1]],marker='o',s=45,label='start')
        ax.scatter([g['x'] for g in goals],[g['y'] for g in goals],marker='*',s=35,label='goals')
        for g in goals:
            ax.text(g['x']+3,g['y']+3,str(g['index']),fontsize=6)
        ax.set_xlim(0,MAP_PX);ax.set_ylim(MAP_PX,0);ax.set_aspect('equal')
        ax.set_title(f'Navigation Arena [{a.profile}] maze={a.maze_index} seed={a.seed}')
        ax.set_xlabel('x (px)');ax.set_ylabel('y (px)');ax.legend(fontsize=8)
        fig.tight_layout();fig.savefig(os.path.join(a.out_dir,'arena_track.png'),dpi=140);plt.close(fig)
    except Exception as e:
        print('plot warning:',e)

    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
