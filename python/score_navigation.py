# -*- coding: utf-8 -*-
"""Score TankTrouble navigation tracks with corruption checks.

Core score (existing AI is normalized to 1):
- path: map-aware local efficiency = A* shortest feasible distance / actual distance
  over ~1 game-second windows. This avoids penalizing a necessary detour around a wall.
- smoothness: accumulated heading change per 100 px (lower is better).
- safety: hard gate. Any maze/track wall overlap rejects the match; valid matches get safety_score=1.
  Near-wall occupancy remains diagnostic only.

The older chord/actual local efficiency is still exported as a diagnostic field,
but it is no longer the core path score.
"""
from __future__ import annotations
import argparse, csv, glob, math, os, json
import numpy as np
from navigation_mvp import (
    load_maze, load_polys, build_maps,
    point_to_cell, cell_to_point, nearest_free, astar,
    distance_transform_edt,
)

MAX_FRAME_GAP_S = 2.0
MAX_SINGLE_STEP_PX = 120.0
WALL_OVERLAP_TOL_PX = 0.5
GRID_PX = 5
MAP_EFF_WINDOW_S = 1.0


def load(path):
    with open(path, newline='', encoding='utf-8-sig') as f:
        rows = []
        for r in csv.DictReader(f):
            out = {}
            for k, v in r.items():
                if v == '':
                    continue
                try:
                    out[k] = float(v)
                except ValueError:
                    out[k] = v  # 非数值列（如 controller_mode）原样保留
            rows.append(out)
        return rows


def validate_single_round(rows):
    """Reject known recording corruption before any score is computed."""
    if len(rows) < 3:
        return False, 'too_few_frames'
    for a, b in zip(rows, rows[1:]):
        dt = b['t'] - a['t']
        # Equal timestamps can occur for multiple bridge frames while Construct
        # game time is paused/frozen. Only true backwards time is corruption.
        if dt < -1e-6:
            return False, 'time_went_backwards'
        if dt > MAX_FRAME_GAP_S:
            return False, 'multi_round_gap_%.2fs' % dt
        dm = math.hypot(b['me_x']-a['me_x'], b['me_y']-a['me_y'])
        df = math.hypot(b['foe_x']-a['foe_x'], b['foe_y']-a['foe_y'])
        if dm > MAX_SINGLE_STEP_PX or df > MAX_SINGLE_STEP_PX:
            return False, 'position_jump_me_%.1f_foe_%.1f' % (dm, df)
    return True, ''


def resample_xy(rows, prefix, step=5.0):
    pts = [(r[prefix+'_x'], r[prefix+'_y']) for r in rows]
    if len(pts) < 2:
        return []
    out = [pts[0]]
    carry = 0.0
    cur = pts[0]
    for target in pts[1:]:
        sx, sy = cur
        tx, ty = target
        d = math.hypot(tx-sx, ty-sy)
        if d < 1e-9:
            cur = target
            continue
        while carry + d >= step:
            need = step - carry
            q = need / d
            sx = sx + (tx-sx)*q
            sy = sy + (ty-sy)*q
            out.append((sx, sy))
            d = math.hypot(tx-sx, ty-sy)
            carry = 0.0
            if d < 1e-9:
                break
        carry += d
        cur = target
    return out


def _cell_path_px(path, grid_px=GRID_PX):
    if not path or len(path) < 2:
        return 0.0
    return sum(math.hypot((b[0]-a[0])*grid_px, (b[1]-a[1])*grid_px)
               for a,b in zip(path,path[1:]))


def map_path_efficiency(rows, prefix, blocked, period=MAP_EFF_WINDOW_S, grid_px=GRID_PX):
    """Map-aware path efficiency over approximately one-game-second windows.

    For each valid motion window:
        efficiency = shortest feasible A* distance / actual traveled distance
    The score is capped at 1 to absorb small grid-snap discretization errors.
    """
    if len(rows) < 2:
        return 0.0, 0

    vals = []
    start_idx = 0
    next_t = rows[0]['t'] + period
    end_t = rows[-1]['t']

    while next_t <= end_t + 1e-9 and start_idx < len(rows)-1:
        end_idx = start_idx
        while end_idx + 1 < len(rows) and abs(rows[end_idx+1]['t'] - next_t) <= abs(rows[end_idx]['t'] - next_t):
            end_idx += 1
        if end_idx <= start_idx:
            # Advance at least one frame to avoid a repeated-timestamp loop.
            end_idx = min(len(rows)-1, start_idx + 1)

        seg = rows[start_idx:end_idx+1]
        actual = 0.0
        for a,b in zip(seg,seg[1:]):
            d = math.hypot(b[prefix+'_x']-a[prefix+'_x'], b[prefix+'_y']-a[prefix+'_y'])
            if d >= 0.15:
                actual += d

        start = (seg[0][prefix+'_x'], seg[0][prefix+'_y'])
        goal = (seg[-1][prefix+'_x'], seg[-1][prefix+'_y'])
        displacement = math.hypot(goal[0]-start[0], goal[1]-start[1])

        if actual >= 15.0 and displacement >= 5.0:
            sc = nearest_free(point_to_cell(start, grid_px, blocked), blocked, max_r=5)
            gc = nearest_free(point_to_cell(goal, grid_px, blocked), blocked, max_r=5)
            if sc is not None and gc is not None:
                pc = astar(sc, gc, blocked)
                if pc:
                    shortest = _cell_path_px(pc, grid_px)
                    # Include small start/end snap offsets so a 5 px grid does not
                    # systematically under-estimate the feasible path length.
                    shortest += math.hypot(start[0]-cell_to_point(sc,grid_px)[0],
                                           start[1]-cell_to_point(sc,grid_px)[1])
                    shortest += math.hypot(goal[0]-cell_to_point(gc,grid_px)[0],
                                           goal[1]-cell_to_point(gc,grid_px)[1])
                    vals.append(min(1.0, shortest / max(actual, 1e-9)))

        start_idx = end_idx
        next_t += period

    return (float(np.mean(vals)) if vals else 0.0), len(vals)


def trajectory_metrics(rows, prefix, raw_wall, blocked):
    pts = resample_xy(rows, prefix, 5.0)
    if len(pts) < 3:
        return None

    L = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(pts,pts[1:]))
    if L < 20.0:
        return None

    ang = [math.atan2(b[1]-a[1], b[0]-a[0]) for a,b in zip(pts,pts[1:])]
    turn = sum(abs((b-a+math.pi)%(2*math.pi)-math.pi) for a,b in zip(ang,ang[1:]))
    smooth100 = turn / max(L, 1.0) * 100.0

    # Legacy diagnostic: chord / traveled over ~50 px windows.
    win = 10
    eff = []
    for i in range(len(pts)-win):
        chord = math.hypot(pts[i+win][0]-pts[i][0], pts[i+win][1]-pts[i][1])
        traveled = sum(math.hypot(pts[j+1][0]-pts[j][0], pts[j+1][1]-pts[j][1])
                       for j in range(i, i+win))
        if traveled > 20:
            eff.append(chord/traveled)
    local_eff = float(np.mean(eff)) if eff else 0.0

    map_eff, map_windows = map_path_efficiency(rows, prefix, blocked)

    # Explicit behavior statistics requested by the navigation benchmark.
    duration_s = max(0.0, float(rows[-1]['t']) - float(rows[0]['t'])) if len(rows) >= 2 else 0.0
    average_speed = L / duration_s if duration_s > 1e-9 else 0.0
    moving_t = 0.0
    moving_d = 0.0
    for a, b in zip(rows, rows[1:]):
        dt = max(0.0, float(b['t']) - float(a['t']))
        if dt <= 1e-9:
            continue
        d = math.hypot(float(b[prefix+'_x'])-float(a[prefix+'_x']),
                       float(b[prefix+'_y'])-float(a[prefix+'_y']))
        if d / dt >= 5.0:
            moving_t += dt
            moving_d += d
    moving_speed = moving_d / moving_t if moving_t > 1e-9 else 0.0
    idle_ratio = 1.0 - moving_t / duration_s if duration_s > 1e-9 else 1.0

    dist = distance_transform_edt(~raw_wall)
    clear = []
    overlap = []
    for x,y in pts:
        xi = min(raw_wall.shape[1]-1, max(0, int(round(x))))
        yi = min(raw_wall.shape[0]-1, max(0, int(round(y))))
        d = float(dist[yi,xi])
        clear.append(d)
        overlap.append(bool(raw_wall[yi,xi]))

    min_clear = float(np.min(clear))
    near8 = float(np.mean(np.array(clear) <= 8.0))
    near12 = float(np.mean(np.array(clear) <= 12.0))
    overlap_ratio = float(np.mean(overlap))

    return dict(
        path_px=L,
        total_distance_px=L,
        duration_s=duration_s,
        average_speed_px_s=average_speed,
        moving_speed_px_s=moving_speed,
        idle_ratio=idle_ratio,
        smooth_rad_per_100px=smooth100,
        map_path_efficiency=map_eff,
        map_efficiency_windows=float(map_windows),
        local_efficiency=local_eff,
        min_clearance_px=min_clear,
        near_wall_8_ratio=near8,
        near_wall_12_ratio=near12,
        wall_overlap_ratio=overlap_ratio,
    )


def aggregate(metrics):
    keys = metrics[0].keys()
    return {k:float(np.median([m[k] for m in metrics])) for k in keys}


def relative(ours, ai):
    # Core path score is map-aware. local_efficiency and near-wall occupancy are
    # diagnostics only. Safety is a hard validity gate handled before scoring.
    return {
        'path_score': ours['map_path_efficiency']/max(ai['map_path_efficiency'],1e-9),
        'smooth_score': ai['smooth_rad_per_100px']/max(ours['smooth_rad_per_100px'],1e-9),
        'safety_score': 1.0,
        'near_wall_diagnostic_score': (1.0+ai['near_wall_8_ratio'])/(1.0+ours['near_wall_8_ratio']),
        'legacy_local_path_score': ours['local_efficiency']/max(ai['local_efficiency'],1e-9),
        'average_speed_score': ours['average_speed_px_s']/max(ai['average_speed_px_s'],1e-9),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-root', required=True)
    ap.add_argument('--polys', required=True)
    ap.add_argument('--out', default='navigation_scores.csv')
    ap.add_argument('--skipped-out', default='')
    ap.add_argument('--paired-out', default='', help='per-match relative score CSV')
    a = ap.parse_args()

    polys = load_polys(a.polys)
    per = []
    me_all = []
    foe_all = []
    skipped = []

    for track in sorted(glob.glob(os.path.join(a.data_root, '*', 'track_*.csv'))):
        maze = os.path.join(os.path.dirname(track), os.path.basename(track).replace('track_','maze_'))
        if not os.path.exists(maze):
            skipped.append((os.path.basename(track), 'maze_missing'))
            continue

        rows = load(track)
        ok, reason = validate_single_round(rows)
        if not ok:
            skipped.append((os.path.basename(track), reason))
            continue

        raw, _, blocked, _, _ = build_maps(load_maze(maze), polys, 8, GRID_PX)
        mm = trajectory_metrics(rows, 'me', raw, blocked)
        fm = trajectory_metrics(rows, 'foe', raw, blocked)
        if not mm or not fm:
            skipped.append((os.path.basename(track), 'insufficient_motion'))
            continue
        if mm['map_efficiency_windows'] < 1 or fm['map_efficiency_windows'] < 1:
            skipped.append((os.path.basename(track), 'insufficient_map_efficiency_windows'))
            continue

        # A tank center physically lying on the wall mask strongly indicates that
        # the maze snapshot and track do not belong to the same round.
        if mm['min_clearance_px'] <= WALL_OVERLAP_TOL_PX or fm['min_clearance_px'] <= WALL_OVERLAP_TOL_PX:
            skipped.append((os.path.basename(track), 'maze_track_mismatch_wall_overlap'))
            continue

        me_all.append(mm)
        foe_all.append(fm)
        per.append((os.path.basename(track), mm, fm))

    fields = ['file','actor','path_px','total_distance_px','duration_s','average_speed_px_s','moving_speed_px_s','idle_ratio','smooth_rad_per_100px','map_path_efficiency',
              'map_efficiency_windows','local_efficiency','min_clearance_px',
              'near_wall_8_ratio','near_wall_12_ratio','wall_overlap_ratio']
    with open(a.out, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for fn, mm, fm in per:
            w.writerow({'file':fn, 'actor':'green/nav', **mm})
            w.writerow({'file':fn, 'actor':'existing_ai', **fm})

    # Paired experiment: compute relative scores inside each same-match pair
    # first, then aggregate across matches. This controls for map/spawn difficulty.
    paired = []
    for fn, mm, fm in per:
        rr = relative(mm, fm)
        rr['composite_score'] = (max(rr['path_score'],1e-9) *
                                 max(rr['smooth_score'],1e-9) *
                                 max(rr['safety_score'],1e-9)) ** (1/3)
        paired.append({'file': fn, **rr})

    paired_out = a.paired_out or os.path.splitext(a.out)[0] + '_paired.csv'
    paired_fields = ['file','path_score','smooth_score','safety_score','composite_score',
                     'near_wall_diagnostic_score','legacy_local_path_score','average_speed_score']
    with open(paired_out, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=paired_fields)
        w.writeheader()
        w.writerows(paired)

    skipped_out = a.skipped_out or os.path.splitext(a.out)[0] + '_skipped.csv'
    with open(skipped_out, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(['file','reason'])
        w.writerows(skipped)

    result = {
        'score_definition': {
            'path': 'map-aware A* shortest feasible distance / actual distance, ~1s windows',
            'smooth': 'heading-change radians per 100px; lower is better',
            'safety': 'hard gate: any wall overlap/collision proxy rejects match; valid match safety_score=1',
            'distance_speed': 'total_distance_px and average_speed_px_s are reported as explicit behavior metrics; speed is diagnostic and is not yet added to composite weighting',
            'baseline': 'existing_ai = 1.0',
        },
        'n_candidate_tracks': len(per) + len(skipped),
        'n_valid_paired_matches': len(per),
        'n_skipped_matches': len(skipped),
        'skipped': [{'file':fn,'reason':r} for fn,r in skipped],
    }

    if per:
        ai = aggregate(foe_all)
        me = aggregate(me_all)

        def pair_summary(key):
            vals = np.array([float(r[key]) for r in paired], dtype=float)
            # Deterministic bootstrap so repeated scoring of the same data gives
            # the same uncertainty interval.  Resample paired matches, not actors.
            rng = np.random.default_rng(20260901)
            n = len(vals)
            idx = rng.integers(0, n, size=(10000, n))
            boot = vals[idx]
            boot_median = np.median(boot, axis=1)
            boot_mean = np.mean(boot, axis=1)
            return {
                'median': float(np.median(vals)),
                'median_ci95': [float(np.percentile(boot_median,2.5)),
                                float(np.percentile(boot_median,97.5))],
                'mean': float(np.mean(vals)),
                'mean_ci95': [float(np.percentile(boot_mean,2.5)),
                              float(np.percentile(boot_mean,97.5))],
                'win_rate_gt_1': float(np.mean(vals > 1.0)),
                'non_loss_rate_ge_1': float(np.mean(vals >= 1.0)),
                'min': float(np.min(vals)),
                'max': float(np.max(vals)),
            }

        pairwise_summary = {
            'path_score': pair_summary('path_score'),
            'smooth_score': pair_summary('smooth_score'),
            'safety_score': pair_summary('safety_score'),
            'composite_score': pair_summary('composite_score'),
        }
        # Official overall score = median of per-match paired ratios.
        official = {
            'path_score': pairwise_summary['path_score']['median'],
            'smooth_score': pairwise_summary['smooth_score']['median'],
            'safety_score': pairwise_summary['safety_score']['median'],
            'composite_score': pairwise_summary['composite_score']['median'],
        }
        result.update({
            'existing_ai_baseline': ai,
            'green_navigation': me,
            'paired_relative_summary': pairwise_summary,
            'green_relative_to_ai': official,
            'paired_scores_csv': paired_out,
        })
    else:
        result['warning'] = 'No valid paired matches; do not report a navigation score.'

    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
