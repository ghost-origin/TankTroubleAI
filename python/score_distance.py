# -*- coding: utf-8 -*-
"""轮次级运动度量：修复"多局并一局/越界巡航/跳变污染"后的口径。

规则（与用户判据对齐）：
- 轮次 = rounds.csv 记录 + 未记录的尾部 track（match 被 runner 停止）；
- 距离 = 逐帧累加，但：单步 > 120px 视为轮内跳变/假连接，剔除该步；
  me 越过世界边界 (0..570) 的帧不计入并给该轮打 escape=1 标记；
- 每轮输出实际时长（track t 跨度，上限 60s 标准局）。
"""
from __future__ import annotations
import argparse, csv, glob, math, os

MAP_PX = 570
JUMP_PX = 120.0


def round_dist(track_path, obj='me'):
    with open(track_path, newline='', encoding='utf-8-sig') as f:
        rows = []
        for r in csv.DictReader(f):
            try:
                rows.append((float(r['t']), float(r[obj + '_x']), float(r[obj + '_y'])))
            except (ValueError, KeyError):
                continue
    if not rows:
        return 0.0, 0.0, False, rows  # dist, seconds, escape
    dist = 0.0
    escape = False
    prev = None
    for t, x, y in rows:
        if not (0 <= x <= MAP_PX and 0 <= y <= MAP_PX):
            escape = True
            prev = (t, x, y)
            continue
        if prev is not None:
            dm = math.hypot(x - prev[1], y - prev[2])
            if dm <= JUMP_PX:
                dist += dm
        prev = (t, x, y)
    seconds = rows[-1][0] - rows[0][0]
    return dist, seconds, escape, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-root', required=True)
    a = ap.parse_args()

    rows_out = []
    foe_rows = []
    for m in sorted(glob.glob(os.path.join(a.data_root, 'match_*'))):
        recs = {}
        rf = os.path.join(m, 'rounds.csv')
        if os.path.exists(rf):
            with open(rf, newline='', encoding='utf-8-sig') as f:
                for r in csv.DictReader(f):
                    recs[r['round_id']] = r
        match_total = 0.0
        foe_total = 0.0
        cnt = 0
        for tf in sorted(glob.glob(os.path.join(m, 'track_*.csv'))):
            base = os.path.basename(tf)
            dist, sec, escape, _ = round_dist(tf)
            fdist, fsec, fesc, _ = round_dist(tf, 'foe')
            rec = recs.get(base)
            reason = rec['reason'] if rec else '(no-record)'
            match_total += dist
            foe_total += fdist
            cnt += 1
            rows_out.append((os.path.basename(m), base, round(dist, 1),
                             round(sec, 1), int(escape), reason[:44]))
            foe_rows.append((os.path.basename(m), base, round(fdist, 1), round(fsec, 1), int(fesc)))
        print('%-10s total=%.1f  (foe AI total=%.1f)' % (os.path.basename(m), match_total, foe_total))
        for r in rows_out[-cnt:]:
            print('    %-28s dist=%7.1f sec=%5.1f escape=%d reason=%s' % (r[1], r[2], r[3], r[4], r[5]))
        rows_out.append((os.path.basename(m), '', match_total, '', '', ''))
    # 汇总:有效轮(60s内, 无escape)的平均距离
    valid = [r for r in rows_out if r[2] != '' and r[4] == 0]
    if valid:
        avg = sum(r[2] for r in valid) / len(valid)
        print('\nvalid rounds (no escape): %d, avg dist = %.1f px' % (len(valid), avg))
    else:
        print('\nno valid rounds')
    # 游戏内置 AI(foe) 基准:同口径
    fvalid = [r for r in foe_rows if r[4] == 0]
    if fvalid:
        favg = sum(r[2] for r in fvalid) / len(fvalid)
        print('foe AI reference (same rounds): %.1f px over %d rounds' % (favg, len(fvalid)))


if __name__ == '__main__':
    main()
