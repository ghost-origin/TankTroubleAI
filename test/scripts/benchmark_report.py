# -*- coding: utf-8 -*-
"""Merge benchmark outputs into one report while keeping Turn and Navigation scores separate."""
import argparse,json,os

def load(p):
    with open(p,encoding='utf-8') as f:return json.load(f)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',default='test/results/current');a=ap.parse_args()
    turn=load(os.path.join(a.root,'turn','turn_summary.json'))
    arena=load(os.path.join(a.root,'arena','arena_summary.json'))
    long_path=os.path.join(a.root,'arena_long','arena_summary.json')
    long_arena=load(long_path) if os.path.exists(long_path) else None
    report={
      'test_pyramid': ['Turn Benchmark','Navigation Arena','Combat Benchmark'],
      'turn_benchmark':turn,
      'navigation_arena':arena,
      'important':'Turn Benchmark is an independent regression metric and is NOT folded into the overall Navigation Arena score.'
    }
    if long_arena is not None:
      report['long_range_navigation_arena']=long_arena
    with open(os.path.join(a.root,'benchmark_report.json'),'w',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2)
    lines=[
      '# Navigation Benchmark Report','',
      '## Turn Benchmark (independent)',
      f"- Cases: {turn['n_cases']}",
      f"- Success rate: {turn['success_rate']:.3f}",
      f"- Collisions: {turn['n_collision']}",
      f"- Median overshoot: {turn['median_overshoot_deg']:.2f} deg",
      f"- Median exit heading error: {turn['median_heading_exit_error_deg']:.2f} deg",
      f"- Median max cross-track error: {turn['median_max_cross_track_px']:.2f} px",'',
      '## Navigation Arena',
      f"- Goals completed: {arena['goals_completed']}/{arena['goals_attempted']}",
      f"- Goal success rate: {arena['goal_success_rate']:.3f}",
      f"- Strict Virtual Tube plan success rate: {arena['strict_plan_success_rate']:.3f}",
      f"- Diagnostic fallback segments: {arena.get('diagnostic_fallback_segments', 0)}",
      f"- Diagnostic fallback collisions: {arena.get('diagnostic_fallback_collisions', arena['collisions'])}",
      f"- Collisions: {arena['collisions']}",
      f"- Total distance: {arena['total_distance_px']:.1f} px",
      f"- Average speed: {arena['average_speed_px_s']:.1f} px/s",
      f"- Moving speed: {arena['moving_speed_px_s']:.1f} px/s",
      f"- Map path efficiency (successful goals only): {arena['map_path_efficiency']:.3f}",'',
      'Turn Benchmark remains separate from the overall navigation benchmark.'
    ]
    if long_arena is not None:
      nominal=long_arena.get('nominal_a_star_path_px',{})
      lines += [
        '', '## Long-Range Navigation Arena (400px+ A* routes)',
        f"- Goals completed: {long_arena['goals_completed']}/{long_arena['goals_attempted']}",
        f"- Strict plan success rate: {long_arena['strict_plan_success_rate']:.3f}",
        f"- A* route range: {nominal.get('min', 0.0):.1f}-{nominal.get('max', 0.0):.1f} px",
        f"- Mean A* route length: {nominal.get('mean', 0.0):.1f} px",
        f"- Collisions: {long_arena['collisions']}",
      ]
    with open(os.path.join(a.root,'benchmark_report.md'),'w',encoding='utf-8') as f:f.write('\n'.join(lines)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
