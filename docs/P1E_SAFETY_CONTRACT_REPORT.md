# P1E Safety Contract Report

Date: 2026-09-01

## Scope

P1E only closes the planner safety contract bug:

```text
PlanResult.success = True
=> path is non-empty
=> path_validated = True
=> the path passed the final execution safety gate
```

This version intentionally does not tune conservative parameters. P4/topology work
remains off. A*, Virtual Tube V2 footprint parameters, minimum turn radius,
clearance, follower behavior, combat bridge and Tactical V1/P4 policy are not
changed as part of this fix.

## Code Changes

- `python/navigation_mvp.py`
  - Added contract fields to `PlanResult`:
    - `path_source`
    - `path_validated`
    - `validation_reason`
    - `raw_path_points`
    - `executed_path_points`
  - Added a fail-closed `PlanResult.__post_init__` guard. Any successful result
    without validation is automatically converted to a failed result with an
    empty executable path.
  - Changed fallback chase so `build_tube(...) is None` returns
    `fallback_tube_invalid` with `path=[]`; raw A* is never returned as an
    executable fallback.
  - Marked all legitimate success paths with explicit validation metadata.

- `python/navigation_bot.py`
  - Bot accepts a new plan only when:
    - `pr.success`
    - `pr.path_validated`
    - `pr.path` is non-empty
  - `plans_*.csv` now includes validation metadata:
    - `path_source`
    - `path_validated`
    - `validation_reason`
    - `raw_path_points`
    - `executed_path_points`

- `test/test_p1e_safety_contract.py`
  - Added focused regression tests for:
    - contract violation fail-closed behavior
    - fallback tube failure not returning raw A*
    - validated success metadata

- `test/scripts/analyze_headless_behavior.py`
  - Added P1E diagnostics:
    - planner contract violations
    - unsafe accepted plans
    - fallback tube invalid count
  - Diagnostic plot now shows spin time, opening rush speed, rejected unsafe
    fallback paths and unsafe accepted plans.

- `data log/plot_tracks.py`
  - Empty track CSVs now still produce a PNG with the maze/empty-frame state,
    so headless validation has one image per recorded segment.

## Regression Tests

### Unit and Syntax

```text
python -m unittest test.test_p1e_safety_contract test.test_ooda_tactical_v1
```

Result:

```text
Ran 6 tests
OK
```

Syntax check:

```text
python -m py_compile python/navigation_mvp.py python/navigation_bot.py
python -m py_compile test/scripts/analyze_headless_behavior.py data log/plot_tracks.py
```

Result: pass.

### Turn Benchmark

Output directory:

```text
test/results/p1e_contract/turn
```

Result:

```text
cases: 18
success: 18
collisions: 0
success_rate: 1.000
median_overshoot_deg: 0.72
median_heading_exit_error_deg: 0.72
median_max_cross_track_px: 1.20
```

Interpretation: unchanged from the Virtual Tube V2 result. P1E did not alter
turning physics, footprint parameters or controller behavior.

### Navigation Arena

Output directory:

```text
test/results/p1e_contract/arena
```

Result:

```text
goals_completed: 4/20
strict_plan_successes: 3/20
diagnostic_fallback_segments: 17
diagnostic_fallback_collisions: 15
collisions: 16
average_speed_px_s: 97.9
```

Interpretation: unchanged from the prior Virtual Tube V2 result. This confirms
P1E did not tune clearance, minimum turning radius, the coarse grid, or
conservatism knobs.

## Headless Verification

Command profile:

```text
engine: jsdom
matches: 10
duration: 20s
tactical_mode: rear_only
navigation_only: true
```

Output directory:

```text
test/results/p1e_contract/headless_rear_only_10
```

Runner result:

```text
match_001: runner timeout, CSV still emitted
match_002 - match_010: runner rc=0
```

CSV/image completeness:

```text
track csv files: 22
plans csv files: 22
maze csv files: 19
individual track png files: 22
diagnostic png: yes
contact sheet png: yes
```

Three track segments had no maze CSV because they were empty/very short bridge
segments before a valid maze snapshot was received. They are retained and plotted
as zero-frame diagnostics instead of being silently dropped.

Plans CSV safety audit:

```text
total plan rows: 469
success rows: 208
accepted rows: 200
success=1 and path_validated!=1: 0
accepted=1 and path_validated!=1: 0
fallback_tube_invalid: 261
accepted fallback_chase: 15
```

The key P1E acceptance condition passed:

```text
unsafe_fallback_executed = 0
```

`fallback_tube_invalid = 261` is expected and useful: these are raw fallback
paths that the old logic could have leaked into the follower, but P1E now rejects
before execution.

Behavior diagnostic summary:

```text
recording_segments: 22
spin_segments: 4
total_spin_time_s: 112.52
max_spin_run_s: 3.78
high_speed_openings: 1
fallback_plan_ratio: 0.075
total_target_switches: 55
planner_contract_violations: 0
unsafe_accepted_plans: 0
fallback_tube_invalid: 261
```

## Visual Inspection

Reviewed:

- `behavior_diagnosis.png`
- `track_contact_sheet.png`
- key long tracks:
  - `match_003/track_20260901_211320.png`
  - `match_005/track_20260901_211350.png`
  - `match_006/track_20260901_211423.png`

Findings:

- The P1E safety contract is visually consistent with the CSV audit. Rejected
  fallback paths do not appear as raw A* hard-corner execution paths.
- Several zero-frame and very short segments are jsdom round-boundary artifacts.
  They are now visible in the contact sheet and no longer break plotting.
- `match_003` shows localized movement in the lower maze region, then a stop
  near the same corridor area. This matches many `fallback_tube_invalid` rejects:
  the bot refuses unsafe fallback instead of driving the raw path.
- `match_005` shows broad traversal through the map without obvious raw 90-degree
  fallback snapping, but it still contains repeated corridor revisits. This is a
  remaining rear-only tactical/coverage issue, not a P1E leak.
- `match_006` is the longest moving segment. It includes valid fallback execution
  where tube validation passed, but still has large loops and repeated passages.
  The CSV confirms all accepted fallback rows were validated.
- The diagnostic plot's unsafe accepted plan panel is flat at zero.

## Conclusion

P1E is complete for its intended safety contract:

```text
success=True without final validation: 0
accepted unvalidated plans: 0
unsafe fallback executed: 0
```

The remaining poor behavior is now cleaner to diagnose. It is dominated by
`rear_only` target availability, long waits after strict footprint rejection, and
old-path reuse/looping in some maze layouts. Those should be handled in a later
tactical or local-window validation phase, not by loosening P1E.

## Next Step

Run the same 10-20 headless match suite with `tactical_v1` enabled and compare:

```text
rear_only vs tactical_v1
fallback_tube_invalid
unsafe accepted plans
spin time
target switches
map efficiency
```

Do not change clearance or footprint conservatism until the A/B confirms whether
target selection or local execution validation is the dominant remaining blocker.
