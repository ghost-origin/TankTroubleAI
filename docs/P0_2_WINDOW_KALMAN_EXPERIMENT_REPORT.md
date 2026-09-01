# P0-2 Local Window + Kalman Experiment Report

Date: 2026-09-01

## Status

This is an experimental P0-2 engineering build, not a stable navigation release.
It preserves the P1E validated-path contract, keeps P4 disabled, and does not
change footprint size, wall clearance, minimum turn radius, A*, or tactical
scoring.

## Implemented

### Receding-horizon execution

- Global A* remains responsible for route direction.
- Only a near-term prefix is refined and footprint-validated.
- Validated window ladder: 160, 120, 95, 70, then emergency 40 px.
- The 40 px window is not accepted by centreline distance alone. The complete
  swept tank rectangle must remain collision-free.
- The local replan threshold scales with the accepted window. A 40 px window
  replans near 14 px remaining instead of immediately tripping the normal 45 px
  threshold.
- A 45 degree start-heading gate rejects discontinuous window handoffs.
- The online bot refreshes a local window from remaining distance rather than
  waiting only for the next 1 Hz tactical OODA tick.
- Final windows marked `window_goal_reached=1` are allowed to finish without
  redundant per-frame replanning.

New `plans.csv` fields include global path length, executed window length,
selected window target/lookahead lengths, and final-goal coverage.

### Kalman prediction experiment

- Added a four-state constant-velocity filter: `[x, y, vx, vy]`.
- Runtime can forecast the observed foe for a configurable short horizon.
- The forecast changes only the foe point supplied to the existing `rear_only`
  planner. It does not bypass path refinement or footprint validation.
- Logs preserve observed foe position, predicted position, horizon, and the
  forecast polyline.
- Default horizon is `0.0` because the short A/B did not establish a net benefit.

Example opt-in:

```text
--prediction-horizon-s 0.10
```

## Tests

### Unit and contract tests

```text
15 tests passed
```

Coverage includes:

- prefix interpolation across waypoints;
- long route returning a local validated window;
- full short route preservation;
- far invalid geometry not rejecting the current safe window;
- current-window collision failing closed;
- safe emergency 40 px window selection;
- start-heading discontinuity rejection;
- P1E fallback and planner contract regressions;
- constant-velocity Kalman forecast and forecast-path generation.

### Turn Benchmark

```text
18/18 success
0 collisions
median overshoot: 0.718 deg
median exit heading error: 0.718 deg
median max cross-track: 1.203 px
```

No turn regression was observed.

### Navigation Arena default

P1E baseline:

```text
strict initial plans: 3/20
goals completed: 4/20
collisions: 16
diagnostic fallback collisions: 15/17
```

P0-2 experimental result:

```text
strict initial plans: 17/20
strict initial rate: 85%
sustained window successes: 28/55
sustained executable-window coverage: 50.9%
goals completed: 2/20
collisions: 8
diagnostic fallback collisions: 3/3
strict-window collisions: 5
complex goals completed: 0/13
```

Interpretation: the window ladder substantially improves initial executable
coverage and reduces total collision exposure, but window-to-window continuity
still fails near complex bends. The Phase 1 release targets of 60% sustained
coverage and 10/20 goals are not met.

### Navigation Arena long

P1E baseline:

```text
strict initial plans: 0/20
goals completed: 0/20
collisions: 20
```

P0-2 experimental result:

```text
strict initial plans: 18/20
sustained window successes: 35/72
sustained executable-window coverage: 48.6%
goals completed: 0/20
collisions: 3
diagnostic fallback collisions: 2
strict-window collisions: 1
```

Interpretation: the long-route coverage and collision gates improve materially,
but no full long route completes. The next blocker is continuity across windows,
not whole-route reachability.

## Kalman Headless A/B

All three runs used `rear_only`; P4 remained off.

| Horizon | Plan rows | Success | Accepted | Fallback invalid | Mean / max forecast offset | Unsafe accepted |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.00 s | 3 | 2 | 2 | 1 | 0 / 0 px | 0 |
| 0.25 s | 3 | 0 | 0 | 3 | 9.98 / 29.94 px | 0 |
| 0.10 s | 17 | 5 | 3 | 12 | 2.22 / 11.40 px | 0 |

The samples are too short and differ in round fragmentation, so these counts are
not a performance ranking. They do show that prediction never bypasses P1E. A
0.25 s horizon was too aggressive in this sample; 0.10 s remained feasible on
some plans but did not establish a stable benefit. Prediction therefore remains
opt-in and defaults to off.

## Visual Inspection

- All three diagnostic plots keep unsafe accepted plans flat at zero.
- No spin interval is visible, but recordings are too short to use this as a
  positive behavioral claim.
- The 0.10 s track images contain only 2 frames and 19 frames. The longer fragment
  shows foe movement while the controlled tank remains nearly stationary.
- The 5k-12k px/s opening bars are round-boundary/two-frame speed artifacts, not
  real vehicle acceleration.
- No raw A* hard-corner execution leak is visible.

## Decision

Keep the following:

- P1E fail-closed contract;
- receding-window metadata and local-distance trigger;
- 40 px emergency window as an experimental final fallback;
- Kalman implementation and CSV diagnostics with default horizon 0.

Do not promote this build as stable. The next implementation should preserve an
overlapping validated guidance segment between windows, or carry the previous
window tangent/route suffix into the next refinement. That directly addresses
the observed complex-bend handoff failures without loosening footprint safety.
