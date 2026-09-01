# Long-Range Navigation Arena

This fixture stays below the tactical layer. It receives fixed start and goal
coordinates, then evaluates the existing A* output, clearance refinement, local
Bezier trajectory and swept-rectangle feasibility check.

## Run

Use `test/run_navigation_arena_long.bat`.

The fixture uses `maze=0`, `seed=1001` and 20 deterministic segments. Every
segment is selected by actual A* route length, with a hard lower bound of 400px:

- 400-600px: 45%
- 600-900px: 40%
- 900px+: 15%

The generator fails instead of silently substituting a shorter route when it
cannot meet the lower bound.

## Separation of concerns

The fixture does not generate enemies, tactical candidates, utility scores or
control changes. It measures only the route from an already selected target:

`A* -> LOS path -> clearance refinement -> Bezier -> swept rectangle -> existing follower`

`strict_plan_success` means the candidate passed minimum-radius and swept
rectangle checks. Diagnostic fallback output is retained only to expose what the
existing follower does after a rejected plan; it is never counted as a feasible
strict trajectory.

## First long-route baseline

With the current V2 footprint planner, seed 1001 produced 20 routes between
411px and 1205px (mean 655px). The strict planner accepted 0/20. The first
diagnosis shows several required consecutive turns are only 28-40px apart, while
the hard minimum turn radius is 33px. These cases need a local multi-corner
trajectory search or a manoeuvre model, not weaker collision checking.
