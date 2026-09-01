# OODA Hierarchical Navigation Design

## Purpose

TankTroubleAI replans at a one-second OODA cadence. A target policy that treats
the enemy rear as a hard constraint can stall when the rear is behind a wall,
inside a dead end, or reachable only through a long detour. This design keeps
the existing navigation stack and adds a small tactical selection layer above
it: rear remains desirable, but is a soft preference rather than a requirement.

## Layer Responsibilities

| Layer | Responsibility | V1 status |
| --- | --- | --- |
| Strategic / Tactical OODA | Choose the next tactical target. | Implemented |
| Topological Guide | Choose corridors, portals, and junctions. | Deferred to V2 |
| A* Grid Planner | Find a concrete grid route to a selected point. | Reused unchanged |
| Local Turn Planner | Make the route executable with Bezier and swept footprint checks. | Reused unchanged |
| Follower | Execute the approved path. | Reused unchanged |

The ownership boundary is deliberate: OODA selects *where to go*; A* selects
*how to get there*; the local planner and follower must not replace the tactical
choice.

## Tactical V1

`tactical_v1` samples 360 degrees around the foe at the existing attack radii:

- rear: three rear sectors, with the largest tactical reward;
- flank: two side sectors, a useful alternative when the rear is blocked;
- front: three front sectors, allowed at low reward rather than forbidden;
- reposition: nearby safe intermediate points, penalized so they are fallback
  choices rather than a way to idle near the current position.

Every legal candidate is evaluated using the existing A* grid. Its utility is
represented as a cost in path-pixel equivalents:

```text
score = A* path length
      + target/direction switch cost
      + no-LOS cost
      + dead-end cost
      + reposition cost
      - tactical-sector reward
      - LOS reward
      - open-direction reward
```

The candidate with the lowest score is passed through the existing Virtual Tube
and swept-rectangle feasibility gate. No topology graph, skeleton, medial axis,
articulation-point analysis, or new local motion controller is introduced in
V1.

### Reachability and Safety

- A* supplies both reachability and map path length.
- LOS is checked against the real wall mask, with the foe endpoint ignored by a
  small tolerance as in the existing attack planner.
- A simple eight-ray sample on the current inflated A* grid counts available
  exit directions roughly 50 px from the target. One or zero exits receives a
  dead-end penalty; more than two receives a small open-space reward.
- A* paths are not rewritten. The final selected route still uses the existing
  LOS simplifier, Bezier path generation, and swept-footprint validation.

### Stability

The selector retains the old relative-direction penalty and adds a target
switch penalty when a new candidate is more than 35 px from the accepted target.
The online bot keeps its existing path-hold acceptance rule as a second guard.
This prevents a one-second OODA loop from discarding a useful route for a tiny
change in enemy pose.

## A/B Configuration

The online bot accepts:

```text
--tactical-mode tactical_v1
--tactical-mode rear_only
```

`rear_only` is the default baseline. It preserves the previous five rear-half-plane
candidate directions and rear staging penalty. `tactical_v1` remains available
for controlled A/B experiments.

## Logs and Acceptance Criteria

Each `plans_*.csv` row records tactical mode, selected target type, candidate
counts, A* reachable counts, rear reachable counts, reachable-candidate rate,
LOS, open directions, tactical score, and switch cost. The offline replay also
keeps the selected target, path length, planning time, safety check, and reason.

V1 is successful when repeated benchmarks show:

- reachable tactical target rate above 95%;
- a meaningful drop in stalls caused by an unreachable rear target;
- no material increase in target-switch frequency;
- no regression in existing Turn, Navigation, or Combat checks.

## Version Boundaries

V2 may add cached junction/corridor/portal nodes and adjacency-list distances
after V1 proves the soft tactical policy useful. V3 may consider skeletons,
medial axes, articulation points, corridor width, and dynamic edge costs. Those
features are explicitly outside this implementation.
