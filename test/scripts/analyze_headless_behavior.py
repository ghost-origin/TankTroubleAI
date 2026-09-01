"""Diagnose navigation stalls and plan churn from headless match CSV logs."""

from __future__ import annotations

import argparse
import ast
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt


SPIN_ANGULAR_RATE_RAD_S = 0.35
SPIN_SPEED_PX_S = 15.0
RUSH_SPEED_PX_S = 95.0
PLAN_SWITCH_DISTANCE_PX = 35.0


def wrap_angle(delta: float) -> float:
    return (delta + math.pi) % (2.0 * math.pi) - math.pi


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def contiguous_duration(samples: list[dict[str, float]], predicate) -> tuple[float, float]:
    current = 0.0
    longest = 0.0
    total = 0.0
    for sample in samples:
        dt = sample["dt"]
        if predicate(sample):
            current += dt
            total += dt
            longest = max(longest, current)
        else:
            current = 0.0
    return total, longest


def analyse_track(path: Path) -> dict[str, float | str]:
    rows = read_csv(path)
    points = []
    for row in rows:
        try:
            points.append({key: float(row[key]) for key in ("t", "me_x", "me_y", "me_angle")})
        except (KeyError, TypeError, ValueError):
            continue
    points.sort(key=lambda row: row["t"])
    samples = []
    for previous, current in zip(points, points[1:]):
        dt = current["t"] - previous["t"]
        if dt <= 0.0 or dt > 1.0:
            continue
        distance = math.hypot(current["me_x"] - previous["me_x"], current["me_y"] - previous["me_y"])
        samples.append({
            "t": current["t"],
            "dt": dt,
            "speed": distance / dt,
            "angular_rate": abs(wrap_angle(current["me_angle"] - previous["me_angle"])) / dt,
        })
    if not samples:
        return {"track": str(path), "samples": 0}
    spin_total, spin_longest = contiguous_duration(
        samples,
        lambda row: row["angular_rate"] >= SPIN_ANGULAR_RATE_RAD_S and row["speed"] <= SPIN_SPEED_PX_S,
    )
    start_t = samples[0]["t"]
    opening = [row for row in samples if row["t"] <= start_t + 6.0]
    rushing = [row for row in opening if row["speed"] >= RUSH_SPEED_PX_S]
    return {
        "track": str(path),
        "samples": len(samples),
        "duration_s": round(sum(row["dt"] for row in samples), 3),
        "mean_speed_px_s": round(sum(row["speed"] * row["dt"] for row in samples) / sum(row["dt"] for row in samples), 3),
        "max_speed_px_s": round(max(row["speed"] for row in samples), 3),
        "max_angular_rate_rad_s": round(max(row["angular_rate"] for row in samples), 3),
        "spin_total_s": round(spin_total, 3),
        "spin_longest_s": round(spin_longest, 3),
        "opening_rush_ratio": round(len(rushing) / len(opening), 3) if opening else 0.0,
        "opening_max_speed_px_s": round(max((row["speed"] for row in opening), default=0.0), 3),
    }


def analyse_plans(path: Path) -> dict[str, float | int | str]:
    rows = read_csv(path)
    accepted = []
    contract_violations = 0
    unsafe_accepted = 0
    fallback_tube_invalid = 0
    for row in rows:
        success = row.get("success") == "1"
        accepted_flag = row.get("accepted") == "1"
        validated = row.get("path_validated", "0") == "1"
        reason = row.get("reason", "")
        target_type = row.get("target_type", "")
        if reason == "fallback_tube_invalid":
            fallback_tube_invalid += 1
        if success and not validated:
            contract_violations += 1
        if accepted_flag and not validated:
            unsafe_accepted += 1
        if not success or not accepted_flag:
            continue
        try:
            target = (float(row["target_x"]), float(row["target_y"]))
            t = float(row["t"])
        except (KeyError, TypeError, ValueError):
            continue
        accepted.append((t, target, target_type, reason))
    accepted.sort(key=lambda item: item[0])
    switches = 0
    fallback = 0
    for (_, previous, _, _), (_, current, target_type, reason) in zip(accepted, accepted[1:]):
        if math.dist(previous, current) >= PLAN_SWITCH_DISTANCE_PX:
            switches += 1
        if target_type == "fallback" or reason == "fallback_chase":
            fallback += 1
    if accepted and (accepted[0][2] == "fallback" or accepted[0][3] == "fallback_chase"):
        fallback += 1
    duration = accepted[-1][0] - accepted[0][0] if len(accepted) > 1 else 0.0
    return {
        "plans": str(path),
        "accepted_plans": len(accepted),
        "plan_window_s": round(duration, 3),
        "switches": switches,
        "switches_per_10s": round(switches * 10.0 / duration, 3) if duration else 0.0,
        "fallback_plans": fallback,
        "fallback_ratio": round(fallback / len(accepted), 3) if accepted else 0.0,
        "contract_violations": contract_violations,
        "unsafe_accepted_plans": unsafe_accepted,
        "fallback_tube_invalid": fallback_tube_invalid,
    }


def recording_stamp(path: Path) -> str:
    return path.stem.rsplit("_", 1)[-1]


def write_summary(rows: list[dict], path: Path) -> None:
    fieldnames = sorted({key for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def make_plot(rows: list[dict], path: Path) -> None:
    labels = []
    for row in rows:
        track = Path(str(row["track"]))
        stamp = track.stem.replace("track_", "")
        labels.append("%s-%s" % (track.parent.name.replace("match_", "m"), stamp[-6:]))
    x = range(len(rows))
    fig, axes = plt.subplots(2, 2, figsize=(15, 8.5))
    axes = axes.flatten()
    axes[0].bar(x, [row.get("spin_total_s", 0.0) for row in rows], color="#2f6b8a")
    axes[0].set_title("Spin-like time")
    axes[0].set_ylabel("seconds")
    axes[1].bar(x, [row.get("opening_max_speed_px_s", 0.0) for row in rows], color="#c8862c")
    axes[1].axhline(RUSH_SPEED_PX_S, color="#303030", linestyle="--", linewidth=1, label="rush threshold")
    axes[1].set_title("Maximum speed in first six seconds")
    axes[1].set_ylabel("px/s")
    axes[1].legend(frameon=False)
    axes[2].bar(x, [row.get("fallback_tube_invalid", 0) for row in rows], color="#8c4b7a")
    axes[2].set_title("Rejected unsafe fallback paths")
    axes[2].set_ylabel("plans")
    axes[3].bar(x, [row.get("unsafe_accepted_plans", 0) for row in rows], color="#9b2f2f")
    axes[3].set_title("Unsafe accepted plans")
    axes[3].set_ylabel("plans")
    for ax in axes:
        ax.set_xticks(list(x), labels, rotation=60, ha="right", fontsize=7)
        ax.grid(axis="y", color="#dddddd", linewidth=0.6)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_dir", type=Path)
    args = parser.parse_args()
    result_dir = args.result_dir.resolve()
    tracks = sorted(result_dir.glob("match_*/track_*.csv"))
    plans_by_recording = {
        (plan.parent.name, recording_stamp(plan)): analyse_plans(plan)
        for plan in sorted(result_dir.glob("match_*/plans_*.csv"))
    }
    track_rows = []
    for track in tracks:
        row = analyse_track(track)
        plan_row = plans_by_recording.get((track.parent.name, recording_stamp(track)))
        if plan_row:
            for key in ("accepted_plans", "fallback_plans", "switches", "switches_per_10s",
                        "contract_violations", "unsafe_accepted_plans", "fallback_tube_invalid"):
                row[key] = plan_row[key]
        track_rows.append(row)
    write_summary(track_rows, result_dir / "behavior_diagnosis.csv")
    make_plot(track_rows, result_dir / "behavior_diagnosis.png")
    aggregate = {
        "recording_segments": len(track_rows),
        "spin_segments": sum(1 for row in track_rows if float(row.get("spin_longest_s", 0.0)) >= 1.5),
        "total_spin_time_s": round(sum(float(row.get("spin_total_s", 0.0)) for row in track_rows), 3),
        "max_spin_run_s": round(max((float(row.get("spin_longest_s", 0.0)) for row in track_rows), default=0.0), 3),
        "high_speed_openings": sum(1 for row in track_rows if float(row.get("opening_rush_ratio", 0.0)) >= 0.25),
        "fallback_plan_ratio": round(
            sum(float(row.get("fallback_plans", 0.0)) for row in track_rows) /
            max(1.0, sum(float(row.get("accepted_plans", 0.0)) for row in track_rows)),
            3,
        ),
        "total_target_switches": sum(int(row.get("switches", 0)) for row in track_rows),
        "planner_contract_violations": sum(int(row.get("contract_violations", 0)) for row in track_rows),
        "unsafe_accepted_plans": sum(int(row.get("unsafe_accepted_plans", 0)) for row in track_rows),
        "fallback_tube_invalid": sum(int(row.get("fallback_tube_invalid", 0)) for row in track_rows),
    }
    (result_dir / "behavior_diagnosis.json").write_text(json.dumps(aggregate, indent=2), encoding="utf-8")
    print(json.dumps(aggregate, indent=2))


if __name__ == "__main__":
    main()
