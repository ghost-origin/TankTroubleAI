# TankTrouble Navigation MVP - minimal online integration

Add only these three files under repository `python/`:
- `navigation_mvp.py`
- `navigation_bot.py`
- `score_navigation.py`

Add `run_navigation_headless.py` at repository root (or anywhere; pass `--repo`).
No changes are required to `src/ai/ai-bridge.js`, `data log/run_match.js`, or `data log/plot_tracks.py`.

## WebSocket chain
`run_match.js -> ai-bridge.js -> navigation_bot.py -> key action -> ai-bridge.js -> game`

The bot writes compatible `track_*.csv` and `maze_*.csv`, plus `plans_*.csv`.
The plan log includes the selected tactical target type, candidate reachability,
LOS, local exit count, score, and switch cost.

## OODA Tactical A/B

`rear_only` is the default baseline. `tactical_v1` evaluates rear, flank, front,
and reposition candidates and remains an explicit experiment:

```text
python run_navigation_headless.py --matches 10 --tactical-mode tactical_v1
python run_navigation_headless.py --matches 10 --tactical-mode rear_only
```

For direct online runs, pass the same value to `python/navigation_bot.py` with
`--tactical-mode`.

## One-time jsdom requirement
The existing repository headless runner requires jsdom:
```
npm install jsdom --prefix "%TEMP%\tt3jsdom"
set JSDOM_PATH=%TEMP%\tt3jsdom
```
This is the same prerequisite documented by the repository archive README.

## Run 10 headless matches
From repo root:
```
python run_navigation_headless.py --matches 10
```
Each match is plotted automatically by the existing `data log/plot_tracks.py`.
