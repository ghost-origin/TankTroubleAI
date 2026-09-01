# -*- coding: utf-8 -*-
"""Batch headless navigation benchmark.

Default engine=auto:
- use the repository's existing jsdom runner when JSDOM_PATH is available;
- otherwise use run_match_chromium.py.

For navigation-only evaluation pass --navigation-only. This disables firing so
combat deaths do not dominate the path-planning benchmark.
"""
from __future__ import annotations
import argparse, glob, os, signal, socket, subprocess, sys, time


def pick_free_port():
    """Ephemeral loopback port; per-match use avoids lingering exclusive binds."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]
    finally:
        s.close()


def newest(pattern, before=None):
    fs=glob.glob(pattern)
    if before is not None:
        fs=[f for f in fs if os.path.getmtime(f)>=before]
    return max(fs,key=os.path.getmtime) if fs else None


def have_jsdom():
    p=os.environ.get('JSDOM_PATH')
    return bool(p and os.path.exists(os.path.join(p,'node_modules','jsdom')))


def kill_process_tree(proc):
    if proc.poll() is not None:
        return
    try:
        if os.name == 'nt':
            subprocess.run(['taskkill','/PID',str(proc.pid),'/T','/F'],
                           stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=False)
        else:
            os.killpg(proc.pid, signal.SIGKILL)
    except Exception:
        try: proc.kill()
        except Exception: pass


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--repo',default=os.getcwd())
    ap.add_argument('--matches',type=int,default=10)
    ap.add_argument('--port',type=int,default=0,
                    help='fixed WS port (0 = per-match ephemeral)')
    ap.add_argument('--duration',type=float,default=60,
                    help='standard round length in game-seconds (safety cap)')
    ap.add_argument('--out',default='nav_headless_results')
    ap.add_argument('--engine',choices=['auto','jsdom','chromium'],default='auto')
    ap.add_argument('--navigation-only',action='store_true')
    ap.add_argument('--tactical-mode',choices=['rear_only','tactical_v1','chase'],default='chase',
                    help='OODA target policy; chase targets foe position (default); rear_only is the legacy baseline')
    ap.add_argument('--prediction-horizon-s',type=float,default=0.0,
                    help='Kalman foe prediction horizon; use 0 for baseline A/B')
    ap.add_argument('--seed-base',type=int,default=0,
                    help='Chromium deterministic seed base; match i uses seed_base+i')
    ap.add_argument('--game-speed',type=float,default=1.0)
    a=ap.parse_args()

    repo=os.path.abspath(a.repo)
    out=os.path.abspath(os.path.join(repo,a.out)); os.makedirs(out,exist_ok=True)

    engine=a.engine
    if engine=='auto':
        engine='jsdom' if have_jsdom() else 'chromium'
    if engine=='jsdom' and not have_jsdom():
        raise SystemExit('jsdom engine requested but JSDOM_PATH/node_modules/jsdom is missing')
    print('engine:',engine,'navigation_only:',a.navigation_only,'tactical_mode:',a.tactical_mode,
          'prediction_horizon_s:',a.prediction_horizon_s,'matches:',a.matches,flush=True)

    bot=os.path.join(repo,'python','navigation_bot.py')
    jsrunner=os.path.join(repo,'data log','run_match.js')
    chr_runner=os.path.join(repo,'run_match_chromium.py')
    plot=os.path.join(repo,'data log','plot_tracks.py')
    score=os.path.join(repo,'python','score_navigation.py')
    for f in (bot,plot,score):
        if not os.path.exists(f): raise SystemExit('missing: '+f)

    for i in range(a.matches):
        rd=os.path.join(out,'match_%03d'%(i+1)); os.makedirs(rd,exist_ok=True)
        t0=time.time()
        port = a.port if a.port else pick_free_port()
        print('\n=== match %d/%d === (port %d)'%(i+1,a.matches,port),flush=True)
        bot_log=open(os.path.join(rd,'bot.log'),'w',encoding='utf-8')
        bp=subprocess.Popen([sys.executable,bot,'--port',str(port),'--out-dir',rd,'--repo',repo,
                             '--tactical-mode',a.tactical_mode,
                             '--prediction-horizon-s',str(a.prediction_horizon_s),
                             '--exit-on-round-end'],
                            cwd=repo,stdout=bot_log,stderr=subprocess.STDOUT)
        time.sleep(0.6)
        try:
            if engine=='jsdom':
                cmd=['node',jsrunner,str(port),str(a.duration)]
                if a.navigation_only: cmd.append('nav-only')
            else:
                cmd=[sys.executable,chr_runner,str(port),str(a.duration),'--repo',repo,
                     '--game-speed',str(a.game_speed)]
                if a.navigation_only: cmd.append('--no-firing')
                if a.seed_base: cmd += ['--seed',str(a.seed_base+i+1)]

            runner_log=os.path.join(rd,'runner.log')
            with open(runner_log,'w',encoding='utf-8') as rf:
                popen_kw=dict(cwd=repo,env=os.environ.copy(),stdout=rf,stderr=subprocess.STDOUT)
                if os.name != 'nt': popen_kw['start_new_session']=True
                rp=subprocess.Popen(cmd,**popen_kw)
                deadline = time.time() + max(35, int(a.duration) + 20)
                rc = None
                flag_path = os.path.join(rd, 'round_end.flag')
                try:
                    while time.time() < deadline:
                        # bot 判定的局终 → 主动退出 → 该场结束（一次比赛=一局）
                        if bp.poll() is not None:
                            if os.path.exists(flag_path):
                                with open(flag_path, encoding='utf-8') as ff:
                                    print('bot ended round: %s' % ff.read().strip(), flush=True)
                            else:
                                print('WARN: bot exited without round_end.flag', flush=True)
                            try:
                                rc = rp.wait(timeout=6)
                            except subprocess.TimeoutExpired:
                                kill_process_tree(rp)
                                rc = 124
                            break
                        try:
                            rc = rp.wait(timeout=0.5)
                            break
                        except subprocess.TimeoutExpired:
                            continue
                except subprocess.TimeoutExpired:
                    rc = 124
                    kill_process_tree(rp)
                    try: rp.wait(timeout=3)
                    except Exception: pass
                    print('runner hard-timeout; process tree killed',flush=True)
                if rc is None:
                    rc = 124
                    kill_process_tree(rp)
                    try: rp.wait(timeout=3)
                    except Exception: pass
                    print('runner hard-timeout; process tree killed',flush=True)
            print('runner rc=',rc,flush=True)
        finally:
            bp.terminate()
            try: bp.wait(timeout=3)
            except subprocess.TimeoutExpired: bp.kill()
            bot_log.close()

        track=newest(os.path.join(rd,'track_*.csv'),t0-1)
        if track:
            with open(os.path.join(rd,'plot.log'),'w',encoding='utf-8') as pf:
                subprocess.run([sys.executable,plot,track],cwd=repo,
                               stdout=pf,stderr=subprocess.STDOUT,check=False)
        time.sleep(0.2)

    polys=os.path.join(repo,'data log','tile_polys.json')
    analysis=os.path.join(out,'analysis.json')
    with open(analysis,'w',encoding='utf-8') as af:
        subprocess.run([sys.executable,score,'--data-root',out,'--polys',polys,
                        '--out',os.path.join(out,'scores.csv'),
                        '--skipped-out',os.path.join(out,'scores_skipped.csv')],
                       cwd=repo,stdout=af,stderr=subprocess.STDOUT,check=False)
    print('\ndone:',out,flush=True)
    print('analysis:',analysis,flush=True)


if __name__=='__main__':
    main()
