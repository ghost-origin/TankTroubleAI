# -*- coding: utf-8 -*-
"""Batch headless evaluator using the repository's existing run_match.js + plot_tracks.py."""
from __future__ import annotations
import argparse, glob, os, shutil, subprocess, sys, time


def newest(pattern, before=None):
    fs=glob.glob(pattern)
    if before is not None: fs=[f for f in fs if os.path.getmtime(f)>=before]
    return max(fs,key=os.path.getmtime) if fs else None

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--repo',default=os.getcwd()); ap.add_argument('--matches',type=int,default=10)
    ap.add_argument('--port',type=int,default=8766); ap.add_argument('--duration',type=float,default=90)
    ap.add_argument('--out',default='nav_headless_results')
    a=ap.parse_args(); repo=os.path.abspath(a.repo); out=os.path.abspath(os.path.join(repo,a.out)); os.makedirs(out,exist_ok=True)
    jsdom=os.environ.get('JSDOM_PATH')
    if not jsdom or not os.path.exists(os.path.join(jsdom,'node_modules','jsdom')):
        raise SystemExit('JSDOM_PATH missing. Install once: npm install jsdom --prefix "%TEMP%\\tt3jsdom" then set JSDOM_PATH=%TEMP%\\tt3jsdom')
    bot=os.path.join(repo,'python','navigation_bot.py'); runner=os.path.join(repo,'data log','run_match.js'); plot=os.path.join(repo,'data log','plot_tracks.py')
    for f in (bot,runner,plot):
        if not os.path.exists(f): raise SystemExit('missing: '+f)
    for i in range(a.matches):
        rd=os.path.join(out,'match_%03d'%(i+1)); os.makedirs(rd,exist_ok=True)
        t0=time.time()
        bp=subprocess.Popen([sys.executable,bot,'--port',str(a.port),'--out-dir',rd,'--repo',repo],cwd=repo)
        time.sleep(0.8)
        try:
            cp=subprocess.run(['node',runner,str(a.port),str(a.duration)],cwd=repo,env=os.environ.copy(),timeout=180)
            print('match',i+1,'runner rc=',cp.returncode)
        finally:
            bp.terminate()
            try: bp.wait(timeout=3)
            except subprocess.TimeoutExpired: bp.kill()
        track=newest(os.path.join(rd,'track_*.csv'),t0-1)
        if track:
            subprocess.run([sys.executable,plot,track],cwd=repo,check=False)
        time.sleep(0.5)
    # score all generated matches, existing AI is the foe in the same match.
    score=os.path.join(repo,'python','score_navigation.py')
    polys=os.path.join(repo,'data log','tile_polys.json')
    if os.path.exists(score) and os.path.exists(polys):
        subprocess.run([sys.executable,score,'--data-root',out,'--polys',polys,'--out',os.path.join(out,'scores.csv')],cwd=repo,check=False)
    print('done:',out)
if __name__=='__main__': main()
