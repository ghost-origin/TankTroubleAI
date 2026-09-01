# -*- coding: utf-8 -*-
"""Online navigation bot for TankTroubleAI.

Minimal integration: reuse existing src/ai/ai-bridge.js and data log/run_match.js.
This process replaces record_ws.py on the same WebSocket port: it receives the same
state JSON, sends key actions back, and writes compatible track_*.csv/maze_*.csv.
"""
from __future__ import annotations
import argparse, csv, json, math, os, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
# When copied into repository python/, ROOT is repo root; when run standalone allow --repo.

from navigation_mvp import build_maps, load_polys, plan

CSV_COLUMNS = [
    't','me_x','me_y','me_angle','me_vx','me_vy',
    'foe_x','foe_y','foe_angle','foe_vx','foe_vy','n_bullets','n_powerups'
]
MAP_ORIGIN_X = 197.5
MAP_ORIGIN_Y = 31.0
MAZE_SIZE = 10
WAYPOINT_REACHED_PX = 13.0
TURN_ONLY_RAD = math.radians(18.0)
TURN_DEADBAND_RAD = math.radians(5.0)
OODA_PERIOD_S = 1.0


def wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi

class Bot:
    def __init__(self, port, out_dir, repo_root):
        sys.path.insert(0, os.path.join(repo_root, 'data log'))
        from ws_server import WSServer
        self.WSServer = WSServer
        self.port = port; self.out_dir = out_dir; self.repo_root = repo_root
        os.makedirs(out_dir, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.track_path = os.path.join(out_dir, 'track_%s.csv' % stamp)
        self.maze_path = os.path.join(out_dir, 'maze_%s.csv' % stamp)
        self.plan_path = os.path.join(out_dir, 'plans_%s.csv' % stamp)
        self.f = open(self.track_path,'w',newline='',encoding='utf-8')
        self.w = csv.DictWriter(self.f, fieldnames=CSV_COLUMNS); self.w.writeheader()
        self.pf = open(self.plan_path,'w',newline='',encoding='utf-8')
        self.pw = csv.DictWriter(self.pf, fieldnames=['t','success','relative_deg','target_x','target_y','path_length_px','smoothness_rad','planning_ms','reason','path_points'])
        self.pw.writeheader()
        self.polys = None; self.raw_wall = None; self.blocked = None
        self.last_plan_t = -1e9; self.path=[]; self.wp_idx=0; self.client=None
        self.rows=0; self.maze_written=False
        print('track:',self.track_path, flush=True); print('maze:',self.maze_path,flush=True)

    def map_coords(self,o):
        if not o: return None
        return dict(x=float(o['x'])-MAP_ORIGIN_X, y=float(o['y'])-MAP_ORIGIN_Y,
                    angle=float(o.get('angle',0.0)), vx=float(o.get('vx',0.0)), vy=float(o.get('vy',0.0)))

    def on_client(self,c):
        self.client=c; c.on_message=lambda txt:self.on_message(c,txt)
        c.on_close=lambda _c:self.close()
        print('connected:', c.addr, flush=True)

    def save_map(self,m):
        grid=m.get('grid') or []
        if len(grid)<MAZE_SIZE: return
        maze=[[int(grid[y][x]) for x in range(MAZE_SIZE)] for y in range(MAZE_SIZE)]
        with open(self.maze_path,'w',encoding='utf-8') as f:
            for row in maze: f.write(','.join(map(str,row))+'\n')
        if m.get('polys'):
            self.polys=m['polys']
        elif self.polys is None:
            self.polys=load_polys(os.path.join(self.repo_root,'data log','tile_polys.json'))
        self.raw_wall, _, self.blocked = build_maps(maze,self.polys,8,5)
        self.maze_written=True

    def record(self,t,me,foe,msg):
        self.w.writerow({'t':round(t,2),'me_x':round(me['x'],1),'me_y':round(me['y'],1),'me_angle':round(me['angle'],3),
            'me_vx':round(me['vx'],1),'me_vy':round(me['vy'],1),'foe_x':round(foe['x'],1),'foe_y':round(foe['y'],1),
            'foe_angle':round(foe['angle'],3),'foe_vx':round(foe['vx'],1),'foe_vy':round(foe['vy'],1),
            'n_bullets':len(msg.get('bullets') or []),'n_powerups':len(msg.get('powerups') or [])})
        self.f.flush(); self.rows+=1

    def replan(self,t,me,foe):
        if self.raw_wall is None or self.blocked is None: return
        pr=plan((me['x'],me['y']),(foe['x'],foe['y']),foe['angle'],self.raw_wall,self.blocked)
        self.last_plan_t=t
        if pr.success:
            self.path=pr.path; self.wp_idx=1 if len(self.path)>1 else 0
        else:
            self.path=[]; self.wp_idx=0
        self.pw.writerow({'t':round(t,3),'success':int(pr.success),'relative_deg':'' if pr.relative_deg is None else pr.relative_deg,
            'target_x':'' if pr.target is None else round(pr.target[0],2),'target_y':'' if pr.target is None else round(pr.target[1],2),
            'path_length_px':round(pr.path_length,3),'smoothness_rad':round(pr.smoothness_rad,4),'planning_ms':round(pr.planning_ms,3),
            'reason':pr.reason,'path_points':json.dumps([[round(x,1),round(y,1)] for x,y in pr.path])})
        self.pf.flush()

    def action(self,me):
        k={'up':0,'down':0,'left':0,'right':0}
        if not self.path or self.wp_idx>=len(self.path): return {'keys':k,'fire':0}
        while self.wp_idx<len(self.path):
            tx,ty=self.path[self.wp_idx]
            if math.hypot(tx-me['x'],ty-me['y'])<=WAYPOINT_REACHED_PX: self.wp_idx+=1
            else: break
        if self.wp_idx>=len(self.path): return {'keys':k,'fire':0}
        tx,ty=self.path[self.wp_idx]
        desired=math.atan2(ty-me['y'],tx-me['x'])
        err=wrap(desired-me['angle'])
        if err>TURN_DEADBAND_RAD: k['right']=1
        elif err<-TURN_DEADBAND_RAD: k['left']=1
        if abs(err)<TURN_ONLY_RAD: k['up']=1
        return {'keys':k,'fire':0}

    def on_message(self,c,text):
        try: msg=json.loads(text)
        except Exception: return
        if not isinstance(msg,dict): return
        if msg.get('map'): self.save_map(msg['map'])
        me=self.map_coords(msg.get('me')); foe=self.map_coords(msg.get('foe'))
        if not me or not foe:
            c.send_text(json.dumps({'keys':{'up':0,'down':0,'left':0,'right':0},'fire':0})); return
        t=float(msg.get('t',0.0)); self.record(t,me,foe,msg)
        if t-self.last_plan_t>=OODA_PERIOD_S: self.replan(t,me,foe)
        c.send_text(json.dumps(self.action(me),separators=(',',':')))

    def serve(self):
        ws=self.WSServer('127.0.0.1',self.port,self.on_client)
        print('navigation bot ws://127.0.0.1:%d/ai'%self.port,flush=True)
        try: ws.serve_forever()
        finally: self.close()

    def close(self):
        for obj in (getattr(self,'f',None),getattr(self,'pf',None)):
            try: obj.flush(); obj.close()
            except Exception: pass

if __name__=='__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('--port',type=int,default=8766); ap.add_argument('--out-dir',default=os.path.join(HERE,'nav_logs')); ap.add_argument('--repo',default=os.path.abspath(os.path.join(HERE,'..')))
    a=ap.parse_args(); Bot(a.port,a.out_dir,a.repo).serve()
