# -*- coding: utf-8 -*-
from __future__ import annotations
import argparse,csv,glob,math,os,json
import numpy as np
from scipy.ndimage import distance_transform_edt
from navigation_mvp import load_maze,load_polys,build_maps

def load(path):
    with open(path,newline='',encoding='utf-8') as f:
        return [{k:float(v) for k,v in r.items() if v!=''} for r in csv.DictReader(f)]

def resample_xy(rows,prefix,step=5.0):
    pts=[(r[prefix+'_x'],r[prefix+'_y']) for r in rows]
    if len(pts)<2:return []
    out=[pts[0]]; carry=0.0; cur=pts[0]
    for target in pts[1:]:
        sx,sy=cur; tx,ty=target
        d=math.hypot(tx-sx,ty-sy)
        if d<1e-9: cur=target; continue
        while carry+d>=step:
            need=step-carry; q=need/d
            sx=sx+(tx-sx)*q; sy=sy+(ty-sy)*q
            out.append((sx,sy)); d=math.hypot(tx-sx,ty-sy); carry=0.0
            if d<1e-9: break
        carry+=d; cur=target
    return out

def trajectory_metrics(rows,prefix,raw_wall):
    pts=resample_xy(rows,prefix,5.0)
    if len(pts)<3:return None
    L=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(pts,pts[1:]))
    ang=[math.atan2(b[1]-a[1],b[0]-a[0]) for a,b in zip(pts,pts[1:])]
    turn=sum(abs((b-a+math.pi)%(2*math.pi)-math.pi) for a,b in zip(ang,ang[1:]))
    smooth100=turn/max(L,1)*100.0
    # Local path efficiency over ~50 px windows: chord / traveled, avoids rewarding standing still.
    win=10; eff=[]
    for i in range(len(pts)-win):
        chord=math.hypot(pts[i+win][0]-pts[i][0],pts[i+win][1]-pts[i][1])
        traveled=sum(math.hypot(pts[j+1][0]-pts[j][0],pts[j+1][1]-pts[j][1]) for j in range(i,i+win))
        if traveled>20: eff.append(chord/traveled)
    efficiency=float(np.mean(eff)) if eff else 0.0
    dist=distance_transform_edt(~raw_wall)
    clear=[]
    for x,y in pts:
        xi=min(raw_wall.shape[1]-1,max(0,int(round(x)))); yi=min(raw_wall.shape[0]-1,max(0,int(round(y))))
        clear.append(float(dist[yi,xi]))
    min_clear=float(np.min(clear)); near8=float(np.mean(np.array(clear)<=8.0)); near12=float(np.mean(np.array(clear)<=12.0))
    return dict(path_px=L,smooth_rad_per_100px=smooth100,local_efficiency=efficiency,min_clearance_px=min_clear,near_wall_8_ratio=near8,near_wall_12_ratio=near12)

def aggregate(metrics):
    keys=metrics[0].keys(); return {k:float(np.median([m[k] for m in metrics])) for k in keys}

def relative(ours,ai):
    # AI is normalized to 1. Higher is better. Safety uses near-wall ratio; epsilon avoids divide-by-zero.
    return {
      'path_score': ours['local_efficiency']/max(ai['local_efficiency'],1e-9),
      'smooth_score': ai['smooth_rad_per_100px']/max(ours['smooth_rad_per_100px'],1e-9),
      'safety_score': (1.0+ai['near_wall_8_ratio'])/(1.0+ours['near_wall_8_ratio']),
    }

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--data-root',required=True); ap.add_argument('--polys',required=True); ap.add_argument('--out',default='navigation_scores.csv'); a=ap.parse_args()
    polys=load_polys(a.polys); per=[]; me_all=[]; foe_all=[]
    for track in sorted(glob.glob(os.path.join(a.data_root,'*','track_*.csv'))):
        maze=os.path.join(os.path.dirname(track),os.path.basename(track).replace('track_','maze_'))
        if not os.path.exists(maze):continue
        raw,_,_=build_maps(load_maze(maze),polys,8,5); rows=load(track)
        mm=trajectory_metrics(rows,'me',raw); fm=trajectory_metrics(rows,'foe',raw)
        if not mm or not fm:continue
        me_all.append(mm); foe_all.append(fm); per.append((os.path.basename(track),mm,fm))
    ai=aggregate(foe_all); me=aggregate(me_all); rs=relative(me,ai)
    rs['composite_score']=(max(rs['path_score'],1e-9)*max(rs['smooth_score'],1e-9)*max(rs['safety_score'],1e-9))**(1/3)
    fields=['file','actor','path_px','smooth_rad_per_100px','local_efficiency','min_clearance_px','near_wall_8_ratio','near_wall_12_ratio']
    with open(a.out,'w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for fn,mm,fm in per:
            w.writerow({'file':fn,'actor':'green/history',**mm});w.writerow({'file':fn,'actor':'existing_ai',**fm})
    result={'existing_ai_baseline':ai,'historical_green':me,'historical_green_relative_to_ai':rs,'n_paired_matches':len(per),'n_ai_tracks':len(foe_all),'n_green_tracks':len(me_all)}
    print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
