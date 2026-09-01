# -*- coding: utf-8 -*-
"""闭环仿真：对比 纯追踪(PurePursuit) vs 斯坦利(Stanley+曲率前馈) 横向控制器。

游戏物理：加减速 200px/s²、转向 3.8 rad/s、66Hz、桥端速度量化噪声 ±3.3px/s。
"""
import sys, os, math, random
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'python')))
import bot_config as CFG

DT = 1.0 / 66.0
ACCEL = 200.0
DECEL = 200.0
STEER_RATE = 3.8


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def nearest_dense_idx(me, dense):
    bx, by = me
    bi, bd = 0, 1e18
    for i, p in enumerate(dense):
        d = (p[0] - bx) * (p[0] - bx) + (p[1] - by) * (p[1] - by)
        if d < bd:
            bd = d
            bi = i
    return bi


def pure_pursuit_w(me, dense, look):
    bi = nearest_dense_idx(me, dense)
    acc = 0.0
    for i in range(bi, len(dense) - 1):
        x1, y1 = dense[i]
        x2, y2 = dense[i + 1]
        seg = math.hypot(x2 - x1, y2 - y1)
        if acc + seg >= look:
            u = (look - acc) / seg if seg > 1e-6 else 0.0
            return (x1 + (x2 - x1) * u, y1 + (y2 - y1) * u)
        acc += seg
    return dense[-1]


def stanley_w(me, dense, heading, v_est, tangent_prev, omega_applied):
    """返回 (ω_des, tangent) —— 与 navigation_bot._stanley_steer 一致。"""
    n = len(dense)
    bx, by = me
    bi = nearest_dense_idx(me, dense)
    # 切线：用 bi−1→bi+1 的 8px 跨度（最近点跳变时切线方向平滑，D 项不尖峰）
    j0 = max(0, bi - 1)
    j1 = min(n - 1, bi + 1)
    if j1 <= j0:
        return 0.0, 0.0
    tx, ty = dense[j1][0] - dense[j0][0], dense[j1][1] - dense[j0][1]
    tl = math.hypot(tx, ty)
    if tl < 1e-6:
        return 0.0, 0.0
    ux, uy = tx / tl, ty / tl
    tangent = math.atan2(ty, tx)
    th_e = wrap(tangent - heading)
    # D 项：切线旋转率 − 上一帧实际施加的转向角速度（均无测量噪声）
    tangent_rate = 0.0 if tangent_prev is None else wrap(tangent - tangent_prev) / DT
    # 横向误差：以 8px 跨度段为参考（投影钳制）
    dx, dy = bx - dense[j0][0], by - dense[j0][1]
    proj = max(0.0, min(tl, dx * ux + dy * uy))
    ex, ey = bx - (dense[j0][0] + ux * proj), by - (dense[j0][1] + uy * proj)
    e_ct = ux * ey - uy * ex
    kappa = 0.0
    i0, i1, i2 = bi, min(bi + 2, n - 1), min(bi + 4, n - 1)
    if i2 > i1 > i0:
        ax, ay = dense[i0]
        mx, my = dense[i1]
        cx, cy = dense[i2]
        abx, aby = mx - ax, my - ay
        bcx, bcy = cx - mx, cy - my
        ab = math.hypot(abx, aby)
        bc = math.hypot(bcx, bcy)
        ca = math.hypot(cx - ax, cy - ay)
        if min(ab, bc) > 0.5 and ab + bc + ca > 1e-6:
            cross = abx * bcy - aby * bcx
            area2 = abs(cross)
            if area2 > 1e-6:
                r = ab * bc * ca / (2.0 * area2)
                if 1.0 < r < 1000.0:
                    kappa = (1.0 / r) if cross > 0 else (-1.0 / r)
    w_ff = CFG.STANLEY_FF_GAIN * v_est * kappa
    w_h = CFG.PID_KP_TH * th_e + CFG.PID_KD_TH * (tangent_rate - omega_applied)
    w_ct = max(-CFG.STANLEY_CT_MAX, min(CFG.STANLEY_CT_MAX, CFG.STANLEY_K_CT * e_ct))
    w_des = w_ff + w_h - w_ct
    return max(-STEER_RATE, min(STEER_RATE, w_des)), tangent


def simulate(dense, total_s, mode='stanley', x0=0.0, y0=0.0, h0=None,
             v_max_curve=None):
    x, y = x0, y0
    h = h0 if h0 is not None else math.atan2(dense[1][1] - dense[0][1], dense[1][0] - dense[0][0])
    v = 0.0
    v_smooth = None
    throttle = False
    steer_accum = 0.0
    tangent_prev = None
    omega_applied = 0.0
    out = []
    for f in range(int(total_s / DT)):
        # 横向控制
        if mode == 'stanley':
            w_des, tangent_prev = stanley_w((x, y), dense, h, v_smooth or 0.0,
                                            tangent_prev, omega_applied)
        else:
            lp = pure_pursuit_w((x, y), dense, CFG.LOOKAHEAD_PX)
            w_des = CFG.PID_KP_TH * wrap(math.atan2(lp[1] - y, lp[0] - x) - h)
        duty = min(1.0, abs(w_des) / STEER_RATE)
        steer_accum += duty
        steer_on = False
        if steer_accum >= 1.0:
            steer_accum -= 1.0
            steer_on = True
        # 纵向控制（模型观测器 + 滞回）
        v_target = v_max_curve if v_max_curve is not None else CFG.CRUISE_SPEED_LOW
        v_meas = v + random.uniform(-3.3, 3.3)
        if v_smooth is None:
            v_smooth = v_meas
        else:
            a_model = CFG.SPEED_MODEL_ACCEL if throttle else -CFG.SPEED_MODEL_ACCEL
            v_smooth += a_model * DT
            v_smooth += CFG.SPEED_MEAS_CORR * (v_meas - v_smooth)
        if v_smooth < v_target - CFG.SPEED_HYST_PX_S:
            throttle = True
        elif v_smooth > v_target + CFG.SPEED_HYST_PX_S:
            throttle = False
        # 物理
        if steer_on:
            h += (STEER_RATE * DT) if w_des > 0 else (-STEER_RATE * DT)
        omega_applied = (STEER_RATE if steer_on else 0.0) * (1 if w_des > 0 else -1)
        a = ACCEL if throttle else -DECEL
        v = max(0.0, v + a * DT)
        x += v * math.cos(h) * DT
        y += v * math.sin(h) * DT
        out.append((f * DT, v, h, x, y))
    return out


def path_error(r, cx=0.0, cy=0.0, R=None):
    if R is None:
        return [abs(y) for _, _, _, _, y in r]
    return [abs(math.hypot(x - cx, y - cy) - R) for _, _, _, x, y in r]


if __name__ == '__main__':
    random.seed(0)
    dense_straight = [(float(i), 0.0) for i in range(0, 420, 4)]

    def circle(R, arc_step=4.0):
        """整圆路径，弧长 4px 采样（与真实密集路径一致）。"""
        pts = []
        n = int(2 * math.pi * R / arc_step)
        for i in range(n):
            a = 2 * math.pi * i / n
            pts.append((R * math.sin(a), R - R * math.cos(a)))
        pts.append(pts[0])
        return pts

    dense_curve = circle(30.0)
    dense_tight = circle(16.0)

    print('=== 直线 + 10px 初始横向偏移（收敛性与过冲）===')
    for mode in ('pure', 'stanley'):
        r = simulate(dense_straight, 4.0, mode=mode, x0=0.0, y0=10.0)
        errs = path_error(r)
        tail = errs[-120:]
        print('%-8s settle err mean=%.2f max=%.2f | 第1s err=%s' % (
            mode, sum(tail) / len(tail), max(tail),
            ' '.join('%.1f' % e for e in errs[:132:33])))

    print('=== R=30 整圆跟踪误差（速度上限60）===')
    for mode in ('pure', 'stanley'):
        r = simulate(dense_curve, 10.0, mode=mode, v_max_curve=60.0)
        errs = path_error(r, cx=0.0, cy=30.0, R=30.0)
        tail = errs[-240:]   # 最后 ~3.6s 稳态
        print('%-8s err mean=%.2f max=%.2f | 速度 mean=%.1f' % (
            mode, sum(tail) / len(tail), max(tail),
            sum(v for _, v, *_ in r) / len(r)))

    print('=== R=16 整圆（弯道限速 16*3.8=60.8→cap60）===')
    for mode in ('pure', 'stanley'):
        r = simulate(dense_tight, 8.0, mode=mode, v_max_curve=60.0)
        errs = path_error(r, cx=0.0, cy=16.0, R=16.0)
        tail = errs[-200:]
        print('%-8s err mean=%.2f max=%.2f' % (mode, sum(tail) / len(tail), max(tail)))
