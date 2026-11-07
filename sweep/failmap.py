"""One perturbation axis at a time, on top of the nominal level L0, at rising magnitude.

Each entry is a Level override (sim.Level fields); the name is "<axis>=<magnitude>".
"""
from sim import _dyn


def _ax(axis, mag, **kw):
    return {"name": f"{axis}={mag}", **kw}


AXES = (
    [_ax("walls", w, wall_shift=w) for w in (.05, .10, .15, .20, .25)]
    + [_ax("clutter", n, clutter=(n, n)) for n in (1, 2, 3, 4)]
    + [_ax("vslam_drift", d, dyn=_dyn(drift_per_sqrt_m=(d, d), pos_noise=(d / 2, d / 2), vel_noise=(.03, .03),
                                       drift_rate=(d / 10, d / 10), yaw_drift_rate=(d / 2, d / 2)))
       for d in (.01, .02, .04, .06)]
    + [_ax("link_delay_steps", n, dyn=_dyn(delay_steps=(n, n))) for n in (2, 3, 5, 8)]
    + [_ax("stalls", f"{r}/s,long{p}", stall_rate=r, stall_long_p=p, stall_max=m)
       for r, p, m in ((.63, .03, 1.95), (1.0, .08, 2.5), (1.5, .15, 3.0))]
    + [_ax("odom_pause", f"every{e}s,{lo}-{hi}s", pause_every_s=e, pause_s=(lo, hi))
       for e, lo, hi in ((25, .3, 1.4), (10, .3, 1.4), (5, .3, 1.4), (10, 1.0, 2.5))]
    + [_ax("px4_vel_lag_tau", tau, dyn=_dyn(tau=(tau, tau))) for tau in (.35, .7, 1.0, 1.3)]
    + [_ax("px4_kp", kp, dyn=_dyn(kp_xy=(kp, kp))) for kp in (.7, 1.2)]
    + [_ax("vio_hazard_x", s, vio_scale=s) for s in (1, 2, 4, 8)]
    + [_ax("depth_noise_k", k, depth_noise=k) for k in (.01, .02, .04)]
)

# calibration: the unmodified base at the nominal and the FlyX-like level, on the drone's clock
# (rtf 1) and on the FlyX lab's (~0.8x real time; FlyX's 120.9 s is lab wall clock)
CALIB = [{"name": f"{lv}@rtf{r}", "base": lv, "rtf": r} for lv in ("L0", "L2") for r in (1.0, .8)]
