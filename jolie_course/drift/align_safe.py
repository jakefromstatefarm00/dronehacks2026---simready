"""Safeguarded copy of Jolie's ``align_to_map`` (jolie-drift-correction f7b97d3), for the A/B.

Jolie's function is reproduced unchanged when every option is off. Options (``make_safe_align``):

robust      Tukey-weighted Gauss-Newton (c = robust m): hits that do not fit the consensus shift
            (an unmapped obstacle next to a wall, one wall that moved) stop pulling.
min_eig     absolute evidence per constrained direction (Jolie: 10); higher = fewer, surer fixes.
max_shift   refuse a frame whose shift is larger than this (Jolie: 0.3 m).
min_gain    required rms improvement (Jolie: 0.003 m).
inlier_frac after the shift, at least this fraction of the used hits within 5 cm of a wall.
"""
from __future__ import annotations

import numpy as np


def make_safe_align(opts):
    o = dict(robust=0.0, min_eig=10.0, max_shift=.3, min_gain=.003, inlier_frac=0.0,
             faces=False, face_min=15, face_tol=.04)
    o.update(opts or {})
    from pathplan.depth_projection import _bilinear

    if o.get("consensus"):
        # two estimators must agree: Jolie's unweighted fit (wide capture), then a Tukey-robust
        # refinement started from it; noise / clutter / a moved wall make them disagree -> refuse
        plain = make_safe_align({k: v for k, v in o.items() if k not in ("consensus", "robust")})
        robust_opts = dict(o, consensus=0, robust=o.get("robust") or .08, min_gain=-1.0, max_shift=9.0)
        refine = make_safe_align(robust_opts)

        def consensus(points, dist, grad, spec, **kw):
            s0, info = plain(points, dist, grad, spec, **kw)
            if s0 is None:
                return None, info
            s1, _ = refine(np.asarray(points, float) + s0, dist, grad, spec, **kw)
            s1 = np.zeros(2) if s1 is None else s1
            if float(np.linalg.norm(s1)) > o["consensus"]:
                return None, info
            return s0 + s1, info

        return consensus

    def align_to_map(points, dist, grad, spec, *, near=.35, min_points=40, iterations=5):
        info = {"points": 0}
        if points is None or len(points) == 0:
            return None, info
        pts = np.asarray(points, float)
        pts = pts[np.abs(_bilinear(dist, spec, pts)) < near]
        info["points"] = int(len(pts))
        if len(pts) < min_points:
            return None, info
        shift = np.zeros(2)
        rms0 = None
        wts = np.ones(len(pts))
        for _ in range(iterations):
            d = _bilinear(dist, spec, pts + shift)
            g = _bilinear(grad, spec, pts + shift)
            rms = float(np.sqrt(np.mean(d ** 2)))
            rms0 = rms if rms0 is None else rms0
            if o["robust"] > 0:
                u = np.clip(np.abs(d) / o["robust"], 0, 1)
                wts = (1 - u ** 2) ** 2
            H = (g * wts[:, None]).T @ g
            b = (g * wts[:, None]).T @ d
            w, v = np.linalg.eigh(H)
            step = np.zeros(2)
            for lam, vec in zip(w, v.T):
                if lam > max(o["min_eig"], .02 * float(wts.sum())):
                    step -= (vec @ b) / lam * vec
            n = float(np.linalg.norm(step))
            if n > .1:
                step *= .1 / n
            shift += step
            if n < .002:
                break
        d1 = _bilinear(dist, spec, pts + shift)
        rms1 = float(np.sqrt(np.mean(d1 ** 2)))
        info.update(rms_before=round(rms0, 4), rms_after=round(rms1, 4))
        if not np.any(shift) or rms1 > rms0 - o["min_gain"] or np.linalg.norm(shift) > o["max_shift"]:
            return None, info
        if o["inlier_frac"] > 0 and float(np.mean(np.abs(d1) < .05)) < o["inlier_frac"]:
            return None, info
        if o["faces"]:
            # every corrected direction needs >= 2 distinct wall faces that AGREE after the shift: one
            # face that moved (or a lone frontal wall) cannot set the frame on its own
            g1 = _bilinear(grad, spec, pts + shift)
            nrm = g1 / np.maximum(np.linalg.norm(g1, axis=1, keepdims=True), 1e-9)
            ang = np.round(np.arctan2(nrm[:, 1], nrm[:, 0]) / (np.pi / 4)).astype(int) % 8
            off = np.round(np.sum((pts + shift) * nrm, axis=1) / .3).astype(int)
            faces = {}
            for k, key in enumerate(zip(ang, off)):
                faces.setdefault(key, []).append(k)
            for lam, vec in zip(w, v.T):
                if lam <= max(o["min_eig"], .02 * float(wts.sum())):
                    continue
                support = [ix for ix in faces.values()
                           if len(ix) >= o["face_min"] and abs(float(np.mean(nrm[ix] @ vec))) > .5]
                if len(support) < 2:
                    return None, info
                if max(abs(float(np.mean(d1[ix]))) for ix in support) > o["face_tol"]:
                    return None, info
        return shift, info

    return align_to_map
