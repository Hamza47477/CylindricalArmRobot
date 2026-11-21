"""P-R-R-R-R-R robot: fixed column + vertical slide, 3 vertical-axis revolutes (planar arm), 2-joint wrist.
Units: mm, radians. World: Z up, column axis = Z through the origin.

Joints (6 DOF, one prismatic only):
  q1 d2     PRISMATIC  carriage slides up/down on the FIXED column
  q2 th1    REVOLUTE   shoulder, vertical axis on the carriage
  q3 th2    REVOLUTE   elbow 1  (vertical axis)   -> link 1 (L1) then link 2 (L2)
  q4 th3    REVOLUTE   elbow 2  (vertical axis)   -> forearm (LF)
  q5 R1     REVOLUTE   wrist roll about the forearm axis
  q6 R2     REVOLUTE   wrist pitch;  R1/R2 axes cross at the wrist centre W
All three arm links lie in the horizontal plane z = d2.  phi = th1+th2+th3 is the forearm direction.
J = L1 u(th1);  E = J + L2 u(th1+th2);  W = E + LF u(phi);  tool Z = cos(R2) f + sin(R1) sin(R2) g - cos(R1) sin(R2) z
(f = u(phi), g = z x f).  TCP = W + 300 * tool Z.  Tool is a symmetric rod: no tool-roll joint.
"""
import math
import numpy as np

TOOL, SPH_R = 300.0, 300.0
SPH_C = np.array([600.0, 0.0, 650.0])
L1, L2, LF = 450.0, 450.0, 450.0
COL_R, ARM_R, WRIST_R, JOINT_R, COL_H = 45.0, 28.0, 48.0, 50.0, 1400.0
MARGIN = 5.0
SIG_MIN, SING_WRIST = 150.0, math.radians(20)   # singularity margins: min singular value of the planar arm Jacobian [mm/rad], wrist stop on R2
LIM = dict(d2=(40.0, 1290.0), th2=math.radians(150), th3=math.radians(150), r2=(SING_WRIST, math.pi - SING_WRIST))
COLX = COL_R + ARM_R + MARGIN          # links 2/forearm must stay this far from the column axis
PHI = np.linspace(-math.pi, math.pi, 144, endpoint=False)
REF = np.array([0.6, -1.2, 0.6])       # neutral th1, th2, th3 used to pick between equivalent IK solutions


def wrap(a): return (a + np.pi) % (2 * np.pi) - np.pi
def u(a): return np.array([math.cos(a), math.sin(a), 0.0])


def fk(q):
    d2, t1, t2, t3, r1, r2 = q
    J = np.array([0, 0, d2]) + L1 * u(t1); E = J + L2 * u(t1 + t2)
    f = u(t1 + t2 + t3); z = np.array([0, 0, 1.0]); g = np.cross(z, f); W = E + LF * f
    yp = math.cos(r1) * g + math.sin(r1) * z; zp = -math.sin(r1) * g + math.cos(r1) * z
    zt = math.cos(r2) * f - math.sin(r2) * zp
    return dict(J=J, E=E, W=W, f=f, yp=yp, zt=zt, xt=np.cross(yp, zt), tcp=W + TOOL * zt)


def _segd(c, a, b):
    ab = b - a; t = np.clip(np.einsum('...i,...i', c - a, ab) / np.maximum(np.einsum('...i,...i', ab, ab), 1e-12), 0, 1)
    return np.linalg.norm(c - (a + t[..., None] * ab), axis=-1)


def _xy_axis_dist(a, b):                  # distance of segment a-b (xy only) to the column axis
    a2 = np.array(a)[..., :2]; b2 = np.array(b)[..., :2]
    ab = b2 - a2; t = np.clip(-np.einsum('...i,...i', a2, ab) / np.maximum(np.einsum('...i,...i', ab, ab), 1e-12), 0, 1)
    return np.linalg.norm(a2 + t[..., None] * ab, axis=-1)


def sigma_arm(J, E, W):
    """Smallest singular value of the 2x3 position Jacobian of (th1, th2, th3): zero only when links 1, 2 and the forearm are collinear."""
    cols = [np.stack([-(W[..., 1] - P[..., 1]), W[..., 0] - P[..., 0]], -1) for P in (np.zeros_like(J), J, E)]
    a = sum(c[..., 0] ** 2 for c in cols); b = sum(c[..., 0] * c[..., 1] for c in cols); c_ = sum(c[..., 1] ** 2 for c in cols)
    return np.sqrt(np.maximum((a + c_) / 2 - np.sqrt(((a - c_) / 2) ** 2 + b ** 2), 0))


def clearance(J, E, W, c=None):
    """Smallest (distance - required distance): arm links, joints, wrist vs the sphere, and links vs the column."""
    c = SPH_C if c is None else c
    O = np.zeros_like(J); O[..., 2] = J[..., 2]
    r = SPH_R + ARM_R + MARGIN
    return np.minimum.reduce([_segd(c, O, J) - r, _segd(c, J, E) - r, _segd(c, E, W) - r,
                              np.linalg.norm(c - J, axis=-1) - SPH_R - JOINT_R - MARGIN, np.linalg.norm(c - E, axis=-1) - SPH_R - JOINT_R - MARGIN,
                              np.linalg.norm(c - W, axis=-1) - SPH_R - WRIST_R - MARGIN,
                              _xy_axis_dist(J, E) - COLX, _xy_axis_dist(E, W) - COLX])


def structure_hits(q, c=None, radius=SPH_R):
    c = SPH_C if c is None else c; F = fk(q); O = np.array([0, 0, q[0]]); h = []
    if _segd(c, O, F["J"]) < radius + ARM_R or np.linalg.norm(c - F["J"]) < radius + JOINT_R: h.append("link 1 / shoulder")
    if _segd(c, F["J"], F["E"]) < radius + ARM_R or np.linalg.norm(c - F["E"]) < radius + JOINT_R: h.append("link 2 / elbow")
    if _segd(c, F["E"], F["W"]) < radius + ARM_R: h.append("forearm")
    if np.linalg.norm(c - F["W"]) < radius + WRIST_R: h.append("wrist R1/R2")
    return h


def ik_candidates(W, d=None):
    """All planar-arm solutions for wrist centre W: arrays th1, th2, th3, J, E, valid (limits) and clearance."""
    z = W[2]; f = np.stack([np.cos(PHI), np.sin(PHI)], 1); E2 = W[:2] - LF * f; r = np.hypot(E2[:, 0], E2[:, 1])
    ok = (r <= L1 + L2) & (r >= abs(L1 - L2) + 1e-6) & (LIM["d2"][0] <= z <= LIM["d2"][1])
    out = []
    for s in (1, -1):
        rr = np.where(ok, r, 1.0); a = np.arccos(np.clip((L1 ** 2 + rr ** 2 - L2 ** 2) / (2 * L1 * rr), -1, 1)); ang = np.arctan2(E2[:, 1], E2[:, 0])
        t1 = ang + s * a; J2 = L1 * np.stack([np.cos(t1), np.sin(t1)], 1)
        t12 = np.arctan2(E2[:, 1] - J2[:, 1], E2[:, 0] - J2[:, 0]); t2 = wrap(t12 - t1); t3 = wrap(PHI - t12)
        J = np.c_[J2, np.full(len(r), z)]; E = np.c_[E2, np.full(len(r), z)]; Wb = np.broadcast_to(W, E.shape)
        valid = ok & (np.abs(t2) <= LIM["th2"]) & (np.abs(t3) <= LIM["th3"])
        if d is not None:                      # wrist singularity: angle between tool and forearm must stay inside the R2 stops
            R2 = np.arccos(np.clip(np.cos(PHI) * d[0] + np.sin(PHI) * d[1], -1, 1))
            valid = valid & (R2 >= LIM["r2"][0]) & (R2 <= LIM["r2"][1])
        valid = valid & (sigma_arm(J, E, Wb) >= SIG_MIN)
        out.append((wrap(t1), t2, t3, J, E, valid, clearance(J, E, Wb)))
    return out


def ik(tcp, d, qprev=None):
    """Position + tool direction -> (q, ok). ok = limits respected AND the whole structure clear of the sphere.
    Among valid solutions the one closest to qprev (or REF) is returned, so the motion stays smooth."""
    d = np.asarray(d, float); d = d / np.linalg.norm(d); W = np.asarray(tcp, float) - TOOL * d
    ref = REF if qprev is None else np.array(qprev[1:4]); best = None
    for t1, t2, t3, J, E, valid, clr in ik_candidates(W, d):
        good = valid & (clr >= 0)
        cost = (wrap(t1 - ref[0])) ** 2 + (t2 - ref[1]) ** 2 + (t3 - ref[2]) ** 2
        key = np.where(good, -cost - 100.0 * (clr < 8.0), np.where(valid, -1e3 + clr * 1e-3, -1e6 + clr * 1e-3))
        k = int(np.argmax(key))
        if best is None or key[k] > best[0]: best = (key[k], t1[k], t2[k], t3[k], bool(good[k]))
    if best is None or best[0] < -1e5:   # nothing inside the arm's reach at all
        return [W[2], 0, 0, 0, 0, 0], False
    _, t1, t2, t3, good = best; f = u(t1 + t2 + t3); z = np.array([0, 0, 1.0]); g = np.cross(z, f)
    du, dg, dz = d @ f, d @ g, d @ z
    r2 = math.atan2(math.hypot(dg, dz), du); r1 = math.atan2(dg, -dz) if math.hypot(dg, dz) > 1e-9 else 0.0
    return [W[2], float(t1), float(t2), float(t3), r1, r2], good


def sample(n, rng):
    pts = []
    while len(pts) < n:
        p = rng.uniform(-1, 1, 3)
        if p @ p <= 1: pts.append(SPH_C + SPH_R * p)
    ds = rng.normal(size=(n, 3)); ds /= np.linalg.norm(ds, axis=1)[:, None]
    return np.array(pts), ds


def possible(p, d):
    """Can ANY robot with a 300 mm tool do it? Only if the wrist centre is outside the sphere."""
    return np.linalg.norm(p - TOOL * d - SPH_C) >= SPH_R + WRIST_R + MARGIN


def analyse(n=1500, seed=0):
    rng = np.random.default_rng(seed); P, D = sample(n, rng); pos = ok = 0
    for p, d in zip(P, D):
        if not possible(p, d): continue
        pos += 1; ok += ik(p, d)[1]
    return dict(n=n, possible=pos / n, ok_given_possible=ok / max(pos, 1), ok=ok / n)


def analyse_positions(n=150, ndir=60, seed=1):
    rng = np.random.default_rng(seed); P, _ = sample(n, rng)
    dirs = rng.normal(size=(ndir, 3)); dirs /= np.linalg.norm(dirs, axis=1)[:, None]
    anyd = 0; shares = []; blocked = 0
    for p in P:
        poss = [d for d in dirs if possible(p, d)]
        if not poss: blocked += 1; continue                  # no tool direction is possible for ANY robot here (wrist would be inside the sphere)
        sol = sum(ik(p, d)[1] for d in poss)
        anyd += sol > 0; shares.append(sol / len(poss))
    return dict(blocked_positions=blocked / n, any_dir=anyd / max(len(shares), 1), mean_solved=float(np.mean(shares)), worst_solved=float(np.min(shares)))


def task_jacobian(q, h=1e-6):
    """5x6 Jacobian of (TCP/300 [3], tool direction in its 2-D tangent plane [2]) w.r.t. (d2/300, th1, th2, th3, R1, R2).
    Dimensionless, so its condition number is meaningful.  Tool direction has only 2 DOF (the rod is symmetric)."""
    q = np.array(q, float); zt = fk(q)["zt"]; a = np.cross(zt, [0, 0, 1.0])
    if np.linalg.norm(a) < 1e-6: a = np.cross(zt, [1.0, 0, 0])
    a /= np.linalg.norm(a); b = np.cross(zt, a)
    def t(q): F = fk(q); return F["tcp"] / TOOL, F["zt"]
    cols = []
    for i in range(6):
        e = np.zeros(6); e[i] = h; (p1, z1), (p0, z0) = t(q + e), t(q - e)
        dz = (z1 - z0) / (2 * h); col = np.r_[(p1 - p0) / (2 * h), a @ dz, b @ dz]
        cols.append(col * (TOOL if i == 0 else 1))
    return np.array(cols).T


def singularity_report(n=1500, seed=5):
    """Over all solved poses: smallest sigma_arm, smallest wrist angle R2, and condition number of the task Jacobian."""
    rng = np.random.default_rng(seed); P, D = sample(n, rng); sa, r2, cond, k = [], [], [], 0
    for p, d in zip(P, D):
        if not possible(p, d): continue
        q, ok = ik(p, d)
        if not ok: continue
        F = fk(q); sa.append(float(sigma_arm(F["J"], F["E"], F["W"]))); r2.append(min(q[5], math.pi - q[5]))
        s = np.linalg.svd(task_jacobian(q), compute_uv=False); cond.append(s[0] / s[-1]); k += 1
    return dict(solved=k, min_sigma_arm=min(sa), min_R2_deg=math.degrees(min(r2)), max_cond=max(cond), median_cond=float(np.median(cond)))


if __name__ == "__main__":
    rng = np.random.default_rng(3); err = 0
    for _ in range(100):
        p, d = sample(1, rng); p, d = p[0], d[0]
        q, ok = ik(p, d); f = fk(q)
        if ok: err = max(err, np.linalg.norm(f["tcp"] - p), np.linalg.norm(f["zt"] - d))
    print("IK round-trip max error (valid poses):", err)
    print(analyse()); print(analyse_positions()); print(singularity_report())
