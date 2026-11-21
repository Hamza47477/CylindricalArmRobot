"""Reproducible proof for the report. Run:  python3 tests/run_kinematics_tests.py   (writes tests/results_kinematics.txt)"""
import sys, os, math, io, contextlib
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import numpy as np, robot as R

out = []
def log(s=""): print(s); out.append(str(s))

log("TEST 1 - Inverse kinematics accuracy (forward-kinematics round trip)")
rng = np.random.default_rng(3); err_p = err_d = 0; n = 0
for _ in range(2000):
    p, d = R.sample(1, rng); p, d = p[0], d[0]; q, ok = R.ik(p, d)
    if ok:
        F = R.fk(q); err_p = max(err_p, np.linalg.norm(F["tcp"] - p)); err_d = max(err_d, np.linalg.norm(F["zt"] - d)); n += 1
log(f"  {n} solved poses: max position error = {err_p:.2e} mm, max direction error = {err_d:.2e}")

log("\nTEST 2 - Reachability (Monte-Carlo, 3000 random tool-tip positions in the sphere + random tool directions)")
a = R.analyse(3000)
log(f"  physically possible for ANY robot (wrist centre outside sphere): {a['possible']*100:.1f} %")
log(f"  solved among the possible ones:                                  {a['ok_given_possible']*100:.2f} %")
log(f"  solved among ALL random pairs:                                   {a['ok']*100:.1f} %")

log("\nTEST 3 - Which positions of the sphere can be reached? (800 random positions x 60 directions)")
b = R.analyse_positions(800, 60)
log(f"  positions where NO direction is possible for ANY robot (wrist keep-out, ball of radius {R.WRIST_R+R.MARGIN:.0f} mm at the sphere centre): {b['blocked_positions']*100:.2f} %  (geometry predicts {100*((R.WRIST_R+R.MARGIN)/R.SPH_R)**3:.2f} %)")
log(f"  among the other positions, those with at least one valid tool direction: {b['any_dir']*100:.1f} %")
log(f"  average share of possible directions solved:       {b['mean_solved']*100:.1f} %")
log(f"  worst position:                                    {b['worst_solved']*100:.1f} %")

log("\nTEST 4 - Singularities and Jacobian condition number, margins ON (final design)")
s = R.singularity_report(3000)
log(f"  solved poses analysed: {s['solved']}")
log(f"  smallest planar-arm sigma_min : {s['min_sigma_arm']:.1f} mm/rad   (limit {R.SIG_MIN})")
log(f"  smallest wrist angle R2       : {s['min_R2_deg']:.1f} deg         (stop {math.degrees(R.SING_WRIST):.0f} deg)")
log(f"  worst / median condition number of the 5x6 task Jacobian: {s['max_cond']:.1f} / {s['median_cond']:.1f}")

log("\nTEST 5 - Same test with the margins switched OFF (shows why they are needed)")
sig, r2 = R.SIG_MIN, R.LIM["r2"]; R.SIG_MIN = 0; R.LIM["r2"] = (0.0, math.pi)
s0 = R.singularity_report(3000); a0 = R.analyse(3000)
log(f"  smallest wrist angle R2: {s0['min_R2_deg']:.1f} deg")
log(f"  worst / median condition number: {s0['max_cond']:.1f} / {s0['median_cond']:.1f}")
log(f"  solved among possible: {a0['ok_given_possible']*100:.2f} %")
R.SIG_MIN = sig; R.LIM["r2"] = r2

log("\nTEST 6 - Why the other architectures were rejected (see README table) - not re-run here, scripts in the design-search folder")
open(os.path.join(os.path.dirname(__file__), "results_kinematics.txt"), "w").write("\n".join(out) + "\n")
