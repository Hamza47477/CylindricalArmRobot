// Planner / sphere / singularity proof.  Run:  node tests/planner_test.js | tee tests/results_planner.txt
// It extracts the REAL kinematics, IK and planner code from index.html, so the test is of the code the viewer runs.
const fs = require('fs'), path = require('path');
const html = fs.readFileSync(path.join(__dirname, '..', 'index.html'), 'utf8');
const code = html.slice(html.indexOf('const $ ='), html.indexOf('// ── state')).replace('const $ = s => document.querySelector(s), ', 'const ');
const test = `
const randPose = (ref) => { for (;;) { const p = [0,0,0].map(() => Math.random()*2-1); if (dot(p,p) > 1) continue;
  const d = norm([Math.random()-.5, Math.random()-.5, Math.random()-.5]); const r = ik(add(SPH_C, mul(p,300)), d, ref); if (r.ok) return r.q; } };
const plan = (a, b) => { let p = planPath(a, b); if (!p) { planExtra = 0; p = planPath(a, b); planExtra = PLAN_EXTRA; } return p; };
const MOVES = 200; let n = 0, fail = 0, bad = 0, det = 0, minS = 1e9, minR = 1e9, minClr = 1e9, a = randPose(null);
for (let i = 0; i < MOVES; i++) {
  const b = randPose(a), p = plan(a, b); if (!p) { fail++; continue; } if (p.length > 2) det++;
  for (let k = 0; k < p.length - 1; k++) for (let s = 0; s <= 20; s++) { const t = s/20, q = p[k].map((v,j) => v + (p[k+1][j]-v)*t), F = fk(q); n++;
    minS = Math.min(minS, sigmaArm(F.J,F.E,F.W)); minR = Math.min(minR, q[5], Math.PI-q[5]); minClr = Math.min(minClr, clearance(F.base,F.J,F.E,F.W)); if (hits(F).length) bad++; }
  a = b; }
console.log('TEST 7 - Planned motion between random reachable poses (RRT-Connect, 200 chained moves)');
console.log('  moves planned: ' + MOVES + ', failed: ' + fail + ', needing a detour around the sphere: ' + det);
console.log('  configurations checked along all paths: ' + n);
console.log('  configurations with ANY part of the structure (not the tool) inside the sphere: ' + bad);
console.log('  smallest clearance structure-to-sphere (incl. 5 mm margin): ' + minClr.toFixed(1) + ' mm  (>= 0 means clear)');
console.log('  smallest planar-arm sigma_min along all paths: ' + minS.toFixed(1) + ' mm/rad (limit ' + SIG_MIN + ')');
console.log('  smallest wrist angle R2 along all paths: ' + (minR/D2R).toFixed(1) + ' deg (stop ' + (LO[5]/D2R).toFixed(0) + ' deg)');
`;
new Function(code + test)();
