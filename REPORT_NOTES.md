# Technical notes for the report — P-R-R-R-R-R robot (Master ARMAC 2026, Sujet 2)

Everything here is backed by a test you can re-run (section 7). All lengths in mm, angles in rad/deg.

## 1. The robot in one paragraph
Fixed vertical column. A carriage slides along it (**d2**, prismatic). On the carriage, three revolute joints with **vertical** axes (**θ1, θ2, θ3**) carry three 450 mm links (L1, L2, forearm LF) that all move in one horizontal plane. At the end of the forearm a 2-joint wrist (**R1** roll about the forearm, **R2** pitch, axes crossing at the wrist centre W) carries the 300 mm tool. 6 joints, 6 DOF, **1 prismatic + 5 revolute**. The tool is a symmetric rod, so rotation about its own axis is irrelevant and no 7th joint is needed. Sphere: radius 300 mm, centre at (600, 0, 650) from the column base. Catalogue types combined: cylindric-style vertical slide + SCARA-style planar arm + spherical (roll-pitch) wrist. No parallel structure.

## 2. Forward kinematics
u(a) = (cos a, sin a, 0), f = u(θ1+θ2+θ3), g = z × f
- J = (0,0,d2) + L1·u(θ1)  (elbow 1)
- E = J + L2·u(θ1+θ2)  (elbow 2)
- W = E + LF·f  (wrist centre)
- yp = cos R1·g + sin R1·z, zp = −sin R1·g + cos R1·z
- tool axis: **zt = cos R2·f − sin R2·zp**
- tool tip: **TCP = W + 300·zt**

Checked against the FreeCAD model: difference ≈ 1e-13 mm.

## 3. Inverse kinematics (closed form, no numerical solver)
Input: target tip position p and tool direction d (unit vector).
1. Wrist centre: **W = p − 300·d**.
2. Height: **d2 = W_z** (and must be inside the slide range 40…1290 mm).
3. The three vertical joints place a 2-D point, so there is **one redundant degree of freedom: the forearm angle φ = θ1+θ2+θ3**. For a chosen φ the elbow-2 point is E = W_xy − LF·u(φ), r = |E|.
4. Two-link geometry (law of cosines): cos θ2' = (r² − L1² − L2²)/(2·L1·L2) → two elbow branches (±). Then θ1 = atan2(E_y,E_x) ∓ atan2(L2 sin θ2, L1 + L2 cos θ2), θ2 = ±θ2', and **θ3 = φ − θ1 − θ2**.
5. Wrist: with f, g, z as above: du = d·f, dg = d·g, dz = d·z → **R2 = atan2(√(dg²+dz²), du)**, **R1 = atan2(dg, −dz)**.
6. **Using the redundancy:** φ is sampled (144 values) × 2 elbow branches = 288 candidates. Each is kept only if it respects: joint limits, the singularity margins (section 5), and clearance of the whole structure from the sphere and column. Among the valid ones, the solution closest to the previous pose is chosen (smooth motion). So the redundant joint is what lets the arm stay out of the sphere and away from singularities.

**Is it difficult?** No: closed form (two atan2, one law of cosines), 288 cheap candidates, a few ms per pose. Round-trip accuracy: ~1e-12 mm (test 1).

## 4. Jacobian
Task = tool-tip position (3) + tool direction (2, the rod is symmetric) → **5×6 Jacobian**, 6 joints, so the arm is redundant by one for the task.

Planar block (wrist centre, joints d2, θ1, θ2, θ3), with z the vertical unit vector and positions in the arm plane:
- ∂W/∂d2 = z
- ∂W/∂θ1 = z × (W − O), O on the column axis
- ∂W/∂θ2 = z × (W − J)
- ∂W/∂θ3 = z × (W − E)

The horizontal part is a 2×3 matrix **A = [z×(W−O), z×(W−J), z×(W−E)]**. Wrist block: R1, R2 only change the tool direction (angular velocities along f and along yp).
For the report the full 5×6 Jacobian was computed numerically (central differences) and made dimensionless (position divided by 300 mm, d2 column scaled the same way) so its condition number κ = σ_max/σ_min means something. κ = 1 is ideal; κ → ∞ at a singularity.

## 5. Singularities and how they are removed (small changes, same architecture)
| Singularity | Cause | Fix |
|---|---|---|
| Wrist | R2 = 0° or 180° (tool along forearm): R1 axis aligns with R2, R1 undefined | **Mechanical stop R2 ∈ [20°, 160°]** + IK rejects φ outside it |
| Planar arm | Link 1, link 2 and forearm collinear → A loses rank (σ_min(A) = 0) | **Require σ_min(A) ≥ 150 mm/rad**; the redundant joint bends the arm away |
| Column axis | arm through the column axis | already excluded: links stay ≥ 78 mm from the axis |
| Prismatic d2 | — | a slide has no singularity |

Note: θ2 = 0 alone is **not** singular, because θ3 can compensate (redundancy). Only the fully collinear case is. A simple elbow limit would have removed the singularity but cost ≈5 % of reachable poses; the σ_min rule costs 0 %.

## 6. Results (all produced by the test scripts)
| Claim | Result | Test |
|---|---|---|
| IK is exact | max error 8e-13 mm, 4e-16 in direction (1063 poses) | 1 |
| Positions reachable | every position except a small ball of radius 53 mm around the sphere centre (0.55 % of the volume), which no robot with this wrist can reach: there the wrist is always ≈300 mm from the centre, inside its own keep-out. For all other sampled positions (800) ≥1 valid tool direction exists: 100 % | 3 |
| Share of possible orientations solved | **99.57 %** (3000 random pairs); average per position 99.6 %, worst position 86.5 % | 2, 3 |
| Physically possible for ANY robot | ≈ 54 % of random position+direction pairs (wrist centre must stay outside the sphere) | 2 |
| Wrist singularity removed | smallest R2 = 20.1° (1.1° without the stop) | 4, 5 |
| Arm singularity removed | smallest σ_min(A) = 172 mm/rad (limit 150) | 4 |
| Conditioning | worst κ = 15.0 with margins vs 42.5 without; median 6.1 | 4, 5 |
| Margins cost nothing | 99.57 % solved with and without | 4, 5 |
| Structure never enters the sphere while moving | 200 chained planned moves, 16 989 configurations checked, **0** with any part except the tool inside; min clearance 0.1 mm beyond the 5 mm margin | 7 |
| Moves planned successfully | 200 / 200 (153 needed a detour around the sphere) | 7 |
| No singularity while moving | σ_min ≥ 150.2 mm/rad and R2 ≥ 20.6° at every checked point of every path | 7 |

**Why not 100 % of random pairs?** Because about half of all (position, direction) pairs are impossible for any robot: the wrist centre is 300 mm behind the tip, and the sphere has radius 300 mm, so for many directions it would have to be inside the sphere, which the sheet forbids. The 99.57 % is measured over the possible pairs only. The viewer displays "impossible for ANY robot" for those.

## 7. How to reproduce (give these to whoever checks the claims)
From the project folder:
```
python3 tests/run_kinematics_tests.py      # tests 1-5, writes tests/results_kinematics.txt
node tests/planner_test.js                 # test 7, saved output in tests/results_planner.txt
```
The saved output files are the "proof of test". Random seeds are fixed for the Python tests; the planner test uses random poses, so its counts change slightly between runs but the safety results (0 sphere hits, margins respected) must hold every time. Live check: run `python3 server.py`, open http://localhost:8766, watch "Tour the sphere"; the Status panel shows σ_min and R2 live and turns red if the structure enters the sphere.

## 8. Choice justification (for the "discussion" section)
Layout comparison, 4 candidates (Monte-Carlo, same sphere, same tool): earlier rotating-column + 2 slides + elbow 93 %; all-revolute articulated ≤ 98 % with very long links; 5-DOF column + slide + 2 links 99 % but 770 mm links and no spare freedom; **final 99.6 %** with one slide, three identical 450 mm links (cheap, one part number), sphere raised to z = 650 mm so low poses don't need the wrist under the floor. Full table in README.md.
