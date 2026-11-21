# Robotic design for a specific customer need — Master ARMAC 2026, Subject 2
**Robot: P-R-R-R-R-R, 6 degrees of freedom — fixed column, vertical slide, planar arm, spherical wrist**

## 1. Objective and how the design meets each requirement
| Requirement of the sheet | How it is met | Proof |
|---|---|---|
| Reach positions in a sphere of radius 300 mm | Sphere centred at (600, 0, 650) mm in front of the column; the tool tip reaches the sampled positions of the sphere | §8, test 3 |
| Any orientation of the tool | 6 DOF: 3 for the tip position, 2 for the tool direction, 1 redundant joint used for avoidance. The 300 mm tool is a symmetric rod, so its own rotation is irrelevant | §3, §8 |
| Only the tool (Z axis, 300 mm) enters the sphere | The structure is kept outside the sphere by the inverse kinematics and by the path planner | §6, §7, test 7 |
| Catalogue architectures only, no parallel architecture | Combination of cylindric-type vertical slide, SCARA-type planar arm and a roll-pitch spherical wrist; fully serial | §2 |
| Avoid singularities | Removed by two mechanical stops and one rule in the IK, with no change of architecture | §5 |
| Simulation / CAD | Interactive 3D simulator + FreeCAD model | §7, §9 |

## 2. Chosen architecture
| # | Joint | Type | Role |
|---|---|---|---|
| 1 | d2 | Prismatic | Carriage slides up and down the fixed column (only linear axis) |
| 2 | θ1 | Revolute | Shoulder, vertical axis |
| 3 | θ2 | Revolute | Elbow 1, vertical axis |
| 4 | θ3 | Revolute | Elbow 2, vertical axis |
| 5 | R1 | Revolute | Wrist roll about the forearm |
| 6 | R2 | Revolute | Wrist pitch (axes of R1 and R2 cross at the wrist centre W) |

- **6 joints = 6 DOF**: 1 prismatic + 5 revolute. Column Ø90 × 1400 mm, fixed to the base.
- **Three identical 450 mm links** (link 1, link 2, forearm), all moving in one horizontal plane at height d2 — one part number, low cost.
- **300 mm tool** at the end of the wrist.
- Strokes: d2 from 40 to 1290 mm, θ2 and θ3 within ±150°.

**Why this layout** (Monte-Carlo comparison of four layouts on the same sphere and tool):

| Layout | Linear axes | Share of reachable poses solved |
|---|---|---|
| Rotating column + 2 slides + elbow | 2 | 93 % |
| All-revolute articulated arm | 0 | up to 98 %, with very long links |
| 5-DOF column + slide + 2 links | 1 | 99 %, links of about 770 mm, no spare freedom |
| **Fixed column + slide + 3 vertical revolutes + wrist (chosen)** | **1** | **99.6 %** |

Design choices that made the difference:
1. Sphere raised to z = 650 mm and placed 600 mm from the column, so no pose needs the wrist under the floor or on top of the column.
2. A third vertical joint gives one redundant degree of freedom: the same wrist position can be reached with different arm shapes, so the arm can be shaped to stay out of the sphere and away from singularities.
3. A revolute link replaces a long horizontal slide: fewer linear axes, lower cost, simpler mechanics.

## 3. Forward kinematics
With u(a) = (cos a, sin a, 0), f = u(θ1+θ2+θ3), g = z × f:
- Elbow 1: J = (0, 0, d2) + 450·u(θ1)
- Elbow 2: E = J + 450·u(θ1+θ2)
- Wrist centre: W = E + 450·f
- Tool axis: zt = cos R2 · f − sin R2 · zp, with zp = −sin R1 · g + cos R1 · z
- Tool tip: **TCP = W + 300 · zt**

The FreeCAD model and the Python model agree to about 1e-13 mm.

## 4. Inverse kinematics (closed form)
Given a target tip position p and tool direction d:
1. **Wrist centre:** W = p − 300·d.
2. **Height:** d2 = W_z.
3. **Planar arm:** for a chosen forearm angle φ = θ1+θ2+θ3, the elbow point is E = W_xy − 450·u(φ). The law of cosines gives θ2 (two elbow branches) and θ1; then θ3 = φ − θ1 − θ2.
4. **Wrist:** R2 = atan2(√(dg² + dz²), du), R1 = atan2(dg, −dz), with du = d·f, dg = d·g, dz = d·z.
5. **Use of the redundancy:** φ is free. The solver tries 144 values of φ on both elbow branches (288 candidates), keeps those that respect the joint limits, the singularity margins and the sphere clearance, and returns the one closest to the previous pose, so motion stays smooth.

The solution is analytical: no iterations and no numerical solver, a few milliseconds per pose. Accuracy by forward-kinematics round trip: **position error below 1e-12 mm**.

## 5. Jacobian and singularity removal
**Jacobian.** The task (tip position + tool direction) has 5 components, giving a 5×6 Jacobian. Its planar block, for the wrist centre, is
- ∂W/∂d2 = z
- ∂W/∂θ1 = z × (W − O), ∂W/∂θ2 = z × (W − J), ∂W/∂θ3 = z × (W − E)

where O is on the column axis. The condition number κ = σ_max / σ_min of the dimensionless Jacobian measures how far a pose is from a singularity (κ = 1 ideal, κ → ∞ at a singularity).

**Singularities and how each is removed**
| Singularity | Cause | Removal |
|---|---|---|
| Wrist | R2 = 0° or 180°: tool along the forearm, R1 undefined | Mechanical stop **R2 between 20° and 160°** + IK rule |
| Planar arm | Link 1, link 2 and forearm collinear: the arm Jacobian loses rank | Rule **σ_min ≥ 150 mm/rad**; the redundant joint bends the arm away |
| Column axis | Arm passing through the column axis | Links kept at least 78 mm from the axis |
| Prismatic axis | — | A slide has no singularity |

The path planner applies the same limits, so the robot also stays away from singularities **during** motion, not only at the target poses. These changes cost no reachable poses.

## 6. Respecting the sphere boundary
- Every IK solution is checked: links, joints and wrist must stay outside the sphere with a 5 mm margin; only the tool enters.
- **Collision-free motion planning:** every move (Home, Random point, Tour, Go to a point) is planned with RRT-Connect in joint space and shortened. When the direct path would cross the sphere, the arm goes around it from outside. Parts that would touch the sphere turn red in the viewer.
- Result: during 200 chained planned moves, no part of the structure except the tool entered the sphere.

## 7. Simulation
The simulator (`python3 server.py` → http://localhost:8766) is a lightweight 3D viewer that shows:
- base, column, carriage, all joints and the three 450 mm links, the wrist and the 300 mm tool;
- the 300 mm workspace sphere, the target point and the tool-tip trail;
- joint sliders, a "go to a point" mode with tool direction (inverse kinematics), Home, Random point and a tour of the sphere;
- a live readout of the singularity indicators (σ_min and R2) and of the sphere clearance;
- a Monte-Carlo button that measures the reachability.

## 8. Results and accuracy
All results come from reproducible scripts (`tests/`), saved in `tests/results_kinematics.txt` and `tests/results_planner.txt`.

| Quantity | Result |
|---|---|
| Inverse kinematics accuracy | error below 1e-12 mm (position) and 4e-16 (direction), 1063 poses |
| **Orientations reached, among those where the wrist can stay outside the sphere** | **99.57 %** (3000 random position + direction pairs) |
| Average share of possible tool directions solved per position | **99.6 %** (800 random positions) |
| Positions with at least one valid tool direction | **100 %** of the sampled positions where a direction is physically possible (800 sampled; the wrist keep-out blocks a 53 mm ball at the sphere centre for any robot) |
| Smallest wrist angle R2 | 20.1° (stop at 20°) |
| Smallest arm σ_min | 172 mm/rad (limit 150) |
| Worst Jacobian condition number | 15.0 (median 6.1) |
| Planned moves that succeeded | 200 / 200 (153 needed a detour around the sphere) |
| Configurations where the structure was inside the sphere | 0 of 16 989 checked along the paths |
| Singularity margins during motion | σ_min ≥ 150 mm/rad and R2 ≥ 20° at every checked point |

Note on the reference for 99.57 %: the wrist centre lies 300 mm behind the tool tip, so for some directions the wrist would have to be inside the sphere, which the sheet forbids; this holds for any robot with a 300 mm tool. The percentage is computed over the orientations where the wrist can stay outside, and the viewer labels the others "impossible for ANY robot".

## 9. CAD model
`CylindricalArmRobot.FCStd` (FreeCAD) contains the base, fixed column, carriage, the three joints with their links, the wrist with joints R1 and R2, the 300 mm tool, the workspace sphere and the TCP. It is generated by `build_model.py`; the geometry follows exactly the kinematics of §3.

## 10. Conclusion
We designed a serial 6-DOF robot with a single linear axis and three identical links that:
- meets the requirements of the sheet (sphere of radius 300 mm, only the tool inside, serial catalogue architecture);
- has a closed-form inverse kinematics accurate to 1e-12 mm that exploits one redundant joint;
- is free of wrist and arm singularities inside its working area, with a worst condition number of 15;
- reaches 99.6 % of the orientations that are physically possible, and every sampled position of the sphere where an orientation is physically possible;
- moves between points around the sphere with a collision-free planner, validated over 200 chained moves.

## 11. How to reproduce the results
```
python3 tests/run_kinematics_tests.py     # accuracy, reachability, singularities, Jacobian
node tests/planner_test.js                # motion around the sphere, singularities during motion
python3 server.py                         # simulator at http://localhost:8766
freecadcmd build_model.py                 # regenerates the FreeCAD model
```
