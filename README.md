# Column + planar arm robot — Master ARMAC 2026 (P-R-R-R-R-R, 6 DOF, one prismatic axis)

Simulator:  `python3 server.py` -> http://localhost:8766     FreeCAD file:  `freecadcmd build_model.py` -> `CylindricalArmRobot.FCStd`
"Mirror live into FreeCAD window" pushes the pose through the FreeCAD MCP RPC server (port 9875).

## Final architecture (replaces the earlier rotating-column + 2 slides + elbow version)
| joint | type | role |
|---|---|---|
| d2 | prismatic | carriage slides up/down on the FIXED column (the only linear axis) |
| θ1 | revolute | shoulder, vertical axis on the carriage |
| θ2, θ3 | revolute | two more vertical-axis joints; links L1 = L2 = LF = 450 mm (three identical links) |
| R1, R2 | revolute | wrist roll + pitch, axes cross at the wrist centre W; 300 mm tool, no tool-roll joint |
Sphere R300 centred at (600, 0, 650). Strokes d2 40…1290, θ2/θ3 ±150°, column Ø90 x 1400, sphere ≥ 255 mm from the column surface.

## Why this layout (numerical comparison, Monte-Carlo over random TCP position + tool direction)
Poses where the wrist centre W = TCP − 300·d falls inside the sphere are impossible for ANY robot (~47 %). Of the possible ones:
| layout | prismatic axes | solved |
|---|---|---|
| rotating column + 2 slides + elbow (previous) | 2 | 93 % |
| all-revolute articulated arm (shoulder+elbow+wrist) | 0 | ≤ 98 %, needs very long links |
| 5-DOF fixed column + slide + 2 links | 1 | 99 % but links ~770 mm and no redundancy |
| **fixed column + slide + 3 vertical revolutes + wrist (this one)** | **1** | **99.6 %** (≥ 99.9 % on larger samples of the design search) |
What made the difference:
1. The sphere used to sit too low: poses approaching from below needed the wrist under the floor. Centre raised to z = 650.
2. The third vertical-axis joint (the old tool-roll joint, moved here) gives one redundant DOF for the same W, so the IK picks a configuration in which every link stays outside the sphere.
3. Swapping the long horizontal prismatic axis for a revolute link removes a costly linear axis and the rotating column.
4. Sphere moved to 600 mm so the wrist never has to sit on top of the column.
All 160 tested TCP positions (120 on the sphere surface) have valid tool directions; worst point still solves 88 % of its possible directions.

## Singularities — how they are removed (no change of architecture)

A 6-DOF serial arm has exactly two kinds of singularity here. Both are removed by **two cheap mechanical stops plus a rule in the IK**; the layout, link lengths and joint count are unchanged.

| Singularity | Where it happens | Why it is bad | Fix |
|---|---|---|---|
| **Wrist** (R1/R2 axes align) | R2 = 0° or 180° (tool along the forearm) | R1 undefined, infinite R1 speed | **Mechanical stop on R2: 20° … 160°.** The singular pose cannot be reached. |
| **Planar arm** (links 1, 2 and forearm collinear) | position Jacobian of θ1, θ2, θ3 loses rank | the wrist centre cannot move in some direction | **Keep σ_min of the 2×3 Jacobian ≥ 150 mm/rad.** The third vertical joint (redundant) is used to bend the arm away; IK discards forearm angles φ that violate it. |
| Column axis | arm through the column axis | θ1 undefined | already impossible: links stay ≥ 78 mm from the axis |
| Prismatic joint d2 | — | has no singularity | — |

Key point for the report: **θ2 = 0 alone is not a singularity** because θ3 can compensate (the arm is redundant by one DOF). Only the fully collinear case is. So no one-sided elbow limit is needed (that would have cost 5 % of the reachable poses); the σ_min rule removes the singularity at no cost.

**Measured effect** (3000 random poses, `tests/run_kinematics_tests.py`, output in `tests/results_kinematics.txt`; condition number of the 5×6 task Jacobian of TCP position and tool direction, made dimensionless with the 300 mm tool length):

| | without margins | with margins (final) |
|---|---|---|
| smallest wrist angle R2 | 1.1° | **20.1°** |
| worst condition number | 42.5 | **15.0** |
| median condition number | 6.1 | 6.1 |
| solved share of possible poses | 99.57 % | **99.57 %** (no loss) |

The margins cost no reachable poses. Path planning also rejects every configuration that violates them, so the robot never crosses a singularity *while moving* either (`tests/planner_test.js`: 200 chained planned moves, 0 failures, 0 sphere hits, σ_min ≥ 150 mm/rad and R2 ≥ 20° at every checked point). The viewer shows σ_min and R2 live.

## Motion planning (viewer)
Moves between poses are planned in joint space with RRT-Connect (+ shortcutting) and a 28 mm extra clearance, so the structure goes AROUND the sphere.
Offline test: 150 random moves, 0 failures, 0 violations, min clearance 28 mm (117 of them needed a detour); stepping the real animation loop for 60 moves (3983 frames): 0 frames with structure in the sphere.
Only the tool may enter the sphere; self-collision between the arm links and the small vertical offsets between stacked links are not modelled.
