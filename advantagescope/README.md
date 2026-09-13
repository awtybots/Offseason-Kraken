# AdvantageScope custom assets

Point AdvantageScope at this folder once:

**App → Use Custom Assets Folder…** → select `Offseason-Kraken/advantagescope`

Everything in here is then version-controlled with the robot code, so the whole team gets the
same model. The built-in 2026 field still loads normally — this folder only *adds* assets.

## What goes in `Robot_Kraken/`

Already in place, exported 2026-09-12:

| File | Was | What it is |
|---|---|---|
| `model.glb` | `Drivebase+StationaryFixed.glb` | chassis + bumpers + everything that never moves |
| `model_0.glb` | `TurretOnly.glb` | turret **with the hood baked in** (the hood does not articulate) |
| `model_1.glb` | `LinearSlideIntake.glb` | the linear slide intake |

AdvantageScope requires exactly these names, which is why they were renamed.

Measured off the exports (world-space, after composing the node transforms):

- **Units are metres**, colours survived STEP → glb (74 of 75 materials are non-white).
- The glb is **Y-up**, origin horizontally centred on the robot, floor 0.101 m below the origin
  — hence `"position": [0, 0, 0.101]`.
- The robot faces **−X** in Onshape, so the second rotation is `z:180`, not the stock `z:90`.
  This was derived from the turret: its centre lands at (+0.121, −0.187) under `x:90` alone,
  against a measured `TURRET_OFFSET` of (−0.125, +0.201) — same magnitudes, both signs flipped.
- **Bumper box 0.817 × 0.944 m (32.2 × 37.2 in)**, now in `SimRobot.SimConstants`. That is
  ~6.1 in per side outside the swerve module span, which is module inset plus bumper. The
  "29.5 long x 24.5 wide" comment in `SwerveSubsystem` is stale and disagrees with both.

`model.glb` is **53 MB / 4.3 M triangles / 2060 meshes**, against 11–33 MB for the models that
ship with AdvantageScope. It will load slowly. If that bothers you, suppress the fasteners in
Onshape and re-export — they are almost certainly most of those 2060 meshes.

## Onshape → glb

Onshape's own glTF export produces enormous files. Go through STEP instead:

1. In Onshape, right-click the assembly (or the sub-assembly for a component) → **Export** →
   format **STEP**.
2. Open the STEP in [CAD Assistant](https://www.opencascade.com/products/cad-assistant/) (free).
3. **Save as** → `.glb`, and tick **"Merge faces within the same part"**.

Both steps can take several minutes on a full robot assembly.

## Calibrating the transforms

The transforms in `config.json` were derived from the model geometry, so they should be close.
Verify and nudge in this order — rotations always before positions.

1. In the 3D Field tab, set the field to **Axes** so you can see the origin.
2. Drag `AdvantageKit/RealOutputs/Drive/Pose` in as the robot, and pick "Kraken 5829" as the model.
3. **The 10-second check on `z:180`:** does the intake end point the way the robot drives? If the
   robot moonwalks, change `z:180` to `z:0`.
4. Check the wheels sit on the floor rather than sunk into it or hovering. That is `"position"`
   `[0, 0, 0.101]`.
5. Drag `AdvantageKit/RealOutputs/Components/Zeroed` onto the robot object and set the object type
   to **Component**. Both parts jump to wherever their `zeroedPosition` puts them.
6. Adjust each component's `zeroedPosition` until its pivot sits on the origin. Component order is
   turret, then intake. The turret's is derived from `TURRET_OFFSET`, so it should already be
   right; the intake's is a bbox-centroid guess and is the more likely of the two to need nudging.
7. Swap to `Components/Measured`. The turret should yaw with the real turret and the intake should
   slide along +X as the pushout extends.
8. Mirror the **negatives** of whatever `zeroedPosition` values you land on back into
   `SimRobot.SimConstants` (`TURRET_PIVOT`, `INTAKE_HOME`) so the model and the published poses
   agree.

`INTAKE_TRAVEL_M` is still a 12 in guess — set it to the slide's real stroke at
`PUSHOUT_EXTENDED_POS`.

AdvantageScope re-reads the folder when you switch models, so no restart between edits.

## Fixed cameras

The three Limelight entries in `config.json` all sit at the origin with placeholder angles.
Fill in the real mounting positions and they become selectable camera views (right-click the
3D view → Fixed Camera), which is the fastest way to sanity-check what each camera can see.

## Note on the field model

AdvantageScope's 2026 field is the **welded** field. `Constants` uses the **AndyMark** layout,
which is ~14 mm shorter and ~26 mm narrower. Expect sub-inch misalignment when overlaying
AprilTag poses. It does not matter for driving around.

## Driving it (keyboard)

`simgui-ds.json` gives Keyboard 0 a full six-axis Xbox layout, so the sim uses the **same**
drive command and the same bindings as the real robot. Keyboard 0 is already assigned to
Joystick 0.

| Key | Xbox equivalent | Does |
|---|---|---|
| `W` `A` `S` `D` | left stick | translate, field-relative |
| `Q` / `E` | right stick X | rotate |
| `Space` | right trigger | **shoot** (spins up, aims hood, feeds when ready) |
| `F` | left trigger | extend the intake slide + run the intake |
| `R` | left bumper | retract the intake |
| `Y` | right bumper | unjam (reverse kicker / conveyor / rollers) |
| `Z` | A | outtake |
| `X` | B | aim the turret at the hub |
| `O` | start | zero the gyro |
| numpad `4` | POV left | drive-to-pose |
| numpad `8` | POV up | **reset fuel** (respawn the field, empty the hopper, zero scores) |
| numpad `6` | POV right | **preload fuel** (fill the hopper) |
| `U` | back | reset the robot to its start pose |

The POV on Keyboard 0 is the **numpad**, not the arrow keys. On an Xbox controller these three
are just the D-pad and the Back button, and they work identically.

The turret and hood aim at the hub on their own in simulation (they are set as default commands
under `isSimulation()` only), so you do not have to hold `X` to shoot.

**To test shooting:** press D-pad right (numpad `6`) to fill the hopper, drive into your
alliance zone (blue is x < 4.01 m), and hold `Space`. Outside that zone `ControlAllShooting`
switches to ferry mode and aims at the ferry target instead. D-pad up (numpad `8`) puts the field back.

**To test intaking:** hold `F` to extend the slide and run the intake, then drive over the fuel
pile at midfield. `Sim/FuelStored` counts up.

## Things the simulation does not model

- **No field collision for the robot.** The pose is integrated odometry, so you can drive
  straight through the guardrail and the hub. Only *fuel* collides with the field.
- **No SPARK closed loops.** Turret and hood slew kinematically toward their setpoints, so the
  sim exercises the aiming math, not the tuning. The shooter flywheel and the pushout *are* real
  physics models.
