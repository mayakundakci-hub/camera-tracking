# Full system test — 7 rigid bodies

Written 2026-08-20 for the run on the 21st. Places the rail and the rotor from
mocap, and drives the drawn hand from camera-measured joint angles.

The scene is already written and validated. What is *not* settled is three
numbers that only the running rig can supply, so the day is structured as three
passes: **measure the geometry, capture the hand's zero, then run.** Each pass
adds exactly one set of constants, so if something breaks there is only one
place it can have come from.

---

## The seven bodies

| Motive id | Body | Object | Capture |
|---|---|---|---|
| 1 | green plate | `hand_left_base` | continuous |
| 2 | blue plate | `hand_left_j8` | continuous |
| 3 | purple plate | `hand_left_j9` | continuous |
| 4 | pink, rail −Y end | `rail` | latched |
| 5 | blue, rail +Y end | `rail` | latched |
| 6 | top clamp | `rotor` | latched |
| 7 | bottom clamp | `rotor` | latched |

Every one of these has its pivot set **by hand in Motive at a CAD point**. There
is no mount-offset field to correct one that is not — Motive's rigid-body frame
*is* the placement's frame, so the pivot is the calibration. If anyone redefines
a body or resets a pivot, the numbers in `config/objects/` stop describing it and
nothing in the output will say so.

That covers the pivot's **position**. Its **orientation** is a separate claim and
nothing here checks it either: these bodies are world-aligned, and every URDF in
the tree is Z-up while Motive is Y-up. Where a body frame meets a link frame,
that right angle is owed — see `hand_mount` in `hand.json`.

---

## Before you start

1. **Purple plate needs a fourth marker.** It has three, and Motive's *minimum
   markers to boot* for it reads 4, so as configured it can never acquire.
   Add a marker, or lower the threshold to 3 and accept that one occlusion
   loses the body.
2. **Recreate the three hand plates in ONE sitting**, with the hand held still,
   all world-aligned. This is not optional housekeeping — it is the fault that
   produced −41.14° of outstanding turn on J8 against −50.28° on J9 when those
   two should have been equal. Creating them together is what makes
   `zero_pose`'s identity rotation a true statement.
3. Check Motive's **Data Streaming** pane: `Local Interface` must match
   `optitrack.motive_server_ip` in `config.json` (both `127.0.0.1` today).
4. Set `optitrack.required` to `true` in `config.json` so the backend refuses
   to start without cameras rather than coming up dark.

```
python scripts/check_scene.py
```

Should print `ok: 6 objects, 13 placements, 7 Motive bodies (1..7)`.

```
cmake --build --preset build-debug-local
```

---

## Pass 1 — measure the two geometry constants

Run `cameratracking.exe` and let it sit until the latched placements report.

The backend prints a **`[mounts]`** block for each constructed placement, once
per session. It gives you two things you cannot get from a drawing:

### `rotor_opti` — take both lines

```
[mounts] rotor_opti: 'rotor_opti_a' -> 'rotor_opti_b' is NN.NNN deg about [...]
[mounts] rotor_opti: paste into 'construction' --
           "normal_axis": [...],
           "mount_b": { "position_mm": [...] }
```

Paste **both** into `config/objects/rotor.json`. The 90° separation in there now
is a placeholder and is certainly wrong — this is the measurement that replaces
it. It keeps mount_b's own radius and axial offset and changes only the angle,
which matters because the two clamps sit at different stations (334.8 mm and
354.8 mm).

Then read the cross-check line directly underneath:

```
[mounts] rotor_opti: cross-check -- measured chord X mm against Y mm predicted (+Z mm)
```

Those two routes are independent — one through the bodies' orientations, one
through their positions. If they disagree by more than 5 mm the backend says so
and tells you what it means: most likely a clamp that can spin where it is
tightened, in which case its orientation records how it was tightened rather
than where it sits, and the chord is the number to trust. Counting teeth
between the clamps settles it independently of both.

### `rail_origin` — take **only** `normal_axis`

Ignore the `mount_b` it prints for the rail. That line is derived by rotating
mount_a about the part's axis, which only means anything for a rotationally
arranged part. The two rail bodies sit at the same attitude, so it comes back
with both mounts at the same end of the rail. The rail's mount points are CAD
and stay CAD.

**Stop the app, paste, restart.**

---

## Pass 2 — capture the hand's zero

With the geometry settled, park the hand at the pose you want to call zero —
its physical zero, the one the controller means.

Set in `config.json`:

```json
"joint_projection": { "capture_zero": true }
```

Run for ~10 s. Each joint prints **both halves** of its zero pose:

```
[joint] hand_left_j8_projected: if THIS pose is the joint's zero, replace BOTH halves --
           "zero_pose": { "position_mm": [...], "quat_wxyz": [...] }
```

Paste both. They only mean anything together: the rotation says *which* pose is
θ = 0, the position says where the child's origin sits *when it is*. One without
the other describes two different configurations, and the residual cannot see it.

**J8 first, then restart, then J9.** J9's projection hangs off J8's *corrected*
pose, so a zero captured for J9 while J8 is unsettled is measured against a
moving datum. The backend prints that warning itself when it sees a projected
parent.

Set `capture_zero` back to `false`. Left on, it re-measures against whatever
pose the next session happens to start in.

---

## Pass 3 — the run

Restart. You should see, in order:

1. **`[scene]`** — 6 objects, anchor `rail_origin`.
2. **`[latch]`** — rail and rotor committing, with a `spread` and a `std_err`
   for each. Read `spread`, not `std_err`: spread does not shrink with N and is
   the stationarity test, i.e. whether the thing was actually holding still.
3. **`[construct]`** — three geometry checks per constructed placement:
   - `chord` — measured distance between the bodies vs the CAD distance. The
     headline check. On the rail's 1651 mm baseline it catches a body at the
     wrong station; on the rotor it catches a miscounted groove.
   - `normal disagreement` — the two mounts' face normals. Zero if both seat
     flat on one plane.
   - `expect_normal_in_parent` — the only check that is not the part checking
     itself. **Near 0° means right, near 180° means the sign is flipped.** It
     fails by right angles, not by degrees, so do not tune the threshold.
4. **`[joint]`** — per-joint `theta`, `radial`, `axial`, `residual`.

### What to read first

**`radial` and `axial`, before any angle.** Both are independent of the joint
angle — a rotation changes neither a point's distance from its own axis nor its
position along it — so they hold steady while the joint sweeps, and a persistent
non-zero value is geometry, not motion. Radial means `axis_point` is in the
wrong *place*; axial means `axis` points the wrong *way*. If either is over
threshold, no angle from that joint is worth anything and there is no point
looking at the rest.

### On screen

- The **rail** draws from `Rail.stl` at the constructed anchor.
- The **rotor** draws as a 2.21 m × 1.61 m disc, spin axis along the rail.
- The **hand** draws as six connected links riding the green plate, its joints
  moving as you move the real hand.

Sanity-check the render, not just the numbers. A rail rolled 90° about its own
length passes every internal check — chord compares lengths, and both candidate
axes are perpendicular to the vertical — and the render is the only thing that
shows it.

**But read a 90° in the render carefully before blaming the anchor.** The tracked
bodies are world-aligned and Motive is Y-up; every URDF here is Z-up. Any place a
body frame meets a link frame therefore owes a right angle, and it is carried by
the placement that joins them — `hand.json`'s `hand_mount` is the one for the
hand. An identity there stands the hand on end, and it looks *exactly* like a
rolled anchor. The two are told apart from the backend rather than the screen: a
rolled anchor shows up as ~90° on `expect_normal_in_parent`, and it moves
**every** object. A missing joining rotation leaves that check clean and moves
only its own object.

### Logging

The CSV writes exactly while the **Position Tracking** screen is up and the
operator toggle is on. `logs/comparisons_<timestamp>.csv`, one row per delta per
tick.

---

## What is still open, and what it costs

| Open | Effect | Blocks today? |
|---|---|---|
| Rotor clocking separation | Rotor lands in the wrong place around its axis | No — pass 1 measures it |
| Hand `zero_pose` rotation | Every θ offset by one constant; changes are still right | No — pass 2 captures it |
| `hand_mount` **position** (green pivot → URDF link origin) | Drawn hand shifted bodily; no angle affected | No — cosmetic |
| `hand_mount` **rotation** yaw | Drawn hand upright but heading may be a right angle out | No — turn it 90° about the plate's own Y and re-look |
| Robot bridge (`robot/joint_state`) | Controller side of every comparison is dark | **The project's actual purpose** |
| Rail baseline is 1651 mm, not 10 m | Anchor orientation pinned to ~0.035° instead of ~0.006°; ~1.8 mm at the rotor | No — but moving the bodies apart is the cheapest accuracy anywhere in this system, and it multiplies into every object because this is the anchor |

The rotor comparison has **no error bar** — it is one latched sample per
session. The hand's is a continuous stream. If the two disagree, the difference
is attributable to the rotor-origin convention, since dimachaerus laser-hunts X0
while this uses CAD mounts.

---

## If something does not place

`[placement] waiting on <id>` names it and says why. Accumulation never gives
up, so clearing the volume or re-seating a mount lets it latch without a
restart — `latch.timeout_s` only affects reporting.

A placement that fails its gate simply stays unplaced. Nothing renders for it
and no comparison is published; an absent delta and a delta of zero never look
alike.
