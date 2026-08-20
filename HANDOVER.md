# camera-tracking — Handover

**Author:** Maya Kundakci (ext) · **Date:** 20 August 2026
**Repos covered:** `camera-tracking`, `Munera/{ludus, mulib, vcpkg-registry}`

Validate the positions the FANUC controller reports against independent OptiTrack
measurement. Nothing in the code is written in those terms, though — the general form
is *any object with two or more independent pose estimates, and the differences between
them*. The rotor and the hand are the first two instances, not special cases.

---

## 0. Read this first

### One thing the build will not find on its own

- **The NatNet SDK** is not vendored and not in vcpkg. CMake fails hard with an explicit
  message if `NATNET_SDK_DIR` is unset. Mine points at
  `C:/NatNet_SDK_4.5_windows/NatNetSDK` via `CMakeUserPresets.json`, which is
  machine-local — yours will differ.

---

## 1. What I built

### camera-tracking — effectively all of it


#### The measurement core — `nodes/common/`

Header-only, pure math over Eigen, no Qt / eCAL / JSON below the config layer, so each
piece is testable in isolation and includable from every target. Each file opens with a
long design rationale — read those before changing anything they touch.

| File | What it does | Why it exists |
|---|---|---|
| `frames.hpp` | Frame-tagged transform registry over Eigen. `T_to_from` naming, checked composition, route-finding, SI enforcement. | Eigen will happily compose transforms backwards and hand back a confidently wrong answer. Here the frame is part of the value, so that mistake throws at the call site instead of quietly biasing every output number. |
| `scene_config.hpp` | Loader + strict validator for the scene manifest and per-object files. mm/deg → SI conversion happens here, once. | Swapping a rotor, adding a tracking system, or bringing the render up before hardware exists are config edits, not code changes. |
| `object_placement.hpp` | Turns mocap frames and published poses into registry edges. Latch policy with outlier rejection, dropout handling, threading. | The anchor latch is not special-cased: it is a placement with `"capture": "latched"`. One rule, no anchor-shaped exception. |
| `two_mount_pose.hpp` | Builds one large part's pose from two rigid bodies on it — the rail and the rotor, same operation. Three lever-arm-free CAD checks fall out. | Offsetting from a single body puts all three rotational DOF on the answer at full lever arm, and a small marker triangle constrains orientation worst. |
| `joint_projection.hpp` | Forces an independently measured child-link pose onto its revolute joint via closed-form swing-twist. Reports radial and axial geometry error separately. | Three bodies on a 3-DOF hand is 18 numbers where 3 are real. Chaining through the joints converts that redundancy into an error estimate instead of noise. |
| `comparison.hpp` | The project's actual output: a delta per pair of placements on an object, time-matched when both sides are continuous. | A hand at 500 mm/s with 20 ms of thread skew fabricates 10 mm of "error" — the same order as the quantity being measured. |

#### Processes

- **`backend/`** — NatNet client, the placement/comparison/joint-estimate pipeline, and a
  FANUC stub publisher. Owns *all* frame math; publishes resolved poses so nothing
  downstream does coordinate arithmetic.
- **`frontend/`** — Qt Quick 3D viewer and the three-screen operator workflow, built on
  mulib. `RobotScene` is a pure viewer: every pose arrives already in the anchor frame.
- **`nodes/logger_node/`** — CSV recorder, driven by the UI's `session/control` topic so
  the log holds the validation run and not the setup that preceded it.
- **`launcher/`** — `cameratracking.exe`. A Windows job object with `KILL_ON_JOB_CLOSE`,
  children created suspended, any child's exit ending the session. An orphaned backend
  does not look broken; it looks like a running system publishing from stale config, and
  a second one alternates two confident answers at the frontend.
- **`middleware/`, `proto/`** — eCAL protobuf topic wrapper and the seven-message wire
  protocol.

#### Tooling and tests

- `calibration/inspect_mesh.py` — measures what an STL's origin and axes actually are and
  checks them against the P2D2 rotor convention.
- `calibration/recentre_mesh.py` — moves an STL's origin onto its own axis of revolution
  as a pure, recorded translation.
- `calibration/merge_urdf_subtree.py` — bakes a URDF link subtree into one STL at its
  home pose, composing joint *rotations* rather than summing translations.
- `scripts/flatten_xacro.py` — xacro expansion without a ROS install.
- `tests/` — seven GoogleTest binaries. Notable: a brute-force scan distinguishing the
  closed-form joint solve from a plausible-but-suboptimal one, and a test for the claim
  the whole anchoring scheme rests on — that Motive's world origin cancels out.

### Munera/ludus — the robot description layer

- **URDF importer + forward kinematics** (`2de78a6`) — `UrdfImporter`,
  `ForwardKinematics`, and the `RobotData` model they share.
- **FK and URDF improvements** (`a93f900`) — reworked FK onto KDL with a conversions
  shim; added `RobotScene`.
- **SDF support via libsdformat** (`806b343`) — `SdfImporter` / `SdfScene`, resolving
  `model://` mesh URIs file-relative with env vars as override only, mirroring URDF
  `package://` handling.

The payoff for camera-tracking is `ludus::build_robot_scene()`, which returns visuals
already world-posed — FK composed with each visual's `<origin>`, mesh path and scale
resolved. The frontend only maps its output onto mulib rows.

### Munera/mulib — the multi-model 3D stack

- **Multi-model rendering + `mu_robot`** (`ebdf17e`, ~1200 lines) — `MuOrbitViewport.qml`
  (shared orbit/pan/zoom with bounding-sphere framing), `MuMultiModelView.qml`
  (`Repeater3D` driven by a list model, per-row mesh/pose/scale/colour/visibility),
  `CadSceneGeometry` extended to load binary and ASCII **STL** directly with a parse-once
  cache and submesh extraction, `MeshInstanceTable` for many copies of one mesh, and the
  new `mu_robot` library (`RobotVisual` / `RobotVisualModel` + QML wrappers).
- **Per-visual colour** (`f416b9d`) — what lets one object render twice in two colours,
  which is how a measured-vs-expected pair is shown.
- **Quaternion trackball orbit** — the camera used to accumulate Euler angles and
  gimbal-locked at ±90° pitch. Rewritten to compose local-frame quaternion deltas. Landed
  under the shared `Co-ops` account (`f2a109a`, `5fb182e`, with zoom-to-cursor).

This is shared library code: it affects every Mu 3D viewer on rebuild.

---

## 2. How it works

### One transform in the whole system

The render frame is the **rail origin**. Robot-side data arrives rail-relative already, so
it needs no transform; OptiTrack data needs exactly one, latched at startup.

The single assumption is that the rail origin means the same physical point to both
systems. It is falsifiable: a rail-origin error has a constant, distance-linear signature,
while kinematic error varies with arm configuration — which is why arm configuration is
logged beside every delta.

No placement is pre-composed into anchor space; each registers its edge to the mocap frame
and the registry routes queries through the anchor. So if Motive is recalibrated and its
world origin moves by some unknown `M`, the two `M`s meet as `M⁻¹·M` in any anchor-relative
query and cancel exactly. There is no stored world origin to go stale.

### A placement is one claim, and a claim is a frame

An object has a mesh and one or more **placements**, each one claim about where it is. A
rotor the cameras measure and the same rotor as the robot reports it are two placements of
one object — and therefore two frames, `rotor_opti` and `rotor_expected`.

**The placement id *is* the frame name**, so ids must be unique across the entire scene;
the loader enforces it. The consequence is the good part: comparisons need no
configuration. Any object carrying 2+ placements yields a delta per pair, and the
comparison code never learns what a rotor or a hand is.

### Seven ways a placement gets its pose

| Source | Pose comes from |
|---|---|
| `optitrack` | A tracked Motive rigid body, by asset id. |
| `expected_pose` | A pose the robot publishes, already in the parent frame. Names the *role*, not the producer — the topic is config, so replacing the robot stack touches no code. |
| `joint_state` | Joint angles posed through the URDF by ludus FK. |
| `static` | A constant from the config file. Nothing measures it. |
| `fused` | The average of two or more other placements. |
| `projected` | Another placement's measurement forced onto a revolute joint hanging off the parent frame. |
| `constructed` | One pose built geometrically from two mounts on a common face. |

Orthogonally, **capture** is `latched` (measured once at startup, from the first frame
where the body is genuinely well-tracked, then never updated — correct for anything bolted
down) or `continuous`.

### Units: SI everywhere, with exactly two boundaries

Metres and radians internally, without exception. Millimetres and degrees exist only where
a human or a wire is involved: **config files are authored in mm/deg** and converted once
on load, and **published deltas and joint estimates are in mm/deg** because they are the
human-facing end. Nowhere else.

### Latched pairs get a review gate

A continuously tracked pair is a stream you can watch and average, so a bad one announces
itself. A latched pair is a single claim decided at startup — if it is wrong, you find out
by never finding out. So the frontend holds the session at a review gate on exactly those
deltas before tracking begins. Abandoning exits with code **2**, which the launcher reports
distinctly, because "these numbers are wrong, I'm recalibrating" and "it crashed" call for
opposite responses.

The gate is driven by `ObjectDelta::review_gated`, derived purely from both sides being
latched. Add another bolted-down object and it is reviewed too, for free.

---

## 3. Build & run

### Prerequisites

- Visual Studio 2022 toolchain, CMake ≥ 3.25, Ninja
- The NatNet SDK (4.5 is what I used), installed anywhere — you pass its path in
- The Munera checkout at `../Munera`
- vcpkg is vendored at `./vcpkg`; the preset already points the toolchain at it

### Configure and build

First configure is slow: vcpkg builds Qt, OpenCASCADE, sdformat and the gz stack.
Installed packages land in `out/vcpkg_installed`, binaries in
`out/build/<preset>/<config>/`.

### Tests

```
ctest --preset test-debug
```

**Preset mismatch, worth tidying:** `test-debug` is wired to the `windows-msvc-debug`
configure preset, not the `-local` one, so it expects a build tree configured with
`NATNET_SDK_DIR` supplied some other way. Either add a `test-debug-local` testPreset to
`CMakeUserPresets.json` or run the test binaries out of the build directory directly.

### Running a session

Launch **`cameratracking.exe`**, never the three executables by hand. It starts `backend`,
`logger_node` and `frontend` under a job object and guarantees none can outlive the
session — including if the launcher itself is killed outright. It also runs the backend
and logger with the *repo root* as working directory, which is how `config.json` and the
`Rendering/` mesh paths resolve.

Motive must be streaming, with rigid bodies defined and pivots set (§4).
`optitrack.required` in `config.json` controls whether the backend refuses to start
without it.

### The three screens

Forward only — there is no back navigation, because the placements reviewed on screen 2
are latched and cannot be re-measured without restarting the backend.

- **Home** — waits until every gated delta has a reading, or there is nothing gated.
- **Rotor Placement** — the review gate. Shows latched deltas with a severity ramp; accept
  or abandon. Skipped entirely when the manifest holds only continuous objects.
- **Position Tracking** — the live comparison readout. CSV logging runs exactly while this
  screen is active and the operator's toggle is on.

A full-width banner sits above all three whenever joint state is absent: a robot posed
from no data looks identical to one posed from real data, so the arm sitting at its home
pose must never be mistaken for a measurement.

---

## 4. Configuring a scene

```
config/
  scene.json          which placements are active, and which anchors the world
  objects/
    rail.json         one file per PHYSICAL object
    hand_left_j8.json
    rotor_a.json
    rotor_b.json      sitting unlisted until the day you need it
config.json           runtime knobs: topics, rates, thresholds, tolerances
```

### Why one file per object

An object's mesh and its mount geometry must travel together. Split across files, you can
pair Rotor A's geometry with Rotor B's mount offsets: it renders plausibly, and is wrong by
exactly the difference between the two mounts. **Swapping `"rotor_a"` for `"rotor_b"` in
the manifest is the entire procedure for changing rotors.**

### There is no calibration step, by design

There is no mount-offset field between a tracked body and its placement: Motive's rigid-body
frame **is** the placement's frame. Each pivot is set by hand in Motive at a point CAD knows,
so Motive's body origin already *is* the point the config describes. The rail and the rotor
go further and derive their frames from two bodies each, which lets them state geometry as
two CAD points rather than as an offset nothing could check. Nothing is fitted, and there is
no script to run before a session.

### The currently active scene

`scene.json` is set up for the **hand joint test**: world anchor `rail_origin`, with three
hand plates live on Motive streaming ids 1, 2 and 3, projected onto the J8 and J9 hinges.

| Object | Placements | Source / capture |
|---|---|---|
| `rail` | `rail_origin` | static — still an identity anchor, not yet measured |
| `hand_left_base` | `hand_left_base_opti` | optitrack / continuous |
| `hand_left_j8` | `hand_left_j8_opti`, `hand_left_j8_projected` | optitrack / continuous, projected |
| `hand_left_j9` | `hand_left_j9_opti`, `hand_left_j9_projected` | optitrack / continuous, projected |
| `hand_left_fado` | `hand_left_fado_mount`, `hand_left_fado_posed` | static, joint_state (dark until the robot bridge publishes) |

Two fallbacks are written and sitting unlisted. `hand_test` draws with no Motive and no
markers at all, answering only "do the meshes load and sit sensibly". `stub_demo` drives an
object from the simulated FANUC publisher, exercising the placement and comparison path end
to end with no hardware.

### Adding an object

1. Write `config/objects/<name>.json`: an id, display name, mesh path relative to the repo
   root, and one entry in `placements` per independent claim about where it is.
2. Add `"<name>"` to the `objects` array in `scene.json`.
3. Run it. Comparisons, review gating, render rows and CSV columns all follow
   automatically — no code, in either process.

Validation is strict and accumulates: a missing mesh, an unknown asset id, a manifest
naming a file that isn't there all fail at startup naming the offending file and key, and
they report together so a broken config is fixed in one pass instead of one round-trip per
typo.

### Meshes

STL carries triangles and nothing else — origin, units and spin axis are export-time intent
the format does not record. Before trusting a new mesh:

```
python calibration/inspect_mesh.py <file>
```

It measures all three from the geometry and checks them against the P2D2 convention (X down
the spin axis, origin on the axis at the axial midpoint).

Where an export landed on the wrong datum, `recentre_mesh.py` fixes it as a pure, recorded
translation. Both files are kept and the offset is written into the object's config as
provenance — `PSCDisc.stl` stays as the only record of what CAD actually produced, so "the
export datum is still wrong" stays visible instead of papered over. **Re-run it after every
CAD re-export**, and never hand-edit the derived file.

---

## 5. Reading the output

### eCAL topics

| Topic | Message | Publisher |
|---|---|---|
| `scene/placements` | `ScenePlacementsPacket` | backend — the whole scene in one message, republished periodically because eCAL has no latched topics |
| `scene/comparisons` | `ComparisonPacket` | backend — the headline deltas |
| `scene/joint_estimates` | `JointEstimatePacket` | backend — per-joint angle, geometry checks, controller comparison |
| `robot/joint_state` | `JointStatePacket` | *the robot bridge — nothing publishes this yet* |
| `pose_fanuc` | `PosePacket` | the FANUC stub |
| `hand/joint_state_measured` | `JointStatePacket` | backend — the cameras' own answer, in the controller's format |
| `session/control` | `SessionControlPacket` | frontend — the one topic that flows the other way |

### Numbers that matter

- **`spread_mm` vs `std_err_mm`** — not the same thing, and both are published. `spread` is
  per-sample dispersion and does *not* shrink with N; it is the stationarity evidence, i.e.
  was the object holding still. `std_err` is spread/√N, the uncertainty of the committed
  pose. A slowly drifting object has a tiny std_err and a large spread. Without these, an
  error bar of 4 mm and a delta of 4 mm were indistinguishable.
- **`radial_error_mm` / `axial_error_mm`** — **read these before trusting any joint angle.**
  Both are independent of the joint angle, because a rotation changes neither a point's
  distance from its own axis nor its position along it — so they hold steady while the joint
  sweeps and a non-zero value is geometry, not motion. Radial means the axis is in the wrong
  *place*; axial means it points the wrong *way*. Separate because they have different fixes.
- **`chord_error_m`, `normal_disagreement_rad`, `chord_out_of_plane_m`** — the two-mount
  construction's checks, printed at startup. A constructed pose comes out smooth, stable and
  confidently misplaced when the CAD is wrong; nothing about it *looks* wrong. These three are
  measured at the bodies themselves with no lever arm, so unlike the pose they are directly
  comparable against the drawing.
- **`invalid_reason` / `skip_reason`** — never empty when the corresponding valid flag is
  false. An absent delta and a delta of zero must never look alike. Same for
  `reported_status`, which distinguishes the kinds of waiting on the robot bridge — absent,
  stale, describing the other arm, or disowning its own sample. Today "absent" is expected.
- **`time_gap_s`** — how far apart the two matched samples were; 0 for latched pairs. If it
  creeps up, the deltas are measuring thread skew as much as position.

### The CSV

`logs/comparisons_<timestamp>.csv`, one row per delta per tick, 44 columns. Each row carries
the delta, full latch quality for both sides, the anchor's own pose and spread, the complete
arm configuration at that instant (rail position, J1–J6, `active_tool_frame`) and the UI
phase.

Arm configuration is in every row deliberately: it is what separates a rail-origin error from
real kinematic error, since the first has a constant signature and the second varies with
pose. `active_tool_frame` is there because a tool change silently shifts the reported point
and would otherwise corrupt the numbers invisibly.

---

## 6. Munera: mulib & ludus

Layering: **arena** (data/transport) → **ludus** (headless domain) → **mulib** (Qt/QML
presentation).

| Repo | What it is | Targets |
|---|---|---|
| `arena` | protobuf/buf protocol, model, eCAL transport. camera-tracking does not use it. | `Arena::Protocol`, `::Model`, `::TransportEcal` |
| `ludus` | Headless geometry / CAD / robot services. No Qt. URDF+SDF import and FK live here. | `Ludus::Core`, `::Occt`, `::Robot` |
| `mulib` | Qt/QML presentation. Seven libs: mu_core, mu_2d, mu_model, mu_cad, mu_qml, mu_robot, mu_middleware. No Arena dependency. | see §8 |
| `vcpkg-registry` | Private git registry with ports for all three. | — |

### How camera-tracking consumes them

**Local source by default**, via `MUNERA_DIR` + `add_subdirectory`, with
`LUDUS_BUILD_CORE/OCCT/EXAMPLES` off. Chosen because both libraries are under active local
development — the registry would force a commit, push and SHA bump per change — and because
of the stale refs in §8.

The registry path still exists and is one flag: `-DMUNERA_FROM_REGISTRY=ON` plus the
`munera-registry` manifest feature. `CMakeLists.txt` deliberately aliases the local targets
and sets the same QML-import-path variables the installed package would define, so
`frontend/CMakeLists.txt` is byte-for-byte identical in both modes. Keep that property.

Both are behind the `Qt6_FOUND` guard — only the frontend needs them. The backend reads no
URDF at all: a projected placement's joint geometry is measured CAD written straight into
config, not derived from a robot description.

### The parts of mulib worth knowing

- **`MuMultiModelView`** — a `Repeater3D` over a list model, one `Model` per row with its own
  mesh path, object id, position, rotation quaternion, scale, colour and visibility. This is
  what you render N objects with.
- **`MuOrbitViewport`** — shared orbit/pan/zoom with bounding-sphere framing and a quaternion
  trackball camera. Trade-off: a trackball has no locked up-vector, so the horizon rolls over
  time. Accepted deliberately in exchange for pole-free 360° tumbling.
- **`CadSceneGeometry`** — `QQuick3DGeometry` that loads `.stl` (binary and ASCII) and the
  `.mumodel` codec by path, with a parse-once cache and submesh extraction by object id.
- **`mu_robot`** — `RobotVisualModel` is a ready-made `QAbstractListModel` for
  `MuMultiModelView`, with `setVisuals` / `updateTransform`. This is the seam
  camera-tracking's `RobotScene` plugs into.
- **`MuModelView`** — the old CPU rasteriser. Still present in `mu_qml`, not retired, and
  *not* what this project uses. Don't build on it.

---

## 7. Where it stands

### Working today

- The full pipeline compiles and runs: three processes, real Motive data, real CSV output.
  Logs from 13 August show sub-2 mm agreement on the J8 projection.
- Scene config, placement, comparison, joint projection, two-mount construction — all
  implemented and under test.
- The three-screen UI with the review gate, severity ramp and Siemens Energy palette.
- URDF and SDF import plus FK in ludus; multi-model Quick3D rendering in mulib.

### The critical path — BLOCKED

**The dimachaerus C# eCAL bridge.** Nothing publishes `robot/joint_state`. Until it does,
the controller side of every comparison is dark: the hand renders from the cameras rather
than from what the robot claims, and `expected_pose` / `joint_state` placements have no
input. It lives in another codebase and it is the single thing gating the project's actual
purpose.

The consuming side is finished and waiting. It needs joint state, TCP, `active_tool_frame`
and rotor pose, all rail-relative. The `reported_status` plumbing already distinguishes
every kind of waiting, so bring-up should be diagnosable rather than silent.

### Auto-config

You need to run the software many times to start the process.
1. to get the normal axis of rail
2. to get normal axis of rotor and position of mount b (lower mount)
3. to get zero of j8
4. to get zero of j9
Then you are ready to run. This could be streamlined.


## That's it!

you can refer to docs\system-test.md for a full system run for the P2D2 software. 
Feel free to reach out to me if you have any questions

(412)9803142 or maya.kundakci@gmail.com

