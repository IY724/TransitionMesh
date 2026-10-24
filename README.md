# Transition Mesh V1.0

Abaqus/CAE plug-in for parametric transition meshing in 2D and 3D. It supports 2:1 and 3:1 transitions, five geometry modes, multi-layer cascades, hybrid transitions and full-domain meshing.

**English** | [简体中文](README.zh-CN.md)

> This project was built with the assistance of AI-assisted programming (vibecoding).
>
> Non-commercial use is free. Contact the author before any commercial use.
>
> Feedback: open a [GitHub Issue](https://github.com/IY724/TransitionMesh/issues) or email **Z25150070@s.upc.edu.cn**.

## Contents

1. [Introduction](#1-introduction)
2. [Installation](#2-installation)
3. [Dialog and parameters](#3-dialog-and-parameters)
4. [Mode-by-mode guide](#4-mode-by-mode-guide)
5. [Troubleshooting](#5-troubleshooting)
6. [Scope and limitations](#6-scope-and-limitations)
7. [Contact and support](#7-contact-and-support)

## 1 Introduction

### 1.1 Overview

Transition Mesh is a pre-processing plug-in for Abaqus/CAE. It builds the transition between a fine-mesh region and a coarse-mesh region automatically. By hand, this usually means repeated geometry partitions, seeding every edge, and checking that the nodes on both sides match. The plug-in turns the process into fixed templates: it lays 2:1 or 3:1 quadrilateral or hexahedral elements in the transition band, maps N fine-side segments to N/2 or N/3 coarse-side segments, keeps one-to-one node correspondence on the interface, and produces no hanging nodes or triangles inside the band.

The plug-in works on 2D faces and 3D solids. In 3D, the transition band must sit on a solid cell you have already partitioned, and that cell must have usable sweep or structured meshing capability.

<table>
<tr>
<td align="center" width="58%"><img src="docs/images/ui-menu.png" width="100%"/><br/>Plug-ins menu entry and parameter dialog</td>
<td align="center" width="42%"><img src="docs/images/template-topology.png" width="100%"/><br/>Single-period 2:1 and 3:1 templates (schematic)</td>
</tr>
</table>

### 1.2 Five transition modes

| Mode | Geometry | Ratios | Direction | Multi-layer |
| --- | --- | --- | --- | --- |
| `PLANAR` | Straight-edge band, 2D face or source face of a 3D plate | 2:1, 3:1, hybrid | `Orientation` up, down, left, right | Yes; hybrid is single-layer only |
| `PLANAR_ANNULAR` | Planar concentric annulus | 2:1, 3:1 | Fine inside or outside | Yes |
| `PLANAR_QUAD_FOUR` | Rectangular inner frame expanding outwards | 2:1, 3:1 | Fine inside, coarse outside | 2:1 multi-layer; 3:1 single-layer recommended |
| `ANNULAR_RADIAL` | Cylinder end-face band along the radius | 2:1, 3:1 | Fine inside or outside | Yes; the wall must be pre-partitioned at intermediate radii |
| `AXIAL_SURFACE` | Cylinder barrel along the axis | 2:1, 3:1 | Fine loop to coarse loop | Yes; intermediate sections are cut automatically |

Choose the mode from the geometry of the transition region:

- Straight-edge band: `PLANAR`;
- Planar concentric annulus: `PLANAR_ANNULAR`;
- Full ring around a rectangular inner frame: `PLANAR_QUAD_FOUR`;
- Cylinder end face along the radius: `ANNULAR_RADIAL`;
- Cylinder barrel along the axis: `AXIAL_SURFACE`.

If more than one mode fits, for example around a hole in a plate, choose the annulus or the four-side mode according to the hole shape and the existing partitions.

### 1.3 Workflow

The plug-in follows a pick, validate, trial-mesh, execute workflow:

1. **Pick geometry**: choose the mode, pick the target faces and reference edges in the viewport, and confirm each pick with Done;
2. **Set parameters**: enter the fine-side segment count `Seeds`, the number of transition layers `Layers`, and the transition ratio `Ratio`;
3. **Check**: run a read-only validation of the input, divisibility and geometric plan. A FAIL blocks execution;
4. **Pre-mesh**: run a real trial partition, seed and mesh inside a model transaction, audit the mesh, then roll back automatically. The original model is unchanged;
5. **Apply**: choose the execution level under `Action` and execute.

A passing `Check` means the plan is valid. A passing `Pre-mesh` means the combination can be delivered. Changing any input makes the previous verdict stale; the status line shows `Check is stale`, and you need to run Check again.

Execution is protected by a model transaction. Any hard failure rolls the model back completely, leaving no partial partitions or seeds.

`Action` has two levels:

- `Partition + seed only (no mesh)`: partitions and local seeds only, for manual mesh-control adjustments or a second meshing pass;
- `Partition + seed + mesh`: partitions, seeds and generates the mesh in one step.

In 2D modes you can also pick `Full domain` to mesh the transition band and the connected regular regions in one pass.

## 2 Installation

### 2.1 Requirements

- Abaqus/CAE 2024 or later, verified on Abaqus/CAE 2025;
- Windows;
- Pure Python, loaded by the Abaqus built-in interpreter. No third-party libraries are needed.

### 2.2 Install steps

1. Close every Abaqus/CAE window;
2. Download the latest package from [Releases](https://github.com/IY724/TransitionMesh/releases/latest) and extract it;
3. Copy the entire `TransitionMesh` folder into the Abaqus plug-in search directory. All 13 scripts go directly inside that folder; do not split them into the parent directory and do not add another `TransitionMesh` subfolder. The usual location is `%USERPROFILE%\abaqus_plugins`; if you configured another directory with `plugin_path` or a shortcut argument, use that directory;
4. Start Abaqus/CAE and confirm that `Transition Mesh` appears in the `Plug-ins` menu;
5. Open the dialog once and confirm that the status line and parameter sections display correctly.

The installed layout should be:

```text
<Abaqus plug-in search directory>/
└─ TransitionMesh/
   ├─ transitionMesh_plugin.py
   ├─ transitionMeshDB.py
   ├─ transition_mesh_gui_kernel.py
   ├─ transition_mesh_plan.py
   ├─ transition_mesh_rules.py
   ├─ transition_mesh_runtime.py
   ├─ transition_mesh_regions.py
   ├─ transition_mesh_corner_templates.py
   ├─ transition_mesh_plugin.py
   ├─ transition_mesh_quad2.py
   ├─ transition_mesh_quad2_cae.py
   ├─ transition_mesh_plugin_planar2d.py
   └─ transition_mesh_plugin_cylinder_kernel.py
```

Keep exactly one complete copy in the plug-in search directory. The folder name must stay `TransitionMesh`, and all 13 scripts must sit directly inside it. Do not keep old versions, backups, or renamed copies beside it. If the menu entry is missing or a version conflict appears during loading, remove the duplicate copies and restart CAE.

### 2.3 Package layout

The plug-in consists of 13 Python modules, organised into interface, dispatch, rule, kernel and runtime-support layers:

| Layer | Module | Responsibility |
| --- | --- | --- |
| Interface | `TransitionMesh/transitionMesh_plugin.py` | Registers the menu and form, collects keywords, runs pre-execution validation |
| Interface | `TransitionMesh/transitionMeshDB.py` | Parameter dialog and pick handlers, Check polling, field linkage |
| Dispatch | `TransitionMesh/transition_mesh_gui_kernel.py` | Mode routing, Check and Pre-mesh dispatch, execution audit |
| Rules | `TransitionMesh/transition_mesh_rules.py` | Mode capability table, hybrid split, divisibility checks, angular fit |
| Rules | `TransitionMesh/transition_mesh_plan.py` | Count-chain planning, stage-width allocation, value search |
| Kernel | `TransitionMesh/transition_mesh_plugin.py` | Classic planar templates, sketch drawing, partition and seed primitives |
| Kernel | `TransitionMesh/transition_mesh_plugin_planar2d.py` | Planar annulus and four-side planning, partitioning, seeding and meshing |
| Kernel | `TransitionMesh/transition_mesh_quad2.py` | Four-side 2:1 reference topology and count chain |
| Kernel | `TransitionMesh/transition_mesh_quad2_cae.py` | Four-side 2:1 planning, partitioning, seeding, meshing and checks |
| Kernel | `TransitionMesh/transition_mesh_corner_templates.py` | Corner templates and multi-level rectangular-ring audits |
| Kernel | `TransitionMesh/transition_mesh_plugin_cylinder_kernel.py` | Cylinder radial and axial frames, template imprint, seeding and blockwise meshing |
| Runtime | `TransitionMesh/transition_mesh_runtime.py` | Model transactions, process log, result publication |
| Runtime | `TransitionMesh/transition_mesh_regions.py` | Region signatures, validation and batch relocation |

## 3 Dialog and parameters

### 3.1 Dialog layout

The dialog is divided into five areas, top to bottom:

1. **Mode**: the `Type` combo box switches between the five modes;
2. **Selections**: shows the faces, edges, cells and paths required by the current mode;
3. **Common parameters**: `Seeds`, `Layers`, `Ratio`; the four-side mode also shows `Ny`;
4. **Mode-specific parameters**: classic planar shows `Orientation`, `Depth` and `Hybrid`; cylinder modes show `Auto-adapt`;
5. **Execution area**: `Check`, `Suggest values`, `Pre-mesh`, `Action`, `Apply`, `Cancel`, plus the status line.

Switching `Type` hides the unrelated picks and parameters. Confirm each pick with Done; the pick row shows how many entities are currently selected.

### 3.2 Common parameters

| Parameter | Meaning | Notes |
| --- | --- | --- |
| `Seeds` | Fine-side segment count | Edge count in planar modes; loop count in annulus and cylinder modes; main-direction count `Nx` in the four-side mode; per-section circumference count in the axial mode |
| `Layers` | Number of transition layers | 1 for a single layer; greater than 1 for a cascade |
| `Ratio` | Transition ratio | 2:1 or 3:1 |
| `Orientation` | Transition direction | Classic planar only: `HU`, `HD`, `VR`, `VL` |
| `Depth` | Transition-band width | Used when no coarse edge is picked; the measured distance overrides it when a coarse edge is picked |
| `Ny` | Orthogonal fine count | Four-side mode only; `0` means the same as `Nx` |
| `Hybrid` | Mixed transition switch | Classic planar only, single layer, 2:1 and 3:1 mixed |
| `2:1 blocks b` | Number of 2:1 groups | Hybrid only; `b` must be even |
| `Auto-adapt` | Adapt adjacent regular regions | Cylinder modes only; seeds and meshes adjacent regular regions automatically |
| `Action` | Execution level | Partition and seed only, or partition, seed and mesh |

### 3.3 Ratio and divisibility

Let the fine-side segment count be `N`, the ratio be `r`, and the number of layers be `L`. The coarse-side count is:

```text
coarse count = N / (r ^ L)
```

Therefore:

- For 2:1, `N` must be divisible by `2^L`;
- For 3:1, `N` must be divisible by `3^L`.

Examples:

```text
2:1, N=40, L=1  ->  coarse 20 segments
2:1, N=40, L=2  ->  coarse 10 segments
3:1, N=36, L=1  ->  coarse 12 segments
3:1, N=36, L=2  ->  coarse 4 segments
```

Choose `N` as the fine-edge length divided by the target fine element size. For example, a 40 mm fine edge with a target size of about 1 mm needs `N=40`.

The four-side mode adds the two corner contributions to the coarse count; see section 4.3.

### 3.4 Orientation

`Orientation` in the classic planar mode is interpreted in the model's global coordinates, not in the rotated viewport:

| Option | Meaning |
| --- | --- |
| `HU` | Fine edge on top, band grows downwards |
| `HD` | Fine edge at the bottom, band grows upwards |
| `VL` | Fine edge on the left, band grows to the right |
| `VR` | Fine edge on the right, band grows to the left |

If the fine edge is an internal partition edge, the direction must point into the selected face. For example, when the upper half-face and the middle partition edge are selected, use `HD` to grow upwards; when the lower half-face and the middle edge are selected, use `HU` to grow downwards. If the direction points out of the face, the plug-in stops with an error instead of switching to the other side.

### 3.5 Check, Suggest values and Pre-mesh

- **Check**: read-only validation. It checks that the picks are complete, the segment counts divide, the template period is valid and the geometry is compatible. A pass shows `Check PASS`; a failure shows the reason and a correction hint.
- **Suggest values**: runs Check first. If the current parameters are invalid, it fills in a nearby legal set; if they are already legal, it says no change is needed.
- **Pre-mesh**: runs a real partition, seed and mesh in an isolated model transaction, checks element counts, hanging nodes, triangles and Jacobians, then rolls back automatically. A pass shows `Pre-mesh PASS`; a failure reports the reason and leaves the model unchanged.
- **Apply**: executes for real. Pass Check and Pre-mesh first; the final result matches the trial.

## 4 Mode-by-mode guide

### 4.1 PLANAR

Use for straight-edge transition bands, on a 2D face or the source face of a 3D plate.

#### Picks

| Pick | Required | Meaning |
| --- | --- | --- |
| `Partition Face` | Yes, single | The face that receives the transition mesh |
| `Finer Edge` | Yes, single | The edge on the fine-mesh side |
| `Coarse Edge` | Optional, single | The edge on the coarse-mesh side; if omitted, `Depth` sets the band width |
| `Full domain` | Optional, single | Regular region to mesh together in 2D; if omitted, only the transition band is processed |

A 3D plate still uses `PLANAR`. The transition block must be a sweepable solid cell. Partition the geometry beforehand, assign Sweep and the sweep direction to the transition block in the Mesh module, then run the transition partition on the source face.

#### Steps

1. Open the part, go to the Mesh module and set the global seed for the regular region. The global seed should represent the coarse size you want in the main body, not the minimum fine size;
2. Open `Plug-ins -> Transition Mesh` and select `PLANAR` under `Type`;
3. Pick `Partition Face` and confirm with Done;
4. Pick `Finer Edge` and confirm with Done;
5. Pick `Coarse Edge` if needed. If omitted, the plug-in computes the band width from `Depth`;
6. Pick `Full domain` if needed. If omitted, only the transition band is meshed;
7. Enter the fine-edge segment count in `Seeds`, the number of layers in `Layers`, and choose 2:1 or 3:1 in `Ratio`;
8. Choose the direction in `Orientation` and enter the band width in `Depth`. If a coarse edge was picked, `Depth` can be arbitrary because the measured distance is used;
9. Click `Check`, and `Suggest values` if needed;
10. Click `Pre-mesh` to trial-mesh and audit the quality;
11. Choose the execution level under `Action` and click `Apply` or `OK`.

<table>
<tr>
<td align="center" width="50%"><img src="docs/images/planar-dialog.png" width="100%"/><br/>PLANAR mode dialog</td>
<td align="center" width="50%"><img src="docs/images/planar-pick-region.png" width="100%"/><br/>Pick the transition face</td>
</tr>
<tr>
<td align="center" width="50%"><img src="docs/images/planar-pick-fine-edge.png" width="100%"/><br/>Pick the fine edge</td>
<td align="center" width="50%"><img src="docs/images/planar-pick-coarse-edge.png" width="100%"/><br/>Pick the coarse edge (optional)</td>
</tr>
<tr>
<td align="center" width="50%"><img src="docs/images/planar-pick-full-domain.png" width="100%"/><br/>Pick the full domain (optional)</td>
<td align="center" width="50%"><img src="docs/images/planar-params-counts.png" width="100%"/><br/>Set Seeds, Layers and Ratio</td>
</tr>
<tr>
<td align="center" width="50%"><img src="docs/images/planar-params-orientation.png" width="100%"/><br/>Set Orientation, Depth and Hybrid</td>
<td align="center" width="50%"><img src="docs/images/planar-check.png" width="100%"/><br/>Status line after a passing Check</td>
</tr>
</table>

#### Results

<table>
<tr>
<td align="center" width="50%"><img src="docs/images/planar-result-full.png" width="100%"/><br/>2:1 full-domain automatic mesh</td>
<td align="center" width="50%"><img src="docs/images/planar-result-band-only.png" width="100%"/><br/>Transition band only</td>
</tr>
<tr>
<td align="center" width="50%"><img src="docs/images/planar-result-seed-only.png" width="100%"/><br/>Action set to partition and seed only</td>
<td align="center" width="50%"><img src="docs/images/planar-plate-3to1.png" width="100%"/><br/>3D plate, 3:1 transition</td>
</tr>
</table>

If you do not want the plug-in to generate the final mesh, switch `Action` to `Partition + seed only (no mesh)`. The plug-in keeps your global seeds and fixes only the short edges inside the transition band, so the main body is not dragged to the fine size.

#### Multi-layer

When `Layers` is greater than 1, the plug-in allocates the band width to each stage. Multi-layer transitions work best with a larger `Depth`. If the band is too narrow for the number of layers, the elements become very flat and meshing may fail.

The workflow is the same as the single-layer case. You can leave the coarse edge unpicked and specify only the fine edge, direction and `Depth`; the plug-in grows the stages outwards.

<table>
<tr>
<td align="center" width="50%"><img src="docs/images/planar-multilayer-before.png" width="100%"/><br/>Two-layer transition: picks and parameters before Apply</td>
<td align="center" width="50%"><img src="docs/images/planar-multilayer-result.png" width="100%"/><br/>Two-layer 2:1 result</td>
</tr>
</table>

#### Hybrid

A hybrid transition mixes 2:1 and 3:1 in one band. It is available only in `PLANAR`, and `Layers` must be 1.

Enable `Hybrid`; the dialog shows `2:1 blocks b`. Let the fine-side count be `N` and the number of 2:1 groups be `b`:

```text
fine segments left for 3:1 = N - 2b
number of 3:1 groups       = (N - 2b) / 3
total coarse segments      = (N + b) / 3
```

Constraints:

- `b` must be even, because the 2:1 template tiles in pairs;
- `N - 2b` must be divisible by 3;
- the total coarse count must lie between `N/3` and `N/2`.

For example, `N=80` and `b=10`:

```text
fine segments left for 3:1 = 80 - 2 x 10 = 60
number of 3:1 groups       = 60 / 3 = 20
total coarse segments      = 10 + 20 = 30
```

If the parameters do not satisfy the rules, click `Suggest values` and let the plug-in adjust them.

<table>
<tr>
<td align="center" width="50%"><img src="docs/images/planar-hybrid-before.png" width="100%"/><br/>Hybrid 2:1 + 3:1: parameters before Apply</td>
<td align="center" width="50%"><img src="docs/images/planar-hybrid-result.png" width="100%"/><br/>Hybrid result</td>
</tr>
</table>

### 4.2 PLANAR_ANNULAR

Use for planar concentric annular bands. The fine side can be inside or outside.

#### Pre-partitioned geometry

Use two concentric circles to split the disk into three independent face regions:

```text
fine regular ring  |  transition band  |  coarse regular ring
```

An uncut disk face is not accepted. If the fine side is outside, pick the inner and outer loops as they are; the plug-in identifies the direction from the measured arc radii.

#### Picks

| Pick | Required | Meaning |
| --- | --- | --- |
| `Transition region` | Yes, single | Target annular face |
| `Fine loop` | Yes, multiple | All edge segments of the fine loop |
| `Coarse loop` | Yes, multiple | All edge segments of the coarse loop |
| `Full domain` | Optional, multiple | Regular rings on both sides, for full-domain meshing |

#### Parameters and rules

- `Seeds` is the count around the whole loop, not the count on one arc segment;
- the fine count must pass the divisibility check, and the full ring must contain at least 3 template periods;
- if the model is divided into equal sectors, the coarse count must also be divisible by the sector count;
- the native vertices of the fine and coarse loops must lie on the angular grid. If the angular deviation exceeds 0.001 degrees, the plug-in asks you to adjust `N` or the radial partition lines;
- with `Full domain` selected, the plug-in classifies each face by radius into the fine regular ring, the transition band and the coarse regular ring. The regular rings are seeded by arc length so the nodes on the shared circles match one-to-one.

#### Steps and results

1. Partition the concentric circles and make sure the transition band can be structured;
2. Select `PLANAR_ANNULAR` under `Type`;
3. Pick `Transition region`, `Fine loop` and `Coarse loop`; pick `Full domain` if needed;
4. Enter `Seeds`, `Layers` and `Ratio`;
5. Run `Check`, then `Pre-mesh`, then `Apply`.

<table>
<tr>
<td align="center" width="50%"><img src="docs/images/annular-dialog.png" width="100%"/><br/>PLANAR_ANNULAR dialog</td>
<td align="center" width="50%"><img src="docs/images/annular-pick-region.png" width="100%"/><br/>Pick the transition ring</td>
</tr>
<tr>
<td align="center" width="50%"><img src="docs/images/annular-pick-fine.png" width="100%"/><br/>Pick the fine loop</td>
<td align="center" width="50%"><img src="docs/images/annular-pick-coarse.png" width="100%"/><br/>Pick the coarse loop</td>
</tr>
<tr>
<td align="center" width="50%"><img src="docs/images/annular-result.png" width="100%"/><br/>Annulus 3:1 result</td>
<td align="center" width="50%"><img src="docs/images/annular-cases.png" width="100%"/><br/>Other cases: single-layer, multi-layer, fine inside or outside, sectors, full domain</td>
</tr>
</table>

### 4.3 PLANAR_QUAD_FOUR

Use to build a closed ring around a rectangular inner frame. The topology is fixed: fine mesh inside, coarse mesh outside.

#### Picks

| Pick | Required | Meaning |
| --- | --- | --- |
| `Transition region` | Yes, single | The ring face from the inner frame to the outer frame |
| `Fine loop` | Yes, multiple | Four connected edges of the inner frame, forming a closed rectangle |
| `Coarse loop` | Optional, multiple | Outer frame loop; if omitted, the plug-in computes it |
| `Full domain` | Optional, multiple | Central region and outer regular faces, for full-domain meshing |

`Fine loop` checks that opposite sides are equal and the four corners are right angles. Rotated rectangles are supported. If the fine mesh is picked on the outside, the plug-in stops and asks you to swap the loops.

#### Parameters and topology

- `Nx` is the main-direction fine count, `Ny` is the orthogonal fine count; `Ny=0` uses the same count as `Nx`;
- `Nx` and `Ny` are validated independently. A legal value in one direction does not make the other direction legal;
- 2:1 single layer: coarse count is `Nx/2 + 2`, where the +2 comes from the two corner contributions, and the outer frame extends by two coarse elements per side;
- 3:1 single layer: coarse count is `Nx/3 + 2`;
- 2:1 supports multi-layer cascades. Before a non-first stage, the plug-in inserts a regular orthogonal ring outside the previous stage so the next fine count is a multiple of 4. The inserted ring does not change the existing fine seeds;
- 3:1 is constructible, but the transition row has an element aspect ratio of about 3, so the quality is lower than 2:1. Prefer 2:1 when a regular mesh matters. When 3:1 is used with multiple layers, `Check` reports the quality caveat.

Leaving `Coarse loop` empty is recommended. The plug-in computes the outer frame from the ratio and reports it in the summary, so you do not have to calculate the expansion by hand.

#### Steps and results

1. Draw the rectangular inner frame and, if needed, pre-partition the full transition ring;
2. Select `PLANAR_QUAD_FOUR` under `Type`;
3. Pick `Transition region` and `Fine loop`; pick `Coarse loop` and `Full domain` if needed;
4. Enter `Nx`, `Ny`, `Layers` and `Ratio`;
5. Run `Check`, then `Pre-mesh`, then `Apply`.

<table>
<tr>
<td align="center" width="50%"><img src="docs/images/quadfour-dialog.png" width="100%"/><br/>PLANAR_QUAD_FOUR dialog</td>
<td align="center" width="50%"><img src="docs/images/quadfour-pick.png" width="100%"/><br/>Pick the inner frame and transition face</td>
</tr>
<tr>
<td align="center" width="50%"><img src="docs/images/quadfour-result.png" width="100%"/><br/>Square inner frame, 2:1 result</td>
<td align="center" width="50%"><img src="docs/images/quadfour-rect-result.png" width="100%"/><br/>Rectangular inner frame, 2:1 result with separate Ny</td>
</tr>
<tr>
<td align="center" width="50%"><img src="docs/images/quadfour-rect-pick.png" width="100%"/><br/>Pick the rectangular inner frame</td>
<td align="center" width="50%"><img src="docs/images/quadfour-cases.png" width="100%"/><br/>Other cases: 2:1 three and four layers, 3:1 single layer, rectangular 3:1</td>
</tr>
</table>

### 4.4 ANNULAR_RADIAL

Use on the annular end face of a thick-walled cylinder to transition through the wall thickness. The fine and coarse loops must lie on the same end face and be concentric.

#### Picks

| Pick | Required | Meaning |
| --- | --- | --- |
| `Region Faces` | Yes, multiple | Transition band faces on the end face |
| `Fine Edge Loop` | Yes, multiple | Fine circular edges; the fine side can be inside or outside |
| `Coarse Edge Loop` | Yes, multiple | Coarse circular edges; coplanarity and concentricity are checked |
| `Target Cells` | Yes, multiple | Solid cell that receives the transition mesh |
| `Sweep Path Edges` | Optional, multiple | 3D sweep path edges belonging to the target cell; derived automatically if omitted |

#### Preconditions

- Partition an independent solid cell for the transition band;
- Assign a sweep technique to that cell in the Mesh module, preferably with an explicit sweep path;
- the end face must contain a radial straight edge that crosses the band, usually from a sector partition of the wall, for angular phase alignment. Without it, the plug-in asks you to complete the sector partition first;
- multi-layer radial transitions require pre-partitioning the solid at the intermediate radii so each cell covers exactly one stage.

#### Parameters and rules

- `Seeds` is the total count around the fine loop;
- the template is aligned on a 90-degree quadrant, so the template period must divide 90 degrees;
- for 2:1, `N` should be a multiple of 16;
- for 3:1, `N` should be a multiple of 12;
- if the value does not fit, the status line shows nearby legal counts, or click `Suggest values`.

With `Auto-adapt regular regions (seed + mesh)` enabled, the plug-in also seeds and meshes the adjacent inner and outer regular regions. Disable it to process only the transition band.

#### Steps and results

1. Partition the transition cell on the cylinder end face and assign the sweep;
2. Select `ANNULAR_RADIAL` under `Type`;
3. Pick `Region Faces`, `Fine Edge Loop`, `Coarse Edge Loop` and `Target Cells`; pick `Sweep Path Edges` if needed;
4. Enter `Seeds`, `Layers` and `Ratio`; enable `Auto-adapt` if needed;
5. Run `Check`, then `Pre-mesh`, then `Apply`.

<table>
<tr>
<td align="center" width="50%"><img src="docs/images/radial-dialog.png" width="100%"/><br/>ANNULAR_RADIAL dialog</td>
<td align="center" width="50%"><img src="docs/images/radial-result.png" width="100%"/><br/>Cylinder radial 3:1 result</td>
</tr>
<tr>
<td align="center" colspan="2"><img src="docs/images/radial-cases.png" width="80%"/><br/>Cylinder radial 2:1 cases: two layers, fine loop outside</td>
</tr>
</table>

### 4.5 AXIAL_SURFACE

Use on the barrel of a cylindrical solid or shell to transition along the axis from a fine loop to a coarse loop.

#### Picks

| Pick | Required | Meaning |
| --- | --- | --- |
| `Region Faces` | Yes, multiple | Transition band faces on the barrel |
| `Fine Edge Loop` | Yes, multiple | Circular edges at the fine end section |
| `Coarse Edge Loop` | Yes, multiple | Circular edges at the coarse end section |
| `Target Cells` | Yes, multiple | Solid cell that receives the transition mesh |
| `Sweep Path Edges` | Optional, multiple | Generatrix edges along the axis, belonging to the target cell; derived automatically if omitted |

#### Geometry requirements

- the fine and coarse loops lie on different cross sections of the same cylindrical surface;
- the plug-in determines the axis from the line joining the two circle centers and checks that both sections are perpendicular to it;
- partition the cylinder along the axis into a fine region, the transition band and a coarse region, keeping the through-thickness sweep topology;
- the number of axial slices is computed by the plug-in from the ratio; no manual input is needed;
- the plug-in checks the angular fit between the axial slices and the existing radial seams. If the slices coincide with all existing seams, it fails and suggests nearby legal counts. An uneven distribution produces a warning.

#### Parameters and rules

- `Seeds N (per section)` is the circumference count at the fine end section;
- for 2:1, `N` must be divisible by 4, because each sector uses a pair of 4-fine to 2-coarse templates;
- for 3:1, `N` must be divisible by 3;
- axial multi-layer does not require manual pre-partitioning at each stage. The stage interfaces are planes perpendicular to the axis, and the plug-in cuts the intermediate sections automatically;
- with `Auto-adapt regular regions (seed + mesh)` enabled, the adjacent regular solid regions are also seeded and meshed.

#### Steps and results

1. Partition the cylinder along the axis into fine, transition and coarse regions, keeping the through-thickness sweep topology;
2. Select `AXIAL_SURFACE` under `Type`;
3. Pick `Region Faces`, `Fine Edge Loop`, `Coarse Edge Loop` and `Target Cells`; pick `Sweep Path Edges` if needed;
4. Enter `Seeds`, `Layers` and `Ratio`; enable `Auto-adapt` if needed;
5. Run `Check`, then `Pre-mesh`, then `Apply`.

<table>
<tr>
<td align="center" width="50%"><img src="docs/images/axial-dialog.png" width="100%"/><br/>AXIAL_SURFACE dialog</td>
<td align="center" width="50%"><img src="docs/images/axial-band.png" width="100%"/><br/>Axial transition band and template</td>
</tr>
<tr>
<td align="center" colspan="2"><img src="docs/images/axial-result.png" width="80%"/><br/>Cylinder axial 3:1 result</td>
</tr>
</table>

## 5 Troubleshooting

Check status and errors are reported in the status line and in pop-up dialogs. Every operation runs inside a model transaction, so a failed check or a failed mesh rolls the model back to its previous state.

| Condition | Message | Fix |
| --- | --- | --- |
| More than one entity or no entity in a single-pick group | `requires exactly one face/edge; got N` | Re-pick a single face or edge and confirm with Done |
| Entities from different parts | `Selected entities belong to multiple parts` | Clear the selection and pick again within one part |
| Stale selection after a mode switch | `Command dispatch failed... Repick active geometry` | Clear the stale selection and re-pick the geometry of the active mode |
| Missing required loop or solid | `needs fine and coarse edge loops / Select target cells` | Complete the fine and coarse loops, or pick the target cells in 3D |
| Fine count not divisible | `Need divisible by X. near A/B` | Click `Suggest values` to fill a nearby legal value |
| Hybrid ratio conflict | `2:1 blocks b must be even / a not integer` | Make `b` even and adjust `N` so `N-2b` is divisible by 3 |
| Band width outside the face | `Transition depth must be positive / reaches beyond width` | Reduce `Depth`, or pick the coarse edge and use the measured distance |
| Invalid planar orientation | `Orientation direction is not in face plane` | Choose a direction perpendicular to the fine edge and inside the face |
| Annular vertex off the angular grid | `Vertex angle not on angular grid (dev > 0.001 deg)` | Adjust `N` as suggested, or fix the radial partition lines |
| Fine side outside in the four-side mode | `Inward transition not supported` | Swap the inner and outer loops; use `PLANAR_ANNULAR` for a ring |
| Outer frame does not satisfy rule 4 | `Rule 4 not met: need half-sizes...` | Leave the outer loop empty and let the plug-in compute it, or re-partition to the suggested size |
| Missing radial alignment edge | `No radial edge found in the band for phase alignment` | Pre-partition the end face into radial sectors |
| Missing intermediate radial partition | `Stages > 1: target Cells must already be partitioned...` | Pre-partition the cells at the intermediate radii, or continue from the Check dialog |
| Axial slices collide with existing seams | `Axial slice pattern collides with the existing cuts` | Use the suggested legal count and offset the existing seams |
| Zero elements after meshing | `Generate Mesh produced 0 elements` | Check the transition control and boundary seeds, then remesh |
| Mesh audit failed | `Mesh audit failed or incomplete` | The model was rolled back; increase the band width or reduce the number of layers |
| No sweep on the solid | `Face not attached to solid / sweep path missing` | Assign the sweep direction and path to the transition cell in the Mesh module |
| Pre-mesh blocked | `Pre-mesh failed (model rolled back)` | The model was rolled back; fix the cause shown in the dialog |
| Duplicate modules or version conflict | `Deployment manifest conflict / version mismatch` | Close Abaqus, remove duplicate copies from the plug-in directory and restart |

## 6 Scope and limitations

- The 3D modes (3D plate, cylinder radial, cylinder axial) require the transition band to lie on a pre-partitioned solid cell with usable sweep or structured meshing capability;
- multi-layer radial transitions require pre-partitioning the solid at the intermediate radii; multi-layer axial transitions cut the intermediate sections automatically;
- `PLANAR_QUAD_FOUR` only supports fine inside and coarse outside; a reversed pick is blocked;
- the annulus mode does not accept an uncut disk face; the transition ring must be cut with concentric circles first;
- `Hybrid` is available only in `PLANAR` and only with a single layer;
- the cylinder modes are intended for regular cylinders, hollow cylinders and thick-walled end faces, not for arbitrary free-form surfaces, variable-radius surfaces or complex openings;
- run at least `Check` before the final execution, and preferably `Pre-mesh` as well;
- keep exactly one complete copy in the plug-in search directory.

## 7 Contact and support

### 7.1 Contact

- Non-commercial use is free. For commercial use, collaboration or customisation, contact **Z25150070@s.upc.edu.cn**;
- Feedback: open a [GitHub Issue](https://github.com/IY724/TransitionMesh/issues) or send an email;
- This project was built with the assistance of AI-assisted programming (vibecoding).

### 7.2 Buy Me a Coffee

The research group does not reimburse token costs, so development and maintenance are self-funded. If this plug-in saves you time, you are welcome to buy me a coffee and support future updates.

<table>
<tr>
<td align="center" width="50%"><img src="docs/images/wechat-qr.jpg" width="300"/><br/>WeChat Pay</td>
<td align="center" width="50%"><img src="docs/images/alipay-qr.jpg" width="300"/><br/>Alipay</td>
</tr>
</table>
