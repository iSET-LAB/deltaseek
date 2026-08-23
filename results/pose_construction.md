# Candidate pose construction — audit

Audit of how candidate sensor poses are generated for the Hall B sensor
ablation, written 2026-08-23. Every number below is measured from the code at
commit `56b8d4e`, not recalled.

---

## 1. The construction

Base positions come from a regular grid over navigable free space
(`viewpoints.py:117-130`):

```python
def free_base_poses(elements, bounds, spacing=1.5, yaws=(0.0,),
                    inflation=ROBOT_RADIUS):
    xs = np.arange(x_min, x_max + 1.0e-9, spacing)
    ys = np.arange(y_min, y_max + 1.0e-9, spacing)
    for x, y in itertools.product(xs, ys):
        if is_blocked((x, y), blocked):
            continue
        for yaw in yaws:
            poses.append(((float(x), float(y)), float(yaw)))
```

**Yaw is a dimension of the candidate list, not a free variable.** Each
surviving position is emitted once per yaw bin. It is *not* folded into the
visibility test, so a change is only observable if some *sampled* heading
frames it.

The ablation calls this with `spacing = 1.0` m and four yaw bins
(`sensor_ablation.py:71-73`, repeated identically at `391-393`):

```python
bases = room_base_poses(
    elements, scene_bounds(elements), args.spacing,
    (0.0, 1.5707963, 3.1415927, -1.5707963), args.room)
```

so yaw is discretised into **4 bins at 90 deg spacing**. Poses are then clipped
to the room rectangle (`sensor_ablation.py:52-64`).

Each base pose is combined with the five arm postures
(`viewpoints.py:165-176`). The chassis pose is computed from `world_base` alone
and is therefore identical across the five postures of a given base pose; the
wrist pose is `world_base @ arm_pose` and differs for each.

### Counts

| quantity | value | decomposition |
| --- | --- | --- |
| grid spacing | 1.0 m | — |
| free positions in the room | 60 | — |
| yaw bins | 4 | 90 deg apart |
| base poses | 240 | 60 positions x 4 yaws |
| arm postures | 5 | — |
| viewpoints | 1200 | 240 base poses x 5 postures |
| **distinct chassis poses** | **240** | 60 positions x 4 yaws x 1 |
| **distinct wrist poses** | **1200** | 60 positions x 4 yaws x 5 postures |

### Do both configurations share base positions?

**Yes.** `run_configuration` builds `bases` before the sensor set is consulted
(`sensor_ablation.py:71-77`); `sensors` only filters at
`viewpoint.poses(sensors)`. There is no confound.

One latent coupling is worth recording. `build_viewpoints` discards a viewpoint
when the **wrist** camera falls below `min_camera_height = 0.25` m
(`viewpoints.py:170-171`), and that discards its chassis pose too. Today the
lowest wrist camera sits at 0.291 m, so nothing is dropped (1200 = 240 x 5
exactly). A lower arm posture would silently shrink the chassis candidate set,
which would be a real confound.

---

## 2. Reconciling 240 and 600

**240 is correct. 600 was never right, and it is not in the paper.**

- **240** = 60 positions x 4 yaws, the number of distinct chassis sensor poses.
- **1200** = the same base poses x 5 arm postures, i.e. viewpoints, which is
  also the number of distinct *wrist* poses.
- **600** does not correspond to anything in the construction. It came from an
  earlier version of `observation_counts` that counted *viewpoints* rather than
  distinct poses, then quoted a denominator of 1200/2. Because the chassis pose
  repeats once per arm posture, that inflated its apparent coverage fivefold.
  The counting bug was fixed before the paper was written.

The paper is clean: `deltaseek.tex` says 240 at lines 58, 235, 237, 290, 292 and
310, and 600 appears nowhere in it. The stale figure survives in three
non-paper places, listed in section 6 below.

---

## 3. Yaw discretisation against the field of view

| quantity | value |
| --- | --- |
| chassis horizontal FOV | 1.25 rad = **71.62 deg** |
| chassis vertical FOV | 0.992 rad = 56.84 deg |
| yaw bin spacing | **90 deg** |
| gap per bin boundary | 90 - 71.62 = **18.38 deg** |
| total unframable azimuth | 4 x 18.38 = **73.5 deg of 360 = 20.4%** |

**The bin spacing exceeds the horizontal FOV, so there are blind headings.**
From any given position, 20.4% of azimuth directions fall between the sampled
frusta and cannot be framed by any sampled yaw.

This weakens claims of the form "unobservable from every pose" *for reasons of
azimuth* — that is, for the occlusion and incidence classes. It does **not**
affect the height claim, for the reason established in section 4: yaw cannot
change the frustum's vertical extent, so no azimuth refinement can lift the
ceiling. Section 5 confirms this empirically.

---

## 4. The chassis ceiling, analytically

The chassis camera has zero roll, so the world-z row of its rotation is
`(-sin p, 0, cos p)` for pitch `p`. A frustum point is `(f, y, z)` in camera
axes with `f <= far`, `|z| <= f tan(v/2)`. World height is therefore

```
    z_world = h - sin(p) f + cos(p) z
            = h + f (cos(p) tan(v/2) - sin(p))          maximised at z = f tan(v/2)
```

which is increasing in `f`, so the maximum sits at the far plane's top edge:

```
    z_max = h + far * (cos(p) * tan(v/2) - sin(p))
```

**Yaw does not appear.** It rotates about the vertical axis and cannot enter
the z-row.

| quantity | value |
| --- | --- |
| camera height `h` | 0.417526 m |
| pitch `p` | 0.170000 rad (9.74 deg, downward) |
| `tan(v/2)` | 0.541113 |
| far | 5.0 m |
| **analytic `z_max`** | **2.2382 m** |
| swept `z_max` (1 mm grid) | 2.2370 m |
| difference | **1.18 mm**, i.e. one grid step |

Numeric confirmation of yaw invariance: swept over 16 yaws, the maximum
attainable height varied by **0.00e+00 m**.

Three tests now guard this (`test/test_sensors.py`):
`test_chassis_ceiling_matches_its_closed_form`,
`test_chassis_ceiling_does_not_depend_on_yaw`, and
`test_height_changes_sit_above_the_chassis_ceiling`.

### Two corrections this produced

**The paper's reach figures are wrong.**

| | paper says | correct | why |
| --- | --- | --- | --- |
| chassis reach | 2.23 m | **2.24 m** | 2.2382 rounds up; the old 2.23 came from a 5 mm sweep that landed at 2.23499 |
| wrist reach | 3.59 m | **4.63 m** | the old sweep's `z` range stopped at 3.6 m, so it reported its own upper bound |
| wrist-only band | 0.21 m | **0.20 m** | 2.44 - 2.2382 |

**The height margin is 11.8 mm.** The height pair sits at 2.250 m against a
ceiling of 2.2382 m. The empirical result does not rest on that margin --- the
sweep uses `visible_fraction` with a 5% surface threshold, not the bound --- but
the claim is fragile to mount changes. Raising the chassis camera by 12 mm, or
levelling it, would erase it.

---

## 5. Sensitivity to sampling resolution

Chassis-only sweep, counting distinct chassis poses from which each change is
observable, at increasing yaw and position resolution.

| deviation | class | 60x4<br>(240 poses) | 60x8<br>(480 poses) | 60x16<br>(960 poses) | 230x4<br>(920 poses) | 230x8<br>(1840 poses) |
| --- | --- | --- | --- | --- | --- | --- |
| T1-pallet-stack | trivial | 30 (12.5%) | 61 (12.7%) | 130 (13.5%) | 126 (13.7%) | 245 (13.3%) |
| T2-site-cabin | trivial | 23 (9.6%) | 44 (9.2%) | 97 (10.1%) | 98 (10.7%) | 181 (9.8%) |
| H1-rack-tote | height | **0** | **0** | **0** | **0** | **0** |
| H2-cabinet-carton | height | **0** | **0** | **0** | **0** | **0** |
| I1-window-displaced | incidence | 10 (4.2%) | 21 (4.4%) | 49 (5.1%) | 46 (5.0%) | 98 (5.3%) |
| I2-standing-panel | incidence | 14 (5.8%) | 30 (6.2%) | 57 (5.9%) | 61 (6.6%) | 126 (6.8%) |
| O1-crate-behind-stack | occluded | 16 (6.7%) | 34 (7.1%) | 66 (6.9%) | 63 (6.8%) | 127 (6.9%) |
| O2-drum-behind-rack | occluded | 29 (12.1%) | 60 (12.5%) | 114 (11.9%) | 118 (12.8%) | 234 (12.7%) |
**The height pair stays at exactly 0.0000 visible fraction at every
resolution**, up to 1840 poses at 22.5 deg yaw spacing and 0.5 m grid. The main
capability claim is not a discretisation artefact.

Counts for the other six rise roughly in proportion to the pose count, so their
*share* is stable to about one percentage point across an eightfold change in
sampling. `I2-standing-panel` moves 5.8% -> 6.8%; `O1-crate-behind-stack` holds
at 6.7% -> 6.9%. Neither is an artefact either, and neither becomes easy.

Note the expected counts in the task brief (70 for the panel, 105 for the
crate) are the superseded viewpoint-based numbers. On the corrected
distinct-pose basis they are 14 and 16 of 240.

---

## 6. Follow-ups this audit did not action

Reported, not changed, per the audit-first constraint:

1. **`deltaseek.tex` reach figures.** 2.23 -> 2.24, 3.59 -> 4.63, band 0.21 ->
   0.20. `fig_reach.pdf` needs regenerating with them.
2. **Stale "600"** in `README.md:306`, `PUBLICATION_STATUS.md:131,135`, and two
   docstrings, `sensor_ablation.py:127,168`.
3. **Yaw resolution.** 90 deg bins leave 20.4% of azimuth unframable. The
   height claim is unaffected, but reporting the occlusion and incidence counts
   at 22.5 deg spacing would be more defensible.
4. **`min_camera_height` coupling** between the wrist and the chassis candidate
   set (section 1). Dormant, worth decoupling.
