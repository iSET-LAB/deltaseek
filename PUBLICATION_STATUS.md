# DeltaSeek — state of the work and what stands between it and submission

Updated 2026-08-22. Target venue: the Construction Robots workshop
(<https://construction-robots.github.io/>). Every number below is measured from
the code in this repository.

---

## 1. The short version

The infrastructure is finished and tested: 20 commits, 72 passing tests, a
working IFC pipeline on a real Revit export, and a simulated A300 + UR5e
carrying the three sensors the physical robot will have.

The science has moved twice since the paper was written, and the paper has not
followed. `ieeeconf/deltaseek.pdf` argues a **negative** result at building
scale. A single-room experiment then produced the **opposite** result. A sensor
ablation has now qualified both by asking the question a reviewer asks first,
and its answer is narrow: **exactly two of eight deviations require the wrist
camera, and both for the same reason.**

That is a real contribution and a much smaller one than the abstract currently
implies. Reconciling the three is the decision that gates everything else.

---

## 2. What exists and works

| Component | File | State |
| --- | --- | --- |
| IFC import, oriented boxes | `ifc_to_manifest.py` | works on a real Revit export |
| Room extraction | `extract_room.py` | clips in element frames |
| Deviation sampling | `deviation_sampler.py` | seeded, reproducible |
| Observability classes | `benchmark.py` | trivial / height / incidence / occluded |
| Multi-sensor observation | `detection.py`, `visibility.py` | any-sensor-observes semantics |
| Forward kinematics | `kinematics.py` | verified against live TF to 1e-16 |
| Traversal cost | `navigation.py` | Dijkstra on the platform's Nav2 costmap |
| Planners + 3 baselines | `planner.py` | shared candidates, shared budget |
| Evaluation | `evaluate.py`, `compare.py` | matched-distance scoring, `--sensors` |
| Sensor ablation | `sensor_ablation.py` | capability and routing reported apart |
| Gazebo worlds | `generate_benchmark.py` | verified running, no segfaults |

### The robot

Generated from `clearpath/robot.yaml`: Husky A300 with AMP enclosure and UR5e,
carrying an eye-in-hand D435 (`camera_0`), a chassis D435 (`camera_1`) and an
Ouster OS1. All three publish in simulation. Everything sits on the enclosure
deck, the top face at z = 0.407 spanning x = ±0.486 and y = ±0.226:

| | position (base_link) | worst clearance to the arm |
| --- | --- | --- |
| Ouster OS1 | (0.380, 0.000, 0.429) | 0.078 m |
| chassis D435 | (0.460, 0.000, 0.418) | 0.168 m |
| UR5e base | (0.243, 0.000, 0.407) | — |

Clearances are after subtracting link radii. **Known cost of this layout:** the
camera stands 0.093 m directly in front of the lidar on the same centreline and
blocks roughly 30° of its azimuth dead ahead, along the direction of travel.
Raising the lidar clears it.

### The real IFC model

`ERS_B_STRUCT.ifc` — IFC2X3 from Revit 2025, 133 products, all converting
without failure. One storey built, 44.5 × 19.0 × 4.5 m. Despite the filename
the content is architectural: 42 walls, 24 doors, 24 windows, no columns or
beams.

---

## 3. The three results

### 3.1 Building scale — the negative result the paper reports

Synthetic storey, 110 elements, 5 seeds, density 0.2, 60 m budget, 5 m range:

| planner | drivable cost | straight-line cost |
| --- | --- | --- |
| coverage | **70.0 ± 5.3** | 70.0 ± 5.3 |
| deviation seeking | 63.6 ± 6.1 | **72.1 ± 5.7** |
| frontier | 47.9 ± 4.3 | 47.1 ± 6.1 |
| goal directed | 25.7 ± 4.2 | 42.9 ± 4.5 |

**Deviation-seeking does not beat coverage once distance is measured around
walls.** Drivable distance averages 1.8× the straight line and reaches 11×, so
this is not a rounding difference. The likely cause is routing: greedy
benefit-per-metre is myopic over 40 m, while coverage's nearest-neighbour tour
is a decent TSP heuristic.

### 3.2 Room scale — the positive result the paper does not report

Hall B, clipped from the real IFC: 7.0 × 12.5 m. 25 m budget, 5 m range,
**one seed, one layout, six deviations**:

| planner | found | @6.2 m | @12.5 m | @18.8 m | @25 m |
| --- | --- | --- | --- | --- | --- |
| **deviation seeking** | 6/6 | 3 | **6** | 6 | 6 |
| coverage | 6/6 | 4 | 4 | 6 | 6 |
| frontier | 4/6 | 4 | 4 | 4 | 4 |
| goal directed | 4/6 | 2 | 2 | 2 | 4 |

The reading that unifies 3.1 and 3.2: **over 40 m routing dominates and
coverage wins; over 12 m viewpoint quality dominates and deviation-seeking
wins.** Same algorithm, two regimes.

### 3.3 Sensor ablation — what the wrist camera is actually worth

This is the newest result and the one that most changes what can be claimed.

Eight hand-placed deviations in Hall B, two per observability class, in a
committed scenario. Three sensor configurations, same planner, same 25 m
budget, same 5 m range. Reproduce with:

```bash
ros2 run deltaseek_gazebo sensor_ablation \
  --manifest config/benchmarks/hall_b_ablation_nominal.yaml \
  --scenario config/benchmarks/hall_b_ablation_deviations.yaml \
  --room 2.9 -6.2 9.9 6.3 --start 6.4 4.5 --output-dir results
```

| class | chassis | wrist | both |
| --- | --- | --- | --- |
| trivial | 2/2 | 2/2 | 2/2 |
| height | **0/2** | 2/2 | 2/2 |
| incidence | 1/2 | 2/2 | 2/2 |
| occluded | 2/2 | 2/2 | 2/2 |
| **all** | **5/8** | 8/8 | 8/8 |

**The headline is not 5/8.** A planned run conflates what a sensor *can* see
with what a route happened to reach. Separating them:

- **2 of 8 are invisible to the chassis camera from every one of 240 candidate
  chassis poses**, at 0.0000 visible fraction: `H1-rack-tote` and
  `H2-cabinet-carton`, both `height`. This is a capability limit and it is
  stable under eightfold finer sampling.
- **1 further deviation** (`I2-standing-panel`) was missed by the chassis-only
  run but is visible from 14 of those 240 poses. That is routing, not evidence
  for the arm.

This distinction was not academic. Repositioning the arm at the author's
request moved the chassis headline from 6/8 to 5/8 — which looked like the arm
becoming *more* necessary and was entirely routing noise. The capability answer
did not move. `sensor_ablation.py` now reports both, so the headline cannot
drift with the planner's mood.

#### Why only `height` works

In closed form, the highest world point a camera can place inside its frustum
is `h + r(cos p · tan(v/2) − sin p)`, with no yaw term. At the 5 m limit that is
**2.238 m** for the chassis camera and **4.627 m** for the wrist; both agree
with a numeric sweep to 3 mm. The ceiling starts at 2.44 m, so the wrist-only
band is **0.20 m tall** and the height pair sits in it at 2.25 m — clearing the
chassis ceiling by just **11.8 mm**.

**The narrowness is itself a finding.** In a 2.47 m room a pitched-down chassis
camera covers nearly the whole vertical extent. The arm's height advantage is
real, geometrically confined here, and would grow substantially in a plant
room, warehouse or double-height space. Any claim must say which.

#### What occlusion actually costs

`occluded` and `incidence` were designed to require the arm and did not. The
crate behind the pallet stack is hidden from 93% of chassis poses — genuinely
occluded — but a planner only needs one of the remaining 16. Where both
configurations succeed, the difference is distance:

| configuration | median distance to detection |
| --- | --- |
| chassis | 14.2 m |
| wrist | 6.0 m |
| both | 8.9 m |

**Occlusion is a routing cost paid in metres, not a sensing limit.** That is a
defensible and interesting claim. It is a different claim from the one the
project currently makes.

---

## 4. Issues, ordered by what threatens the submission

### Issue 1 — the paper argues a case the experiments no longer support
**Blocking.** `deltaseek.tex` is built around §3.1. A section heading reads
"Greedy information gain does not beat coverage". Its table and both result
figures encode the 110-element synthetic storey. The room experiment, the real
IFC model, the chassis sensors and the ablation appear nowhere.

Options:

1. **Publish the negative result as written.** Honest, complete, compiled.
   Weak headline.
2. **Rewrite around the two-regime story**, with the ablation bounding the
   claim. Strongest paper; needs Issue 2 resolved.
3. **Rewrite around the ablation alone** — "here is precisely what an
   eye-in-hand camera buys, and it is narrower than assumed". Smallest claim,
   fully supported by data in hand, and the only option that needs no further
   experiments.

Doing nothing submits option 1 by default.

### Issue 2 — the room planner result is a single sample
**Blocking for option 2.** Six deviations, one layout, one seed, no error bar.
Needs randomised clutter across ~20 layouts, preserving occlusion structure
rather than scattering boxes, and 15–25 deviations per room. Roughly half a
day. Note the ablation (§3.3) does *not* depend on this — it is a
deterministic geometric study by design.

### Issue 3 — the ablation is one scene and one room height
**High, and it bounds the claim rather than blocking it.** Eight deviations in
one 2.47 m room. The 0.20 m wrist-only band is a property of that ceiling. A
second room at a different height would turn a scene-specific observation into
a trend, and is cheap: `extract_room` already does the extraction.

### Issue 4 — box geometry destroys openings
**Medium.** 48 `IfcRelVoidsElement` in the source are discarded; a 39 m facade
with eleven doors becomes a solid barrier. Hall B is consequently a sealed box
whose door is impassable, and the robot must be spawned inside it. Acceptable
for a single-room study if stated; fatal for a multi-room one. The fix reuses
`synthetic_storey._wall_with_door`. About a day.

### Issue 5 — detection is geometric, not perceptual
**Medium; state it, do not fix it.** No detector, no renderer, no noise;
perfect recognition and localisation assumed. This measures *viewpoint
quality*, which is the right thing for a planner loop, but the paper must not
imply a real detector would find these deviations.

### Issue 6 — the framing does not match the stated task
**Medium; an opportunity.** The task is to gather enough information to *decide*
about the unmodelled objects and the window. The metric is binary detection
against a distance budget. A decision task wants graded evidence and a stopping
rule — distance to confident decision. Strictly stronger, because "how much
looking is enough" is a question coverage cannot answer. Out of reach for this
deadline.

### Issue 7 — administrative
- `\thanks` is a placeholder; affiliation and contact missing.
- `\bibitem{TODOactive}` is an explicit placeholder. The other 16 entries were
  written from memory and **all need verifying** against the real record.
- `ERS_B_STRUCT.ifc` is untracked. Its provenance and whether it may be
  published are unknown to me; a figure derived from it makes that a question
  that must be answered.
- **The repository is public.** This file, including its frank assessment of
  the paper, is visible to anyone.
- The deadline was understood to be the 24th. **Confirm against the workshop
  site** — the plan below depends on it entirely.

---

## 5. Recommendation

Issue 3 in §4 has changed the calculus. Before the ablation, the honest
fallback was a negative result. Now there is a positive, reproducible,
deterministic finding that needs no further experiments:

> An eye-in-hand camera resolves deviations a chassis camera cannot, but on
> this scene that is exactly the deviations in a 0.20 m band near the ceiling.
> Occlusion, which is the intuitive case for a manipulator, turns out to cost
> distance rather than detections: 14.2 m against 6.0 m median.

That is a smaller claim than the abstract makes and it is fully supported. It
is also more useful to a construction-robotics audience than a negative result,
because it tells them when the extra axis is worth paying for.

If the deadline is the 24th, in order:

1. **Rewrite around the ablation** (option 3). No experiments needed; the data
   exists and is committed.
2. **Add a second room at a different ceiling height** if time allows — it
   converts the 0.20 m band from an anecdote into a trend, and costs an hour.
3. Fix the administrative items in parallel.
4. Keep §3.1 and §3.2 as a planner-comparison section under honest framing, or
   defer them entirely to the follow-up.

What should **not** happen is submitting the room planner result as the
headline. One seed and six deviations will not survive a reviewer who asks for
a standard deviation, and the ablation now offers a better claim that does not
need one.
