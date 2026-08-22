# DeltaSeek — state of the work and what stands between it and submission

Written 2026-08-22. Target venue: the Construction Robots workshop
(<https://construction-robots.github.io/>). Everything below is measured from
the code in this repository, not estimated.

---

## 1. The short version

The infrastructure is finished and tested. The science is not settled, and the
paper currently on disk argues a case the most recent experiments contradict.

There is a compiled four-page extended abstract at `ieeeconf/deltaseek.pdf`
whose central result is a **negative** one: at building scale the
deviation-seeking planner loses to plain coverage. Two days ago the project
moved to a single-room scene, and there the same planner **wins** clearly. Both
results are honest. They are not yet reconcilable into one story, and the
paper has not been rewritten to reflect the newer one.

That is the decision the author has to make, and nothing else on this list
matters until it is made.

---

## 2. What exists and works

Sixty-three tests pass. Eight commits on `deviation-seeking-pipeline`.

| Component | File | State |
| --- | --- | --- |
| IFC import, oriented boxes | `ifc_to_manifest.py` | works on a real Revit export |
| Room extraction | `extract_room.py` | works, clips in element frames |
| Deviation sampling | `deviation_sampler.py` | seeded, reproducible |
| Visibility / detection | `visibility.py`, `detection.py` | ~12 ms per viewpoint |
| Forward kinematics | `kinematics.py` | verified against live TF to 1e-16 |
| Traversal cost | `navigation.py` | Dijkstra on the platform's own Nav2 costmap |
| Planners + 3 baselines | `planner.py` | shared candidate set, shared budget |
| Evaluation | `evaluate.py`, `compare.py` | matched-distance scoring |
| Gazebo worlds | `generate_benchmark.py` | verified running, no segfaults |

The robot is the official Clearpath A300 + UR5e generated from
`clearpath/robot.yaml`, carrying an eye-in-hand D435, a chassis D435 and an
Ouster OS1 on the platform flanks. All three publish in simulation.

### The real IFC model

`ERS_B_STRUCT.ifc` — IFC2X3 out of Revit 2025, 133 products, all of which
convert without a single failure. One storey built, 44.5 x 19.0 x 4.5 m.
Despite the filename the content is architectural: 42 walls, 24 doors,
24 windows, no columns or beams.

---

## 3. The two results

### 3.1 Building scale — the negative result the paper reports

Synthetic storey, 110 elements, 5 seeds, density 0.2, 60 m budget, 5 m range.
Detection rate, mean +/- s.d.:

| planner | drivable cost | straight-line cost |
| --- | --- | --- |
| coverage | **70.0 +/- 5.3** | 70.0 +/- 5.3 |
| deviation seeking | 63.6 +/- 6.1 | **72.1 +/- 5.7** |
| frontier | 47.9 +/- 4.3 | 47.1 +/- 6.1 |
| goal directed | 25.7 +/- 4.2 | 42.9 +/- 4.5 |

Read the first column. **Deviation-seeking does not beat coverage once
distance is measured around walls.** Under straight-line distance it appears to
lead, but the margin sits inside one standard deviation and vanishes under a
realistic cost. Drivable distance averages 1.8x the straight line here and
reaches 11x, so this is not a rounding difference.

The likely cause is routing, not objective: greedy benefit-per-metre is myopic
over 40 m, while coverage's nearest-neighbour tour is a decent TSP heuristic.

### 3.2 Room scale — the positive result the paper does not report

Hall B, clipped from the real IFC: 7.0 x 12.5 m, 12 modelled elements, one
window displaced 0.39 m, five objects absent from the model. 25 m budget,
5 m range, **one seed, one layout**.

| planner | found | @6.2 m | @12.5 m | @18.8 m | @25 m |
| --- | --- | --- | --- | --- | --- |
| **deviation seeking** | 6/6 | 3 | **6** | 6 | 6 |
| coverage | 6/6 | 4 | 4 | 6 | 6 |
| frontier | 4/6 | 4 | 4 | 4 | 4 |
| goal directed | 4/6 | 2 | 2 | 2 | 4 |

Manipulator ablation, same scene:

| | with arm | arm frozen |
| --- | --- | --- |
| deviation seeking | **6/6 at 12.5 m** | 4/6 even at 25 m |
| coverage | 6/6 at 18.8 m | 6/6 at 25 m |

The coherent reading of both tables: **over 40 m routing dominates and coverage
wins; over 12 m viewpoint quality dominates and deviation-seeking wins.** Same
algorithm, two regimes. That is a more interesting paper than either result
alone — but it is currently an interpretation of five seeds plus one seed, not
a demonstrated finding.

---

## 4. Issues, in the order they threaten the submission

### Issue 1 — the paper argues a case the newer experiments contradict

**Severity: blocking.**

`ieeeconf/deltaseek.tex` is written around the building-scale negative result.
Section titles include "Greedy information gain does not beat coverage".
Table~\ref{tab:main}, `fig_curves.pdf` and `fig_ablation.pdf` all encode the
110-element storey. The room experiment, the real IFC model and the chassis
sensors appear nowhere.

Three ways out:

1. **Publish the negative result as written.** Defensible, honest, and already
   compiled. Weak headline for a workshop.
2. **Rewrite around the regime story.** Strongest paper, needs the room result
   replicated across seeds (see Issue 2). Realistically a day of experiments
   plus a day of writing.
3. **Rewrite around the room result alone.** Fastest to a positive headline and
   the weakest scientifically, because n=1.

Doing nothing means submitting (1) by default.

### Issue 2 — the room result is a single sample

**Severity: blocking for options 2 and 3 above.**

Six deviations, one clutter layout, one seed. Every detection is worth 17
percentage points, so the curves are jagged and no error bar exists. The result
is currently an anecdote.

What it needs: randomised clutter placement across ~20 layouts, and 15-25
deviations per room rather than 6. The clutter is what makes the room
non-trivial — without the pallet stack occluding the window, every planner ties
— so the randomisation has to preserve *occlusion structure*, not just scatter
boxes. Estimated half a day.

### Issue 3 — the robot has three sensors; the evaluation models one

**Severity: high. This one can invalidate the claim.**

The simulation carries an eye-in-hand D435, a chassis D435 and an Ouster OS1.
`detection.py` still assumes a single camera with one `ObservationParams`.

This matters because the current `--fixed-sensor` ablation freezes the *arm* at
one posture and calls that the base-mounted comparison. No real robot parks its
only camera on a stopped arm, so the ablation answers a question nobody asks.
The first thing a reviewer will ask is "isn't a chassis camera enough?"

A 360-degree lidar will see the 1.6 m pallet stack, the drum and the tool chest
from almost anywhere in the room. Those detections become free and stop
counting. What it will not resolve: the 0.39 m in-plane window displacement,
the carton's top face, the crate in the stack's shadow. So the honest
comparison narrows to exactly the deviations that justify the arm — smaller
numbers, stronger claim.

**The risk is real and should be faced before writing, not after: if the lidar
resolves everything, the thesis is dead.** Running this ablation is the
cheapest available way to falsify the project's own claim.

One detail the model owes: the lidar is on the platform's left flank, so the
robot's own body occupies one azimuth sector. It is not the clean 360-degree
sweep a roof mount would give.

Estimated one day.

### Issue 4 — box geometry destroys openings

**Severity: medium, and it caps how far the real IFC can be pushed.**

The importer approximates each product by a box. Openings are modelled in the
source file (48 `IfcRelVoidsElement`, 14 of 42 walls voided, one wall carrying
11 openings) and are thrown away. A 39 m facade with eleven doors in it becomes
a solid 39 m barrier.

Consequences already observed: Hall B is a sealed box, its door is impassable,
and the first planner run scored 1 of 6 for every planner because the robot
spawned outside the walls and could not get in. The workaround was to spawn
inside. That is fine for a single-room study and will not survive a
multi-room one.

The fix is to split voided walls into jamb and lintel segments, reusing the
machinery in `synthetic_storey._wall_with_door` which is already guarded by
`test_doorways_are_real_openings`. Estimated a day.

**A note on scope:** this is only a limitation if the paper claims to handle
real BIM geometry. For a single-room study with the door treated as closed, it
is a modelling choice that can be stated in one sentence.

### Issue 5 — detection is geometric, not perceptual

**Severity: medium. Must be stated, need not be fixed.**

An element counts as observed when enough of its sampled surface falls in the
frustum, faces the camera within an incidence limit, and is unoccluded. There is
no detector, no renderer, no noise. The model assumes perfect recognition and
perfect localisation.

This is a defensible choice — it is fast enough to sit inside a planner loop,
which rendering is not, and it measures *viewpoint quality* rather than
perception robustness. But the paper must say plainly that it does not measure
whether a real detector would find these deviations, and the abstract must not
imply otherwise.

### Issue 6 — the framing does not match the stated task

**Severity: medium, and it is an opportunity rather than a defect.**

The task as described is for the robot to gather enough information to *make a
decision* about the unmodelled objects and the window. The pipeline measures
something else: binary detected/not-detected against a distance budget.

A decision task wants graded evidence and a stopping rule — travel until belief
about every element crosses a confidence threshold, then report
distance-to-confident-decision. That reframing is strictly stronger, because
"how much looking is enough" is a question coverage has no answer to at all,
whereas "what fraction did you see" is one it answers well.

Changing this touches `detection.py` and `evaluate.py`. Estimated a day. It is
probably out of reach for this deadline and belongs in the follow-up.

### Issue 7 — administrative

**Severity: low individually, blocking collectively.**

- `\thanks` in `deltaseek.tex` is a placeholder; affiliation and contact are
  missing.
- Reference `\bibitem{TODOactive}` is an explicit placeholder for the active
  sensing construction work. The other 16 entries were written from memory and
  **every one needs verifying against the real bibliographic record** before
  submission.
- **No git remote is configured.** Eight commits, the paper, and all figures
  exist on one disk with no backup.
- `ERS_B_STRUCT.ifc` is untracked. Its provenance and whether it may be
  published are unknown to me; if a figure derived from it goes in the paper,
  that question has to be answered.
- The submission deadline was understood to be the 24th. **Confirm this against
  the workshop site** — the whole plan below depends on it.

---

## 5. Recommendation

If the deadline really is the 24th, there are roughly two working days, and
Issues 2 and 3 together are about a day and a half. That is tight but not
impossible, and the order matters:

1. **Run the lidar ablation first (Issue 3).** It is the only item that can
   invalidate the thesis. Discovering that after rewriting the paper would
   waste the remaining time; discovering it first costs a day and redirects
   everything.
2. **Then randomise the clutter (Issue 2).** This turns the room result from an
   anecdote into a claim with an error bar.
3. **Then rewrite** around whichever story survives, most likely the
   two-regime framing.
4. Fix the administrative items in parallel — they need no experiments, and the
   missing git remote should be fixed today regardless of anything else.

If the deadline cannot accommodate that, the honest fallback is to submit the
existing negative-result paper. It is complete, compiled, and true. A workshop
is a reasonable venue for "the obvious approach does not work and here is a
reproducible benchmark showing why."

What should **not** happen is submitting the room result as the headline
without replication. One seed and six deviations will not survive a reviewer
who asks for a standard deviation.
