# Sensor-configuration ablation, Hall B

Detections by observability class, as `found / total`.

| class | chassis | wrist | both |
| --- | --- | --- | --- |
| trivial | 2/2 | 2/2 | 2/2 |
| height | 0/2 | 2/2 | 2/2 |
| incidence | 1/2 | 2/2 | 2/2 |
| occluded | 2/2 | 2/2 | 2/2 |
| **all** | **5/8** | **8/8** | **8/8** |

| configuration | median distance to detection |
| --- | --- |
| chassis | 14.2 m |
| wrist | 6.0 m |
| both | 8.9 m |

## Capability versus routing

Deviations the chassis camera cannot resolve from any candidate viewpoint, as opposed to ones it merely failed to reach inside the budget.

| deviation | class | chassis can ever see it |
| --- | --- | --- |
| T1-pallet-stack | trivial | yes |
| T2-site-cabin | trivial | yes |
| H1-rack-tote | height | **no** |
| H2-cabinet-carton | height | **no** |
| I1-window-displaced | incidence | yes |
| I2-standing-panel | incidence | yes |
| O1-crate-behind-stack | occluded | yes |
| O2-drum-behind-rack | occluded | yes |

## What the chassis camera pays instead

Deviations both configurations resolve, and the distance each spent getting there.

| deviation | class | chassis | wrist |
| --- | --- | --- | --- |
| T1-pallet-stack | trivial | 14.2 m | 12.9 m |
| T2-site-cabin | trivial | 14.2 m | 6.0 m |
| I1-window-displaced | incidence | 14.2 m | 6.0 m |
| O1-crate-behind-stack | occluded | 14.2 m | 24.2 m |
| O2-drum-behind-rack | occluded | 15.2 m | 0.0 m |

Per-deviation detail is in `sensor_ablation.csv`; the machine-readable form, including per-viewpoint observation counts, is in `sensor_ablation.json`.
