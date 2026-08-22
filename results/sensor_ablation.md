# Sensor-configuration ablation, Hall B

Detections by observability class, as `found / total`.

| class | chassis | wrist | both |
| --- | --- | --- | --- |
| trivial | 2/2 | 2/2 | 2/2 |
| height | 0/2 | 2/2 | 2/2 |
| incidence | 2/2 | 2/2 | 2/2 |
| occluded | 2/2 | 2/2 | 2/2 |
| **all** | **6/8** | **8/8** | **8/8** |

| configuration | median distance to detection |
| --- | --- |
| chassis | 15.2 m |
| wrist | 8.0 m |
| both | 10.4 m |

## What the chassis camera pays instead

Deviations both configurations resolve, and the distance each spent getting there.

| deviation | class | chassis | wrist |
| --- | --- | --- | --- |
| T1-pallet-stack | trivial | 15.2 m | 11.4 m |
| T2-site-cabin | trivial | 14.2 m | 9.0 m |
| I1-window-displaced | incidence | 15.2 m | 8.0 m |
| I2-standing-panel | incidence | 15.2 m | 8.0 m |
| O1-crate-behind-stack | occluded | 15.2 m | 8.0 m |
| O2-drum-behind-rack | occluded | 0.0 m | 0.0 m |

Per-deviation detail is in `sensor_ablation.csv`.
