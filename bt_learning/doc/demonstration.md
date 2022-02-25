# Demonstration Format
All demonstrations are stored in YAML files. One series of demonstrations is a directory containing directories `demo1`, `demo2`, ..., `demoN` for each demonstration, and an `info.yaml` file. Each demonstration directory in turn contains YAML files `data_1.yaml`, `data_2.yaml`, ..., `data_K.yaml` for each demonstrated action in the task. The file `info.yaml` contains information about available reference frames and default reference frame.

Example file structure for a series of demonstrations of a task:
```
task/
 ├── demo1/
 │    ├── data_1.yaml
 │    ├── data_2.yaml
 │    └── data_3.yaml
 ├── demo2/
 │    ├── data_1.yaml
 │    └── data_2.yaml
 └── info.yaml
```

The YAML files contain a field `vec_pos` with fields for each frame containing the taraget position. Similarily, `vec_quat` contains target orientation quaternions. The field `type` contains the annotated type of the action.

Example YAML file for a picking action:
```YAML
type: pick
vec_pos:
  A: [-0.0013425822135018084, 0.01170481157653755, 0.017696849864566086]
  D: [-0.01782831744782143, -0.08395474054058023, 0.014197105265681131]
  E: [0.07103208572743952, -0.013148624256369285, 0.015340850148278684]
  F: [0.11320364255236381, -0.038074429964779444, 0.012556165783961862]
  marker_61: [0.09157772896470023, -0.018709829748717946, 0.14601130830561176]
  yumi_base_link: [0.49252288818359374, 0.037978500366210935, 0.07990389251708985]
vec_quat:
  A: [0.05880858030727856, -0.6792410383999195, 0.7311623150883596, 0.023975646608011197]
  D: [0.059482777143942026, -0.5393003885889096, 0.8399424346414689, 0.010658169620422885]
  E: [0.053632249851261186, -0.37075008597251896, 0.9271521103924982, 0.007544516021471585]
  F: [0.05357205989702771, -0.7858056782158152, 0.614600423344811, 0.0436553559892875]
  marker_61: [0.09060197691797679, -0.015787533661608847, 0.692462194456621, 0.715568406798231]
  yumi_base_link: [0.03974246278042145, -0.5747181837306714, 0.8173089118591473, 0.011211089914948891]

```

Example `info.yaml`:
```yaml
default_frame: yumi_base_link
frames:
- yumi_base_link
- A
- D
- E
- F
- marker_61
```

Note that in the [demonstration script](../bt_learning/learning_from_demo/demonstration.py) it is possible to set the rule that determines how to select the action's target [position](../bt_learning/learning_from_demo/demonstration.py#L238) and [orientation](../bt_learning/learning_from_demo/demonstration.py#L248).