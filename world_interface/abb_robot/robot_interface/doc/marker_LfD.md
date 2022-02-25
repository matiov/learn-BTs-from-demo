# Running the LfD system

Open a ROS2 command prompt and source the workspace (Windows: `call src\install\setup.bat`, Linux: `source src/install/setup.bash`).  
First, bringup the robot and the Aruco marker detection:
```
ros2 launch camera_interface yumi_cube.launch.py
```
Then, start the LfD system:
```
ros2 launch robot_interface lfd.launch.py
```
Make sure that the `config` files in both packages have matching values for the `base_frame` and that the value for [`ee_frame`](https://github.com/matiov/learn-BTs-from-demo/blob/master/world_interface/abb_robot/robot_interface/config/lfd_markers.yaml#L8) conincides with [`to_frame`](https://github.com/matiov/learn-BTs-from-demo/blob/master/world_interface/camera_interface/config/cube_detection.yaml#L15).

If you want to start the system without connecting to the robot to for example use the planner and generate tree figures, set the parameter [has_robot](https://github.com/matiov/learn-BTs-from-demo/blob/master/world_interface/abb_robot/robot_interface/config/lfd_markers.yaml#L11) to `False`. Note that the buttons *Add demonstration* and *Run* are only displayed if the system is connected to the robot since they are not supported otherwise. An example of demonstrations is provided in the [demonstration folder](https://github.com/matiov/learn-BTs-from-demo/tree/master/bt_learning/bt_learning/learning_from_demo/demonstrations) for the task *'cube in the box'*.

![image](https://user-images.githubusercontent.com/68166261/125573312-e3e10363-7bf3-4621-9441-e8e40baa036b.png)

(*Note:* the GUI might look differently in Linux Ubuntu.)

### Loading demonstrations
Demonstrations can be loaded by clicking on *Select demo folder*. Either select an existing directory containing demonstrations as specified [here](https://github.com/matiov/learn-BTs-from-demo/blob/master/bt_learning/doc/demonstration.md), or right click to create a new empty directory and select it to create a new task with no demonstrations. Once a set of demonstrations has been loaded, a progress bar is shown while the tree is built.

When starting a new empty task, the default behavior is to only include frames *yumi_base_link*, *A*, *D*, *E*, *F*, and *marker_61* (the box). It is also possible to include frames *B*, *C*, and *marker_60* (the kitting box) and exclude additional frames. To do so, edit the [frames](https://github.com/matiov/learn-BTs-from-demo/blob/master/world_interface/abb_robot/robot_interface/config/lfd_markers.yaml#L7) parameter to change which the default frames are when starting a new task.

### Add demonstration
When clicking on *Add demonstration*, a new dialog is shown with actions to select from.

![image](https://user-images.githubusercontent.com/68166261/125755990-c421b98f-62fd-4f91-9353-17579e0ca02e.png)

The robot is automatically put in lead-through mode. An action is demonstrated by moving the end effector to the desired target pose and clicking on the action you just demonstrated.

| Action     | Description                                                                           | Effect when demonstrating |
| ---------- | ------------------------------------------------------------------------------------- | ------------------------- |
| Pick       | Pick an object.                                                                       | The gripper is closed.    |
| Fine place | Place an object and the precise placing position matters.                             | The gripper is opened.    |
| Drop       | Drop an object somewhere around the demonstrated position. Precision does not matter. | The gripper is opened.    |


### Run
When clicking on *Run*, the webbrowser is opened showing the executed tree, and the robot's arm is moved to home position. The execution of the tree is shown in the webbrowser with green indicating *Success*, red *Failure*, and orange *Running*. The *Run* button is also replaced with a *Stop* button that stops ticking the tree and preempts the current behavior.

### Show tree
An svg file containing a simplified overview is opened in the computer's default program for displaying svg files.

### Save tree
Figures of the tree in various formats can be saved. When clicking on *Save tree*, a dialog appears that lets you select a folder. If you select an existing folder, a folder named `btN` is created where `N` is incremented so there are no file name conflicts. If you select a directory that does not exist, it will be created and the figures stored directly in it. The table below gives an explanation of what each saved file contains. A *traditional* style BT is drawn with arrows for sequences and question marks for fallbacks. A *pytrees* style BT is drawn as the pytrees library draws them.

| File                       | Description                                                                                                     |
| -------------------------- | --------------------------------------------------------------------------------------------------------------- |
| `full.svg`                 | Full tree in traditional style.                                                                                 |
| `full.tex`                 | Full tree in traditional style for rendering with TikZ as was done in the thesis.                               |
| `full_pytrees.svg`         | Full tree in pytrees style.                                                                                     |
| `simplified.svg`           | Simplified tree in traditional style.                                                                           |
| `simplified.tex`           | Simplified tree in traditional style for rendering with TikZ as was done in the thesis.                         |
| `simplified_pytrees.svg`   | Simplified tree in pytrees style.                                                                               |
| `positions.yaml`           | Contains values for the position symbols p<sub>n</sub>.                                                         |
| `tree.yaml` and `settings` | Although the feature has not been implemented, these contain all information needed to load the tree from file. |

## Utilities
There are some utility scripts that can help with various tasks.

### Plotting clusters
There is a utility that shows the output of the clustering. To run it on a folder `demo` with demonstrations for a task, run `python -m robot_interface.plot_clusters demo`. The tool plots the target position of all actions in all frames. Points with the same color have been detected as equivalent by the system and they are represented with a cross in the frame in which they occur.

![cluster](https://user-images.githubusercontent.com/68166261/125589628-615ff618-8263-4514-a512-6b02e048a6c0.png)

For example, the figure above shows the output of the tool for the action *Place F* in the kitting task. There are three colors so the system has detected three groups of equivalent actions. Moreover, we can see that the green and blue groups are detected to occur in frame *marker_60* while the black group has been detected to occur in frame *B*.

### Combining demonstrations
Another utility exists for combining demonstrations from multiple people into one large set of demonstrations. It can be run with `python -m bt_learning.learning_from_demo.combine_demonstration` (run with `-h for help`). The options that can be used are listed below:

| Option            | Parameters              | Description                          |
| ----------------- | ----------------------- | ------------------------------------ |
| `--out`, `-o`     | `OUT`                   | Name of the output directory to create and store the combined demonstrations in. |
| `--demos`, `-d`   | `DEMOS [DEMOS ...]`     | List of demonstration directories to combine. Supports wildcards.                |
| `--exclude`, `-e` | `EXCLUDE [EXCLUDE ...]` | List of demonstration directories to exclude from --demos. Supports wildcards. To exclude specific demonstrations within a demonstration directory, separate them with '::'. Eg. demo::2::4 excludes demonstrations 2 and 4 from demo. |
| `--ndemos`, `-n`  | `NDEMOS`                | Number of demonstrations to include in the destination. They will be selected randomly from the demonstrations specified by --demos and --exclude. If left out, all demonstrations are copied. |

#### Example
Let's reconstruct the combination of all non-expert demos that was used for an experiment. Let's say the demonstrations are stored in `non_expert_demos` and we want to combine them into a folder `non_expert_combinations/all`. We want to exclude the demonstrations of person 3 since they didn't complete the task successfully, and the first demonstration of person 6 since *pick* was recorded incorrectly. We also want to exclude all tasks that the users came up with themselves, stored in folders `ùser_free`. Some demonstrations have been manually edited because the picking position was bad and the original recording is kept in `user_unedited`. We also want to exclude those. The final command for combining all the desired demonstrations is thus

```
python -m python -m bt_learning.learning_from_demo.combine_demonstration -o non_expert_combinations/all -d non_expert_demos/* -e non_expert_demos/person_3 non_expert_demos/person_6::1 non_expert_demos/*free non_expert_demos/*unedited
```

Consider now that we want to run an experiment with 10 demonstrations instead of all. We can use the `-n` option to select 10 demonstrations at random:

```
python -m python -m bt_learning.learning_from_demo.combine_demonstration -o non_expert_combinations/all_10 -n 10 -d non_expert_demos/* -e non_expert_demos/person_3 non_expert_demos/person_6::1 non_expert_demos/*free non_expert_demos/*unedited
```

Note that since the demonstrations are selected at random, there is a probability that most come from the same person. If you want a different set of demonstrations you can simply rerun the command.
