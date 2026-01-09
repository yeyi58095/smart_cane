# Introduction for Each Package

## `smart_cane_sim`

`smart_cane_sim` is for simulation-related, contains `models/`, `urdf/`, `worlds/` to let gazebo render the world, and contains `launch/` for launching the program.

This package is for slam-ing

## `smart_cane_perception`

Now, this package is proposed for image recognition. The main file to update is `yolo_lidar_landmark.py`, this file is using yolo to image recognition, the others is just testing the accessibility such as whether image from virtual camera in gazebo is reachable or not with openCV, simpler image recognition method. Additionally, this file, combine the data from LiDAR, so that we can localate the aproximate location onto the recongnized landmark. With locating the approximate location, it will call the function and method in `smart_cane_landmakrs/goal_snapper.py` to filter the hundreds of possible position to one most possible position.

To import the python packages for yolo, a virtual environment was created named as `rosenv`, and had been installed related packages. However, when we use `ros2 run ` or related commend, the executed program is from `install/` that autoally generated during compiling when `colcon build`,  and add some intrusment like `#!/usr/bin/evn python` to use the system default python instead custom environment python. Hence, a file `fix_yolo_shebang.py` is created for replacing to let python shift to use expected environment.

## `smart_cane_bringup`

This package was created for setting up the needed program once with launching, or executable file. `launch/amcl_sim.launch.py` and `initialpose_pub.py` is included, the latter is for publishing the initial coordination. there are serveral steps done during `amcl_sim.launch.py`, step-by-step is, changing the head of the `yolo_sign_detector`, activating the simulation world and robot, activating the navigation2 for amcl, publishing the initial pose for amcl, activating the yolo_node and so on.

### This detail and function each file executing

* `launch/amcl_sim.launch.py`: To activate the node what necessary for amcl-ing and navigating, image recognition without labeling and recording was also be added in it. The order of activating the node is
  * `fix_shebang`: changing the head for yolo-related files.
  * `sim_world`: activiating the simulation world, and placing the turtlebot onto the world.
  * `nav2`: activiating the navigation-related commends, written by ros2 developers.
  * `initialpose`: we need to publish the initial position of the turtlebot initially.
  * `yolo_node`: detecting the surrounding objects.
  * `rviz_node`: activiating the rviz node that parameter had been setted.


## `smart_cane_landmarks`

This package is related to LANDMARK, including filtering the all possible position detected by LiDAR and camera, which done by `goal_snapper.py`, navigating to the designated landmark by user, which done by `goto_landmark.py`, and publish the marker so that the corrdination in `landmarks.txt` can be shown on the rivz, which done by `landmark_visualzer.py`.

The form for using `goto_landmark.py` is `ros2 run smart_cane_landmarks DESTINATION`
