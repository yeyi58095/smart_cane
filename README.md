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
  * `initialpose`: we need to publish the initial position of the turtlebot initially.  (this had been replaced to under `sim_world.launch`)
  * `yolo_node`: detecting the surrounding objects.
  * `rviz_node`: activiating the rviz node that parameter had been setted.
 
* `rviz/amcl_with_landmark_shown_or_creating.rviz`: Including the chosen topics with initializated. Including
  * `LaserScan`
  * `Image`
  * `Map`
  * `Polygon`: show the robot position
  * `Path`
  * `MarkerArray`
  and so on.

## `smart_cane_landmarks`

This package is related to LANDMARK, including filtering the all possible position detected by LiDAR and camera, which done by `goal_snapper.py`, navigating to the designated landmark by user, which done by `goto_landmark.py`, and publish the marker so that the corrdination in `landmarks.txt` can be shown on the rivz, which done by `landmark_visualzer.py`.

The form for using `goto_landmark.py` is `ros2 run smart_cane_landmarks DESTINATION`

## `smart_cane_nav`

This package is related to navigation, including launching the fils in nav2_bringup written by other developers, and some file I wrote for indicating the direction published by `/cmd_vel` or `/nav_cmd_vel` which I defined self-ly.

### The detail of each file included this package

`launch/nav2_autonomy.launch.py` and `launch/nav2_guidance.launch.py`: Both of these are for applying method `nav2_bringup` offering. But the former publish `/cmd_vel` and control tb3 directly, and the latter publish `/nav_cmd_vel` for indicating the inforation about direction to user with piping with other file majorly.

`qt_cmd_vel_ui.py`: Offer the GUI for user to visualize the direction and the (angle) vecolity that `/nav_cmd_vel` indicating. Also, user can publish the `/cmd_vel` to control the tb3 actually.

`nav_cmd_vel_ui.py`: Just only indicating the orientation in CLI interface.
