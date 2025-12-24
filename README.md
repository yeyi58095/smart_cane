# Introduction for Each Package

## `smart_cane_sim`

`smart_cane_sim` is for simulation-related, contains `models/`, `urdf/`, `worlds/` to let gazebo render the world, and contains `launch/` for launching the program.

This package is for slam-ing

## `smart_cane_perception`

Now, this package is proposed for image recognition. The main file to update is `yolo_sign_detector.py`, this file is using yolo to image recognition, the others is just testing the accessibility such as whether image from virtual camera in gazebo is reachable or not with openCV, simpler image recognition method.
To import the python packages for yolo, a virtual environment was created named as `rosenv`, and had been installed related packages. However, when we use `ros2 run ` or related commend, the executed program is from `install/` that autoally generated during compiling when `colcon build`,  and add some intrusment like `#!/usr/bin/evn python` to use the system default python instead custom environment python. Hence, a file `fix_yolo_shebang.py` is created for replacing to let python shift to use expected environment.

## `smart_cane_bringup`

This package was created for setting up the needed program once with launching, or executable file. `launch/amcl_sim.launch.py` and `initialpose_pub.py` is included, the latter is for publishing the initial coordination. there are serveral steps done during `amcl_sim.launch.py`, step-by-step is, changing the head of the `yolo_sign_detector`, activating the simulation world and robot, activating the navigation2 for amcl, publishing the initial pose for amcl, activating the yolo_node and so on.
