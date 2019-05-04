# cartesio_gui
GUI tools for the CartesI/O framework. Currently available:
- **cartesio_rviz_gui**: manage CartesI/O tasks with a dockable GUI for RViz
- **joint_state_sliders**: a replacement for ROS joint_state_publisher, with chain-wise organization to ease the control of complex robots like humanoids. When used inside the *XBotCore* framework, it also allows **online impedance modulation**.

## cartesio_rviz_gui
Load it from RViz->Panels->Add New Panel

## joint_state_sliders
Two ready-to-use launch files available:
 - `cartesio_posture_sliders.launch`, to use the gui to control a postural task in CartesI/O
 - `xbot_joint_command_sliders.launch`, to control a XBotCore-powered robot in joint space
 
Also check out the [**rosmon**](http://wiki.ros.org/rosmon) utility, which is a smarter roslaunch!
