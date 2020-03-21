1.Paste iros_track folder in Firmware/Tools/sitl_gazebo/models.
2.Paste frames.world in Firmware/Tools/sitl_gazebo/worlds.
3.Paste iris_fpv_frames.launch in Firmware/launch.
4.Put this code in .bashrc--> export GAZEBO_RESOURCE_PATH=~/ros_ws1/src/Firmware/Tools/sitl_gazebo
5.Build Firmware.
6.Run enviroment using roslaunch enviroment env.launch. 