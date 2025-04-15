Project made on Docker for ROS1 Noetic
All necessary packages should be in this repository.
Important: You need to install Xming and run Xlaunch before you can run GUI.
To source type source ~/.bashrc
to run Rviz type roslaunch uav_gimbal rviz_viewer.launch
to run the uav and CV Simulation, type rosrun uav_gimbal uav_sim.py 
If you want to edit the files you have to go to ~/catkin_ws/src/uav_gimbal
there you will see
launch: contains launch files. The only one that is relevant is rviz_viewer.launch
scripts: contains python scripts(also "nodes"). The relevant ones are uav_comms_interface.py and uav_sim.py
robot_description: contains the rviz configuration and URDF files.
If Rviz is not displaying the robot, check if rviz_viewer.rviz is there, if not, manually add a robot using displays in the Rviz menu and save the config in robot_description as rviz_viewer.rviz
