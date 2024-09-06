# Getting certian errors for this package, unable to launch the executable. 
Getting the following error: 

atharv@atharv-ROG-Strix-G512LI-G512LI:~/assignment_ws$ ros2 run question_3_pkg move_program 
[INFO] [1725603982.045561308] [moveit_rdf_loader.rdf_loader]: Robot model parameter not found! Did you remap 'robot_description'?

[FATAL] [1725603982.045698638] [move_group_interface]: Unable to construct robot model. Please make sure all needed information is on the parameter server.
terminate called after throwing an instance of 'std::runtime_error'
  what():  Unable to construct robot model. Please make sure all needed information is on the parameter server.

Upon typing ros2 param list, able to find the robot_description parameter. Have to debug the error. 

Rviz + Panda simulator for Moveit2 in ROS2 Foxy distribution is working. Once I run my custom executable, I am getting the error. 
  
