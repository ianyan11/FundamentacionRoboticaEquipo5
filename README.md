# Challenge: Pick and place 

This workspace contains a bunch of packages related with the usage of a xarm manipulator arm (either the model 5,6,7). 
You will only have to place your codes inside the folders "path_planner" and "pick_place". In order to start the
simulation the following codes need to be launched following this order. 

Friendly reminder, the world has already been created for you and you don't need to modify any of the gazebo files, just acces them to check the physical dimensions of the boxes. All their poses are forwarded as tf2 transforms. 

0) Download the 'table' 3D model: In Gazebo simulator, navigate through the model database for 'table' item, drag and place
   the 3D model inside the virtual environment. It will then be downloaded locally, as 'table' is needed for running the demo.
   An example of how to do this can be found in Rebeca's sesion.
1) xarm_gazebo xarm6_challenge.launch
2) xarm6_gripper_moveit_config xarm6_gripper_moveit_gazebo.launch
3) rosrun path_planner GoalPlanner.py
4) rosrun pick_place solution_template.py (your solution should be here)

Once those launch files are launched the system is ready to run your own codes. 

