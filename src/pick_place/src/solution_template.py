#!/usr/bin/env python

# Python 2/3 compatibility imports
#from __future__ import print_function
#from six.moves import input
import rospy
import sys
import tf_conversions
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import copy
import tf2_msgs.msg
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose
from path_planner.srv import *
from tf.transformations import *
from moveit_msgs.msg import Grasp
from pickle import TRUE
from gazebo_msgs.srv import GetModelState

class Planner():

  def __init__(self):
    #TODO: Initialise move it interface
      #Initialize moveit commander
      
      moveit_commander.roscpp_initialize(sys.argv)
      #Init node
      rospy.init_node('TaskSolution', anonymous=True)
      #Create robot commander object
      robot = moveit_commander.RobotCommander()
      #Create scene interface to manipulate the scene
      scene = moveit_commander.PlanningSceneInterface()
      #Group name, the name of our robot
      group_name = "xarm6" 
      #Move group interface initialized with the name of our robot
      move_group = moveit_commander.MoveGroupCommander(group_name)
      #Add group gripper
      group_name_gripper = "xarm_gripper"
      gripper_move_group = moveit_commander.MoveGroupCommander(group_name_gripper)
      #Initialize the publisher for displaying the planned path
      display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

      # Get the name of the reference frame for this robot:
      planning_frame = move_group.get_planning_frame()
      print("============ Planning frame: %s" % planning_frame)

      # Print the name of the end-effector link for this group:
      eef_link = move_group.get_end_effector_link()
      print("============ End effector link: %s" % eef_link)

      # Get and print a list of all the groups in the robot:
      group_names = robot.get_group_names()
      print("============ Available Planning Groups:", robot.get_group_names())
      
      #Print robot state
      print("============ Printing robot state")
      print(robot.get_current_state())
      print("")

      # Misc variables 
      self.box_name = ""
      self.robot = robot
      self.scene = scene
      self.move_group = move_group
      self.gripper_move_group = gripper_move_group
      self.display_trajectory_publisher = display_trajectory_publisher
      self.planning_frame = planning_frame
      self.eef_link = eef_link
      self.group_names = group_names
      #Get the inicial coordinates of models using ros service 
      # get_model_state from gazebo
      self.model_initial_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

  def wait_for_state_update(self,box_name, box_is_known=False, box_is_attached=False, timeout=1):
    #TODO: Whenever we change something in moveit we need to make sure that the interface has been updated properly
    
    #Get the start time to compute when we ran into a timeout
    start = rospy.get_time()
    seconds = rospy.get_time()
    #While we is not a time out
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = self.scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      
      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = self.box_name in self.scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

      # If we exited the while loop without returning then we timed out
      return False

  #Method used to extract XYZ coordinates of a frame
  def lookCordinates(self, frame):
    #Get te coordinates struct of a frame
    #  with reference on ground_plane
    resp_cordinates = self.model_initial_coordinates(frame,'ground_plane')
    dx = resp_cordinates.pose.position.x
    dy = resp_cordinates.pose.position.y
    dz = resp_cordinates.pose.position.z
    coodinates = [dx, dy, dz]
    return coodinates


  def addObstacles(self):
      
    #TODO: Add obstables in the world

    #target names
    targets = ["RedBox",
               "BlueBox",
               "GreenBox"]
    #goal names
    boxes = ["DepositBoxGreen",
               "DepositBoxRed",
               "DepositBoxBlue"]

    
    #Define de boxes position
    posRedBox = self.lookCordinates(targets[0])
    posBlueBox = self.lookCordinates(targets[1])
    posGreenBox = self.lookCordinates(targets[2])
    #Define xarm pos & path variables
    posXarm = self.lookCordinates('xarm6')

    #Append this positions to self
    self.posGreenBox = posGreenBox
    self.posRedBox = posRedBox
    self.posBlueBox = posBlueBox
    self.posXarm = posXarm



  def goToPose(self,pose_goal):
    #TODO: Code used to move to a given position using move it
    #Copy global variables into local 
    move_group = self.move_group
    posXarm = self.posXarm
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    #Initialize waypoints vector & append current pose
    waypoints = []
    
    #Since moveit works with relative position to the xarm
    # we calculate path positions substracting the xarm position
    # form the absolute goal position
    #NOTE: X & Y coordinates are switched because gazebo and rviz does
    # not have same orientation
    wpose = move_group.get_current_pose().pose
    wpose.position.x = float(pose_goal[1])-posXarm[1]
    wpose.position.y = -float(pose_goal[0]-posXarm[0])
    #Add a little displacement on z to don't hit the box
    wpose.position.z = float(pose_goal[2])-posXarm[2]+0.2
    #Append waypoints
    waypoints.append(copy.deepcopy(wpose))

    
    #Move down to take the box
    wpose.position.z = -0.001
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
    waypoints,   # waypoints to follow
    0.01,        # eef_step
    0.0)         # jump_threshold
    #Display trajectory
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)
    move_group.execute(plan, wait=True)
    move_group.stop()
    #Clear targets
    move_group.clear_pose_targets()
    rospy.sleep(0.1)

  #Method used to return home after reaching last goal
  def goBackFromPose(self, last_goal):
    #Copy global variables into local 
    move_group = self.move_group
    posXarm = self.posXarm
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    #Initialize waypoints vector & append current pose
    waypoints = []
    #Get current pose
    wpose = move_group.get_current_pose().pose
    #Move up
    wpose.position.z = float(last_goal[2])-posXarm[2]+0.2
    #Append waypoint
    waypoints.append(copy.deepcopy(wpose))
    #Now move to any XY position
    wpose.position.x = 0.2
    wpose.position.y = 0.2
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
    waypoints,   # waypoints to follow
    0.01,        # eef_step
    0.0)         # jump_threshold
    #Display trajectory
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)
    move_group.execute(plan, wait=True)
    move_group.stop()
    # Clear targets
    move_group.clear_pose_targets()
    rospy.sleep(0.1)

  def detachBox(self,box_name):
  #TODO: Open the gripper and call the service that releases the box
    #Copy global variables into local 
    gripper_move_group = self.gripper_move_group
    #Declare open position
    open_position = 0.01
    #Get joint configuration from gripper
    joint_config = gripper_move_group.get_current_joint_values()
    print("-----ACTUAL Joint Configuration: ", joint_config)
    #Assing open position
    for i in range(6):
      joint_config[i] = open_position
    #Give the joint positions to the gripper planner
    gripper_move_group.go(joint_config, wait=True)
    #Stop gripper move
    gripper_move_group.stop()
    rospy.sleep(0.2)
    #Call attach service with false to dettach action
    att = rospy.ServiceProxy("AttachObject",AttachObject)
    resp = att(False, box_name)


  def attachBox(self,box_name):
  #TODO: Close the gripper and call the service that releases the box
  #Copy global variables into local 
    gripper_move_group = self.gripper_move_group
    #Declare open position
    close_position = 0.2
    #Get joint configuration from gripper
    joint_config = gripper_move_group.get_current_joint_values()
    print("-----ACTUAL Joint Configuration: ", joint_config)
    #Assing open position
    for i in range(6):
      joint_config[i] = close_position
    #Give the joint positions to the gripper planner
    gripper_move_group.go(joint_config, wait=True)
    #Stop gripper move
    gripper_move_group.stop()
    rospy.sleep(0.2)
    #Call attach service with true to execute attach action
    att = rospy.ServiceProxy("AttachObject",AttachObject)
    resp = att(True, box_name)




class myNode():
  def __init__(self):
    #TODO: Initialise ROS and create the service calls

    # Good practice trick, wait until the required services are online before continuing with the aplication
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')
    print("")

  def getGoal(self,action):
    #TODO: Call the service that will provide you with a suitable target for the movement
    try:
      #Call requestGoal service from pathplanner
      request_Goal = rospy.ServiceProxy('RequestGoal', RequestGoal)
      goal = request_Goal(action)
      return goal.goal
    except rospy.rospy.ServiceException as err:
      print("Service call failed: %s" %err)


  def tf_goal(self, goal):
    #TODO:Use tf2 to retrieve the position of the target with respect to the proper reference frame
    #Create tf buffer
    self.tf_buffer = tf2_ros.Buffer()
    self.tf2_listener = tf2_ros.TransformListener(self.tf_buffer)
    #Use lookup transform to return transformation of gripper frame to goal fram
    transform = self.tf_buffer.lookup_transform("sensor_frame",goal,rospy.Time(0),rospy.Duration(10))
    #Return translation positions
    x = transform.transform.translation.x
    y = transform.transform.translation.y
    z = transform.transform.translation.z
    tf_goal = [x, y, z]
    
    return tf_goal

  def main(self):
    #TODO: Main code that contains the aplication
    print("-------------------------------------------------------------")
    print("====|||||| WELCOME TO TEAM 5-MTY PickNPlace Project |||||====")
    print("-------------------------------------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")
    self.planner = Planner()
    self.planner.addObstacles()
    #self.planner.goToPose(self.planner.posRedBox)
    #self.planner.attachBox("RedBox")
    #self.planner.goBackFromPose(self.planner.posRedBox)
    for i in range(3):
        #get pick goal
        box = self.getGoal("pick")
        #Transform goal
        goal = self.tf_goal(box)
        print(goal)
        #Move to goal box
        self.planner.goToPose(goal)
        self.planner.attachBox(box)
        self.planner.goBackFromPose(goal)
        #Get deposit box position and tranforms
        deposit = self.getGoal("place")
        goal = self.tf_goal(deposit)
        self.planner.goToPose(goal)
        self.planner.detachBox(box)
        self.planner.goBackFromPose(goal)
  
    rospy.signal_shutdown("Task Completed")


if __name__ == '__main__':
  try:
    
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass