#!/usr/bin/env python

# Python 2/3 compatibility imports
#from __future__ import print_function
#from six.moves import input

from typing_extensions import Self
import rospy
import sys
import tf_conversions
import tf2_ros
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose
from path_planner.srv import *
from tf.transformations import *
from moveit_msgs.msg import Grasp

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
      self.display_trajectory_publisher = display_trajectory_publisher
      self.planning_frame = planning_frame
      self.eef_link = eef_link
      self.group_names = group_names

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

    #Define de box position
    posGreenBox=[-0.186303,0.411113,1.045000]
    posRedBox = [-0.516300, 0.4, 1.044900]
    posBlueBox = [-0.560639, 0.275921, 1.045000]
    #Append this positions to self
    self.posGreenBox = posGreenBox
    self.posRedBox = posRedBox
    self.posBlueBox = posBlueBox

    #Define box sizes and append it
    boxsize = [0.6, 0.6, 0.6]
    self.boxSize = boxsize


  def goToPose(self,pose_goal):
      pass
    #TODO: Code used to move to a given position using move it


  def detachBox(self,box_name):
      pass
  #TODO: Open the gripper and call the service that releases the box


  def attachBox(self,box_name):
      pass
  #TODO: Close the gripper and call the service that releases the box



class myNode():
  def __init__(self):
    #TODO: Initialise ROS and create the service calls

    # Good practice trick, wait until the required services are online before continuing with the aplication
    #rospy.wait_for_service('RequestGoal')
    #rospy.wait_for_service('AttachObject')
    print("")

  def getGoal(self,action):
      pass
    #TODO: Call the service that will provide you with a suitable target for the movement


  def tf_goal(self, goal):
      pass
    #TODO:Use tf2 to retrieve the position of the target with respect to the proper reference frame


  def main(self):
    #TODO: Main code that contains the aplication
    print("-------------------------------------------------------------")
    print("====|||||| WELCOME TO TEAM 5-MTY PickNPlace Project |||||====")
    print("-------------------------------------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")
    self.planner = Planner()
    self.planner.addObstacles()

    rospy.signal_shutdown("Task Completed")



if __name__ == '__main__':
  try:
    
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass


