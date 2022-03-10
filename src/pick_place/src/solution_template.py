#!/usr/bin/env python
from pickle import TRUE
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
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('TaskSolution', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "xarm6" 
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")
    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = robot.get_group_names()

    # Misc variables 
    self.box_name = ""
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def wait_for_state_update(self,box_name, box_is_known=False, box_is_attached=False, timeout=0.5):

    #TODO: Whenever we change something in moveit we need to make sure that the interface has been updated properly
    start = rospy.get_time()
    seconds = rospy.get_time()
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

	
    #Cargo names
    targets = ["RedBox",
               "BlueBox",
               "GreenBox"]
    #goal names
    boxes = ["DepositBoxGreen",
               "DepositBoxRed",
               "DepositBoxBlue"]

  def goToPose(self,pose_goal):

    #TODO: Code used to move to a given position using move it
    pose_goal = Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    self.move_group.set_pose_targets(pose_goal)
    self.move_group.go(wait= TRUE)
    #move_group.execute(plan, wait=True)
    self.move_group.stop()
    self.move_group.clear_pose_targets()
    


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
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')

  def getGoal(self,action):
    pass
    #TODO: Call the service that will provide you with a suitable target for the movement


  def tf_goal(self, goal):
    pass
    #TODO:Use tf2 to retrieve the position of the target with respect to the proper reference frame


  def main(self):
    #TODO: Main code that contains the aplication
    self.planner = Planner()
    self.planner.addObstacles()

    rospy.signal_shutdown("Task Completed")



if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
