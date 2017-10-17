#!/usr/bin/python

import rospy
import actionlib

from actionlib.msg import TestAction, TestGoal

class StartErrorMonitor():

  def __init__(self):
    self.ac_error_monitor_ = actionlib.SimpleActionClient('/msh/error_monitor', actionlib.msg.TestAction)

    # start state machine
    rospy.loginfo("[start error monitor]: waiting for error monitor action server")
    self.ac_error_monitor_.wait_for_server()
    rospy.loginfo("[start error monitor]: send action")
    goal = TestGoal()
    self.ac_error_monitor_.send_goal(goal)

if __name__ == "__main__":
  rospy.init_node("start_error_monitor")
  rospy.loginfo("[start error monitor]: running")
  SM = StartErrorMonitor()
  rospy.loginfo("[start error monitor]: finished")
