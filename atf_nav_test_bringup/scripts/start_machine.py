#!/usr/bin/python

import rospy
import actionlib

from actionlib.msg import TestAction, TestGoal

class StartMachine():

  def __init__(self):
    self.ac_state_machine_ = actionlib.SimpleActionClient('/state_machine/machine', actionlib.msg.TestAction)

    # start state machine
    rospy.loginfo("[start machine]: waiting for state machine action server")
    self.ac_state_machine_.wait_for_server()
    rospy.loginfo("[start machine]: send action")
    goal = TestGoal()
    self.ac_state_machine_.send_goal(goal)

if __name__ == "__main__":
  rospy.init_node("start_machine")
  rospy.loginfo("[start machine]: running")
  SM = StartMachine()
  rospy.loginfo("[start machine]: finished")
