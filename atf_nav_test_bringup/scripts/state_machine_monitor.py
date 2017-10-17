#!/usr/bin/python
#
# \file
#
# \note
#   Copyright (c) 2017 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_command_tools
# \note
#   ROS package name: cob_monitoring
#
# \author
#   Author: Benjamin Maidel
# \author
#   Supervised by:
#
# \date Date of creation: February 2017
#
# \brief
#   Monitors the state_machine action state and annonces waring via light if not active
#
#
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#

import rospy
import copy
import actionlib

from std_msgs.msg import *
from cob_msgs.msg import *
from cob_light.msg import *
from cob_light.srv import *

from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalStatus


class StateMachineMonitor():
    def __init__(self):
        self.mode = LightMode()
        self.mode.priority = 9
        self.track_id_light = None
        self.last_time_state_received = None
        self.warn_light_active = False
        self.sm_active = False

        rospy.Subscriber("/state_machine/machine/status", GoalStatusArray, self.state_callback)

        rospy.Timer(rospy.Duration(1), self.timer_callback)

    def state_callback(self, msg):
        if len(msg.status_list) > 0:
            if msg.status_list[0].status == GoalStatus.ACTIVE:
                self.sm_active = True
            else:
                self.sm_active = False
            self.last_time_state_received = rospy.get_time()

    def set_light(self, mode):
        action_server_name = "/light_torso/set_light"
        client = actionlib.SimpleActionClient(action_server_name, SetLightModeAction)
        # trying to connect to server
        rospy.logdebug("waiting for %s action server to start",action_server_name)
        if not client.wait_for_server(rospy.Duration(5)):
            # error: server did not respond
            rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
            return False
        else:
            rospy.logdebug("%s action server ready",action_server_name)

        # sending goal
        goal = SetLightModeGoal()
        goal.mode = mode
        client.send_goal(goal)
        client.wait_for_result()
        res = client.get_result()
        self.track_id_light = res.track_id
        return True

    def stop_light(self):
        if self.track_id_light is not None:
            srv_server_name = "/light_torso/stop_mode"
            try:
                rospy.wait_for_service(srv_server_name, timeout=2)
                srv_proxy = rospy.ServiceProxy(srv_server_name, StopLightMode)
                req = StopLightModeRequest()
                req.track_id = self.track_id_light
                srv_proxy(req)
                self.track_id_light = None
            except Exception as e:
                rospy.logerr("%s service failed: %s",srv_server_name, e)
                return False
            return True
        return False

    def timer_callback(self, event):
        #warn if statemachine is not running
        if not self.sm_active or self.last_time_state_received==None\
            or (self.last_time_state_received is not None and (rospy.get_time()-self.last_time_state_received) > 2.0):
            if not self.warn_light_active:
                mode = copy.copy(self.mode)
                mode.mode = LightModes.STATIC
                color = ColorRGBA(1, 1, 0, 1)
                mode.colors = []
                mode.colors.append(color)
                if self.set_light(mode):
                    self.warn_light_active = True
            return

        if self.sm_active\
            and (self.last_time_state_received is not None and (rospy.get_time()-self.last_time_state_received) <= 2.0):
            if self.warn_light_active:
                self.stop_light()
                self.warn_light_active = False
            return

if __name__ == "__main__":
    rospy.init_node("state_machine_monitor")
    monitor = StateMachineMonitor()
    rospy.loginfo("state_machine_monitor started")
    rospy.spin()
    monitor.stop_light()
