#!/usr/bin/python

import rospy
from simple_script_server import *
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from cob_srvs.srv import SetString, SetStringRequest, SetStringResponse

class ContrinuousNavigation():
    def __init__(self):
        rospy.Subscriber("/bms/remaining_capacity", Float64, self.bms_remaining_capacity_callback)
        rospy.Subscriber("/bms/current", Float64, self.bms_current_callback)
        self.dock = rospy.ServiceProxy('/docker_control/dock', SetString)
        self.undock = rospy.ServiceProxy('/docker_control/undock', SetString)
        self.sss = simple_script_server()
        self.battery_low = False
        self.charging = False

    def bms_current_callback(self, msg):
        if msg.data > 0:
            self.charging = True
        else:
            self.charging = False

    def bms_remaining_capacity_callback(self, msg):
        if msg.data < 10.0: # TODO: make this a dyn reconfigure parameter
            self.battery_low = True
        if msg.data > 30.0: # TODO: make this a dyn reconfigure parameter
            self.battery_low = False

    def check_for_recharging(self):
        print "self.battery_low", self.battery_low
        print "self.charging", self.charging
        if self.battery_low and not self.charging:
            self.sss.say(["My battery is low, docking"])
            handle_base = self.sss.move("base","charger")
            if handle_base.get_state() != 3:
                print "could not reach charger pose"
                return
            req = SetStringRequest()
            req.data = ''
            res = self.dock(req)
            if res.success:
                rate = rospy.Rate(1.0)
                while not rospy.is_shutdown() and self.battery_low and self.charging: # blocks until battery is full
                    rospy.loginfo("still charging.")
                    rate.sleep()
                self.sss.say(["My battery is full, undocking"])
                req = SetStringRequest()
                req.data = ''
                self.undock(req)

    def execute(self):
        while not rospy.is_shutdown():
            self.check_for_recharging()
            self.sss.move("base","entrance")
            rospy.sleep(10)

            self.check_for_recharging()
            self.sss.move("base","0")
            rospy.sleep(10)

            self.check_for_recharging()
            self.sss.move("base","1")
            rospy.sleep(10)

if __name__=='__main__':
    rospy.init_node("continuous_navigation")
    cn = ContrinuousNavigation()
    rospy.sleep(1)
    cn.execute()
    rospy.loginfo("continuous navigation running")
    rospy.spin()

