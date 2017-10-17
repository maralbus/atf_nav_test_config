#!/usr/bin/env python

import os
import sys
import rospy
from std_msgs.msg import String
from msh_msgs.msg import RobotStatus, ErrorState
from optparse import OptionParser

class alert_error:
    def __init__(self):
        self.error_count = 0
        #self.initial = True
        self.popup_open = False
        self.timestamp_update = rospy.Time.now()
        self.timestamp_error = rospy.Time.now()
        self.timeout_warn = rospy.Duration(30)
        self.timeout_error = rospy.Duration(60)
        self.frequency = 1000
        self.duration_in_sec = 0.5
        self.error_state = RobotStatus().error_state

    def robot_status_cb(self, msg):
        self.timestamp_update = rospy.Time.now()
        self.error_state = msg.error_state

    def check_for_alarm(self):
        duration = rospy.Time.now() - self.timestamp_update
        if self.error_state.id != ErrorState.OK:
            duration_error = rospy.Time.now() - self.timestamp_error
            if duration_error > self.timeout_warn:
                self.make_alarm()
                rospy.logerr("Error\n" + str(self.error_state))
        elif duration > self.timeout_error:
            self.make_alarm()
            rospy.logerr("Did not hear something from the robot for %d seconds"%duration.to_sec())
        elif duration > self.timeout_warn:
            rospy.logwarn("Did not hear something from the robot for %d seconds"%duration.to_sec())
        else:
            if self.error_state.message:
                rospy.loginfo("Everything OK\n" + str(self.error_state))
            self.timestamp_error = rospy.Time.now()

    def make_alarm(self):
        os.system(('play -n synth %f sin %s vol -12 db 2> /dev/null') % (self.duration_in_sec, self.frequency))
        # check if popup is activated and to avoid double-showing the popup
        if self.popup and (not self.popup_open) and (rospy.Time.now() - self.timestamp_update > self.timeout_error or self.error_count > 10):
            os.system('(zenity --error --text="An error occurred\!" --title="Paul is in danger!")')
            self.popup_open = True

    def run(self, popup):
        self.popup = popup
        rospy.Subscriber("/robot_status", RobotStatus, self.robot_status_cb, queue_size=1)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.check_for_alarm()
            rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    parser = OptionParser(usage="%prog -p or --popup", prog=os.path.basename(sys.argv[0]))
    parser.add_option("-p", "--popup", action="store_true", dest="popup", default=False, help="Use to show popup on errors")

    (options, args) = parser.parse_args()
    rospy.init_node('robot_error_alert', anonymous=True)
    print "\nRobot error alerter is running ... \n", "Popup: ", options.popup, "\n"
    ae = alert_error()
    ae.run(options.popup)
