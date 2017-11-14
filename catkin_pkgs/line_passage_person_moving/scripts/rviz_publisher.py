#!/usr/bin/python

"""
Created on Sep 6, 2017

@author: flg-ma
@attention: Auto Position Publisher for RVIZ
@contact: albus.marcel@gmail.com (Marcel Albus)
@version: 1.3.2
"""

import os
import xml.etree.ElementTree as ET
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseActionGoal
import tf


# colors in terminal prints
class TerminalColors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class RvizPublisher():
    def __init__(self):
        self.tc = TerminalColors()
        # topics you can publish
        self.topics = {'/initialpose': PoseWithCovarianceStamped,
                       '/move_base_simple/goal': PoseStamped,
                       '/clicked_point': PointStamped,
                       '/move_base/goal': MoveBaseActionGoal}

        self.publisher = {}
        for topic, msg_type in self.topics.iteritems():
            self.publisher[topic] = rospy.Publisher(topic, msg_type, latch=True, queue_size=10)

        # python bug... sleep NEEDED!(ros tired...) Min: 0.5 sec
        # rospy.sleep(1.0)

    def __del__(self):
        print self.tc.OKGREEN, 'deleting ', self, self.tc.ENDC

    def euler2quaternion(self, roll, pitch, yaw):
        '''
        converts euler to quaternion
        :param roll: angle around x-axis
        :param pitch: angle around  y-axis
        :param yaw: angle around z-axis
        :return: quaternion of euler-angles
        '''
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return quaternion

    def setupMessage(self, msg_type, frame_id, position_x, position_y, *args):
        '''
        build up message to publish on node
        :param msg_type: type of the message to publish
        :param frame_id: mostly 'map'
        :param position_x: x-position on map
        :param position_y: y-position on map
        :param args: roll, pitch, yaw angle
        :return: fully setup message
        '''
        msg = msg_type()
        # msg.header.seq = 1
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        args = list(args)
        # get roll, pitch, yaw from additional parameters
        roll = args[0]
        pitch = args[1]
        yaw = args[2]
        # convert euler-angles to quaternions
        quaternion = self.euler2quaternion(roll, pitch, yaw)

        if msg_type is PoseWithCovarianceStamped:
            msg.pose.pose.position.x = position_x
            msg.pose.pose.position.y = position_y
            msg.pose.pose.orientation.x = quaternion[0]
            msg.pose.pose.orientation.y = quaternion[1]
            msg.pose.pose.orientation.z = quaternion[2]
            msg.pose.pose.orientation.w = quaternion[3]
            # needed to prevent robot from drifting off
            msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.06853891945200942]
        elif msg_type is PoseStamped:
            msg.pose.position.x = position_x
            msg.pose.position.y = position_y
            msg.pose.orientation.x = quaternion[0]
            msg.pose.orientation.y = quaternion[1]
            msg.pose.orientation.z = quaternion[2]
            msg.pose.orientation.w = quaternion[3]

        elif msg_type is MoveBaseActionGoal:
            msg.goal_id.stamp = rospy.Time.now()
            msg.goal_id.id = 'rviz_publisher_goal'

            msg.goal.target_pose.header.seq = 1
            msg.goal.target_pose.header.stamp = rospy.Time.now()
            msg.goal.target_pose.header.frame_id = frame_id

            msg.goal.target_pose.pose.position.x = position_x
            msg.goal.target_pose.pose.position.y = position_y
            msg.goal.target_pose.pose.orientation.x = quaternion[0]
            msg.goal.target_pose.pose.orientation.y = quaternion[1]
            msg.goal.target_pose.pose.orientation.z = quaternion[2]
            msg.goal.target_pose.pose.orientation.w = quaternion[3]

        elif msg_type is PointStamped:
            msg.point.x = position_x
            msg.point.y = position_y
        return msg

    def publish(self, topic, pose_x, pose_y, *args):
        '''
        publish position on given topic
        :param topic: topic to publish the message on
        :param pose_x: x-position on map
        :param pose_y: y-position on map
        :param args: roll, pitch, yaw angles
        :return: --
        '''
        # generate message to publish
        msg = self.setupMessage(self.topics[topic], 'map', pose_x, pose_y, *args)
        print msg
        self.publisher[topic].publish(msg)

    def getParams(self, filepath=None):
        '''
        get parameter from ".launch"-file for /initialpose publish
        filepath: path to ".launch" file with initialpose information
        :return: position of robot in gazebo
        '''
        file, fileextension = os.path.splitext(filepath)

        # if file is a xml or launch-file
        if fileextension == '.launch' or fileextension == '.xml':
            tree = ET.parse(filepath)
            root = tree.getroot()
            for config in root.findall('arg'):
                if config.attrib['name'] in 'initial_config':
                    content = config.attrib['default']
                    # print content
                    break

            position = {}
            position['x'] = content[content.index('-x') + 3:content.index('-y') - 1]
            position['y'] = content[content.index('-y') + 3:content.index('-R') - 1]
            position['R'] = content[content.index('-R') + 3:content.index('-P') - 1]
            position['P'] = content[content.index('-P') + 3:content.index('-Y') - 1]
            position['Y'] = content[content.index('-Y') + 3:]
            # convert str to float
            for pos in position:
                position[pos] = float(position[pos])
        return position

    def main(self, filepath=None, initialpose=True, goal=True, *pos):
        '''
        publish initialpose and navigation goal
        :return: --
        '''

        if initialpose:
            position = self.getParams(filepath)
            output = 'RVIZ initialpose: [x: ' + \
                     str(position['x']) + '; y: ' + \
                     str(position['y']) + '; R: ' + \
                     str(position['R']) + '; P: ' + \
                     str(position['P']) + '; Y: ' + \
                     str(position['Y']) + ']'
            print self.tc.OKBLUE + '=' * output.__len__()
            print output
            print '=' * output.__len__() + self.tc.ENDC
            self.publish('/initialpose', position['x'], position['y'], position['R'], position['P'], position['Y'])
            print self.tc.OKBLUE + '=' * output.__len__() + self.tc.ENDC

        if goal:
            output = 'RVIZ goal: [x: ' + \
                     str(pos[0]) + '; y: ' + \
                     str(pos[1]) + '; R: ' + \
                     str(pos[2]) + '; P: ' + \
                     str(pos[3]) + '; Y: ' + \
                     str(pos[4]) + ']'
            print self.tc.OKBLUE + '=' * output.__len__()
            print output
            print '=' * output.__len__() + self.tc.ENDC
            # self.publish('/move_base_simple/goal', pos[0], pos[1], pos[2], pos[3], pos[4])
            self.publish('/move_base/goal', pos[0], pos[1], pos[2], pos[3], pos[4])
            print self.tc.OKBLUE + '=' * output.__len__() + self.tc.ENDC


if __name__ == '__main__':
    try:
        rp = RvizPublisher()
        rp.main(True, True, -0.2, 6.0, 0, 0, 0)
    except rospy.ROSInterruptException:
        pass
