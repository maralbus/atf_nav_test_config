#!/usr/bin/python
import unittest
import rospy
import rostest
import sys

import rospkg
import threading
from rviz_publisher import RvizPublisher
from gazebo_msgs.msg import ModelState
import tf

from atf_core import ATF
from simple_script_server import *

sss = simple_script_server()


class Application:
    def __init__(self):
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)  # gazebo move object publisher
        self.name = 'box1'  # name of object to spawn

        self.testcases = ['line_passage',  # 0
                          'line_passage_obstacle',  # 1
                          'line_passage_person_moving',  # 2
                          'line_passage_spawn_obstacle',  # 3
                          'narrow_passage_2_cone',  # 4
                          't_passage',  # 5
                          't_passage_obstacle']  # 6
        self.rospack = rospkg.RosPack()  # get path for ROS package
        rp = RvizPublisher()
        # filepath = self.rospack.get_path('msh_bringup') + '/launch/' + self.args.launch + '.launch'
        if "standalone" == sys.argv[1]:
            filepath = self.rospack.get_path('msh_bringup') + '/launch/' + sys.argv[2] + '.launch'
        else:
            filepath = self.rospack.get_path('msh_bringup') + '/launch/' + self.testcases[3] + '.launch'
        print '=' * len(filepath)
        print filepath
        print '=' * len(filepath)
        rp.main(filepath, True, False)  # 'initialpose' publish
        del rp
        rospy.sleep(1)

        # setup threading daemon to beam object in front of robot
        self.beam_daemon = threading.Timer(15, self.beam_object, [5.0, -0.5, 0.0, 0.0, 0.0])
        self.beam_daemon.setDaemon(True)
        self.beam_daemon.setName('beam_daemon')

        self.atf = ATF()

    def execute(self):
        self.atf.start('testblock_nav')
        self.beam_daemon.start()
        # necessary to catch goal published on topic /move_base/goal
        rospy.sleep(3)
        # line passage spawn obstacle goal
        sss.move("base", [7.0, 0.0, 0.0])
        self.atf.stop('testblock_nav')
        self.atf.shutdown()

    def beam_object(self, x, y, roll, pitch, yaw):
        '''
        beams an available object to the desired position
        :param x: x-Position in [m]
        :param y: y-Position in [m]
        :param roll: Euler angle transformation
        :param pitch: Euler angle transformation
        :param yaw: Euler angle transformation
        :return: --
        '''
        # gazebo takes ModelState as msg type
        model_state = ModelState()
        model_state.model_name = self.name
        model_state.reference_frame = 'world'

        model_state.pose.position.x = x
        model_state.pose.position.y = y
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        model_state.pose.orientation.x = quaternion[0]
        model_state.pose.orientation.y = quaternion[1]
        model_state.pose.orientation.z = quaternion[2]
        model_state.pose.orientation.w = quaternion[3]
        rospy.loginfo('\033[92m' + '----Move Object Gazebo----' + '\033[0m')
        # publish message
        self.pub.publish(model_state)


class Test(unittest.TestCase):
    def setUp(self):
        self.app = Application()

    def tearDown(self):
        pass

    def test_Recording(self):
        self.app.execute()


if __name__ == '__main__':
    rospy.init_node('application_node')
    # if "standalone" in sys.argv:
    if "standalone" == sys.argv[1]:
        app = Application()
        app.execute()
    else:
        rostest.rosrun('application', 'recording', Test, sysargs=None)
