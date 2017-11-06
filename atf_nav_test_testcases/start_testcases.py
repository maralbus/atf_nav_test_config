#!/usr/bin/python

# !/usr/bin/python

"""
Created on Nov 6, 2017

@author: flg-ma
@attention: Auto Testcases using ATF
@contact: albus.marcel@gmail.com (Marcel Albus)
@version: 1.1.0
"""

import os
import shutil
import rospkg
import argparse
from distutils import dir_util


class StartTestcases:
    def __init__(self):

        self.testcases = ['line_passage',  # 0
                          'line_passage_obstacle',  # 1
                          'line_passage_person_moving',  # 2
                          'line_passage_spawn_obstacle',  # 3
                          'narrow_passage_2_cone',  # 4
                          't_passage',  # 5
                          't_passage_obstacle']  # 6
        self.rospack = rospkg.RosPack()  # get path for ROS package
        self.goal_path = self.rospack.get_path('atf_nav_test')
        self.config_path = '/home/flg-ma/git/atf_nav_test_config'
        self.args = self.build_parser().parse_args()

    def build_parser(self):
        parser = argparse.ArgumentParser(description='Start testcases using ATF')
        parser.add_argument('-l', '--launch', help='Name of the testcase, e.g. \'line_passage\'', type=str)
        return parser

    # def copytree(self, src, dst, symlinks=False, ignore=None):
    #     for item in os.listdir(src):
    #         s = os.path.join(src, item)
    #         d = os.path.join(dst, item)
    #         if os.path.isdir(s):
    #             shutil.copytree(s, d, symlinks, ignore)
    #         else:
    #             shutil.copy2(s, d)

    def main(self):
        if self.args.launch != None and self.args.launch in self.testcases:
            for tests in self.testcases:
                if tests == self.args.launch:
                    testcase = tests
                    break
            print '=' * 80
            print '\033[92m' + 'The specified testcase is:', testcase, '\033[0m'
            print '=' * 80
            print '\033[92m' + 'Copying \'application.launch\'...' + '\033[0m'
            print '=' * 80
            # copy 'application.launch' to desired destination
            shutil.copy2(self.config_path + '/atf_nav_test_testcases/' + self.args.launch + '/application.launch',
                         self.goal_path + '/launch')

            print '\033[92m' + 'Copying \'config\'...' + '\033[0m'

            # copy 'config' folder to desired destination
            dir_util.copy_tree(self.config_path + '/atf_nav_test_testcases/' + self.args.launch + '/config',
                               self.goal_path + '/config')
            print '=' * 80

            os.chdir(self.goal_path + '/../../')
            # start ATF
            os.system('catkin_make atf_atf_nav_test')


        elif (self.args.launch == None) or (self.args.launch not in self.testcases):
            print '\033[93m' + '=' * 80
            print 'Wrong user input.'
            print 'Please enter the name of a testcase from the following list: '
            for test in self.testcases:
                print '- ', test
            print '=' * 80 + '\033[0m'


if __name__ == '__main__':
    st = StartTestcases()
    st.main()
pass
