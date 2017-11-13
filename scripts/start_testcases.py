#!/usr/bin/python

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
import time
from distutils import dir_util
import time


class StartTestcases:
    def __init__(self):
        self.testcases = ['line_passage',  # 0
                          'line_passage_obstacle',  # 1
                          'line_passage_person_moving',  # 2
                          'line_passage_spawn_obstacle',  # 3
                          'narrow_passage_2_cone',  # 4
                          't_passage',  # 5
                          't_passage_obstacle']  # 6
        # self.testcases = ['line_passage']
        self.rospack = rospkg.RosPack()  # get path for ROS package
        self.config_path = '/home/flg-ma/git/atf_nav_test_config'
        self.args = self.build_parser().parse_args()
        self.timeformat = "%Y_%m_%d"

    def build_parser(self):
        parser = argparse.ArgumentParser(description='Start testcases using ATF')
        parser.add_argument('-l', '--launch', help='Name of the testcase, e.g. \'line_passage\'', type=str)
        return parser

    def save_files(self, filepath, testcase):
        '''
        save the output files in the desired filepath
        :filepath: path where the generated '.yaml' files should be saved
        :return:
        '''
        src_path = '/tmp/atf_nav_test/'
        eband_param_path = self.rospack.get_path('ipa_navigation_config') + '/robots/default/nav/move_base_eband_params.yaml'
        shutil.copy2(src=eband_param_path, dst=filepath)
        dir_util.copy_tree(src=src_path, dst=filepath)
        print '=' * 80
        print '\033[92m' + 'Copying output \'yaml\'-files of ' + testcase + '\033[0m'
        print '=' * 80

    def main(self):
        '''
        main function of the programm
        :return: --
        '''
        starttime = time.time()
        for case in self.testcases:
            atf_pkg_path = self.rospack.get_path(case)
            # path where the data is saved
            dirpath = 'Data/' + time.strftime(self.timeformat) + '/' + case
            print '=' * 80
            print '\033[92m' + 'The specified testcase is:', case, '\033[0m'
            print '=' * 80

            os.chdir(atf_pkg_path + '/../../../')
            # start ATF
            startstring = 'catkin_make atf_' + case
            os.system(startstring)
            self.save_files(dirpath, case)
            print '\033[93m' + '=' * 80 + '\033[0m'
            print '=' * 80
        endtime = time.time()
        print '\033[93m' + 'Time needed: ', (endtime - starttime) / 60, '[min]' + '\033[0m'
        print '=' * 80

        # elif (self.args.launch == None) or (self.args.launch not in self.testcases):
        #     print '\033[93m' + '=' * 80
        #     print 'Wrong user input.'
        #     print 'Please enter the name of a testcase from the following list: '
        #     for test in self.testcases:
        #         print '- ', test
        #     print '=' * 80 + '\033[0m'


if __name__ == '__main__':
    st = StartTestcases()
    st.main()
pass
