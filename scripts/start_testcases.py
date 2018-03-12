#!/usr/bin/python

"""
Created on Nov 6, 2017

@author: flg-ma
@attention: Auto Testcases using ATF
@contact: albus.marcel@gmail.com (Marcel Albus)
@version: 3.2.0

#############################################################################################

History:
- v3.2.0: added start point for config files as commandline argument
- v3.1.0: updated eband_param_path
- v3.0.1: kill all 'gazebo' tasks, and purge '.ros' directory via commandline
- v3.0.0:   * added argument parser
            * added save files function
            * added 'get_eband_param_config' function
            * added automated ATF start using commandline
- v1.1.0: updated directory structure
- v1.0.0: first init
"""

import os
import shutil
import rospkg
import argparse
from distutils import dir_util
import time
import glob


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
        self.atf_yaml_generated_pth = '/home/flg-ma/git/atf_nav_test_config/scripts/yaml_files'
        self.args = self.build_parser().parse_args()
        self.timeformat = "%Y_%m_%d"
        self.pc_count = self.args.pc_count
        self.move_base_eband_param_path = self.rospack.get_path(
            'msh_navigation_config') + '/robots/cob4-2/nav/move_base_eband_params.yaml'

    def build_parser(self):
        parser = argparse.ArgumentParser(description='Start testcases using ATF')
        parser.add_argument('-c', '--pc_count', help='How many PC\'s are used for the simulation?', type=int, default=1)
        parser.add_argument('-n', '--number_of_pc', help='The number of the PC on which the simulation is run',
                            type=int, default=1)
        parser.add_argument('-s', '--start', help='At which config should the test start?', type=int, default=0)
        return parser

    def save_files(self, filepath, testcase):
        '''
        save the output files of the metrics and the parameter files in the desired filepath
        :filepath: path where the generated '.yaml' files should be saved
        :return: --
        '''
        metric_yaml_output_directory = '/tmp/atf_nav_test/'
        # copy the generated output yaml files from the metrics in the data folder
        dir_util.copy_tree(src=metric_yaml_output_directory, dst=filepath)
        # copy the 'move_base_eband_params.yaml' file in the data folder
        shutil.copy2(src=self.move_base_eband_param_path, dst=filepath)
        print '=' * 80
        print '\033[92m' + 'Copying output \'yaml\'-files of ' + testcase + '\033[0m'
        print '=' * 80

    def cpy_atf_generated_yaml_to_dst(self, yaml_name):
        # copy the 'yaml' files from the atf directory into the config folder for ATF
        shutil.copy2(src=yaml_name, dst=self.move_base_eband_param_path)

    def get_eband_param_configs(self):
        '''
        count the available 'yaml' files and return a list with all filenames
        :return: list with filenames of the 'yaml' files
        '''
        path = '/home/flg-ma/git/atf_nav_test_config/scripts/yaml_files'
        files = glob.glob(path + '/' + 'move_base_eband_params*.yaml')
        # sort alphabetically
        files.sort()
        print 'Found ', files.__len__(), 'files.'
        return files

    def main(self):
        '''
        main function of the program
        :return: --
        '''

        print '=' * 80
        print '=' * 80
        print '\033[92m' + 'Automated Test Simulation'
        print 'including the following testcases: '
        for case in self.testcases:
            print '- ', case
        print '\033[0m'
        print '=' * 80
        print '=' * 80

        starttime = time.time()

        param_config_list = self.get_eband_param_configs()
        pcl = param_config_list
        # split the complete bunch of configs into the defined amount of pieces (defined in self.args.pc_count)
        # e.g. N_allconfigs = 2048, pc_count = 2 --> n = N_allconfigs / pc_count = 1024 configs
        pcl = [pcl[i:i + (pcl.__len__() / self.args.pc_count)] for i in
               xrange(0, len(pcl), (pcl.__len__() / self.args.pc_count))]

        # output how many configs will be tested
        for i in pcl:
            print '=' * 80
            print i.__len__(), 'config files will be tested.'
            passed_tests_counter = i.__len__() * (self.args.number_of_pc - 1)
            print '=' * 80
            break

        passed_tests_counter += self.args.start

        # for yaml in pcl[self.args.number_of_pc - 1]:
        # start configs from commandline argument given start value
        for yaml in pcl[self.args.number_of_pc - 1][self.args.start:]:
            print '=' * 80
            print '\033[92m' + 'Copy the \'atf_yaml\' to the \'msh_navigation_config\'-pkg' + '\033[0m'
            self.cpy_atf_generated_yaml_to_dst(yaml_name=yaml)
            print '=' * 80

            for case in self.testcases:
                atf_pkg_path = self.rospack.get_path(case)
                # path where the data is saved
                data_save_pth = 'Data/' + time.strftime(self.timeformat) + '/' + case + '_' + str(passed_tests_counter)
                print '=' * 80
                print '\033[92m' + 'The specified testcase is:', case, '\033[0m'
                print '=' * 80

                os.chdir(atf_pkg_path + '/../../../')
                # console command to start ATF
                startstring = 'catkin_make atf_' + case
                # start ATF
                os.system(startstring)
                # kill all gazebo tasks
                os.system('kill $(ps aux | grep \'[g]azebo\' | awk \'{print $2}\')')

                # save the generated 'yaml' files from the metrics and the 'move_base_eband_params.yaml' file
                print '=' * 80
                print '\033[92m' + 'Save the generated \'yaml\' files at: ' + data_save_pth + '\033[0m'
                self.save_files(filepath=data_save_pth, testcase=case)
                print '=' * 80

                # purge 'log' files for 'catkin_make ...'
                os.chdir('/home/flg-ma/.ros')
                os.system('rosclean purge -y')
                print '\033[93m' + '=' * 80 + '\033[0m'
                print '=' * 80
            passed_tests_counter += 1
            
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
