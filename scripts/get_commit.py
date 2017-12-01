#!/usr/bin/python

"""
Created on Nov 16, 2017

@author: flg-ma
@attention: get commit string of directory
@contact: albus.marcel@gmail.com (Marcel Albus)
@version: 1.1.0
"""

import os
import subprocess


class GetCommit:
    def __init__(self):
        self.dir_path = '/home/flg-ma/git/catkin_ws/src'
        self.directories = os.walk(self.dir_path).next()[1]
        self.commits = {'ackermann_msgs': '0b8512d0bb5a4c7bc185f2be80bc78b85febf717',
                        'cob_android': 'b801d07d4b169739abec74d1a83425993f9b7cf3',
                        'cob_calibration_data': '8912e878decc0e5a0153f7e0ae6287be37b79499',
                        'cob_command_tools': 'c593edd0c901d3c0769748df6c14586d8c5f6b88',
                        'cob_common': 'd0a90031b85a75c78548b841d9fb6882a2317592',
                        'cob_control': '4b5950aaf23e94135d6fa1fe1917c99320a2e583',
                        'cob_docking': 'eded09a293c4518c112ba80bf4c3b3455a418fca',
                        'cob_driver': 'c299763f6c1c0d24dafc339123fa24e97d2b8b3d',
                        'cob_environments': '30331137806d35a456014f63266a1bd73a1608d5',
                        'cob_extern': '53ae4ddd31c9b1b845174a9721b80eeccbe996cd',
                        'cob_gazebo_plugins': 'b87bfa35ee3b1ede9fbda40ddf08fdbe714a97d7',
                        'cob_hand': 'ffc69f33bc182f22f4c80865c9ae138848602bf0',
                        'cob_karto': '9312a7d0096e2375d246f88cc8e0e007a46874c9',
                        'cob_manipulation': 'ca9c390ff8e4440150f41de1bc42dbb67f5d2607',
                        'cob_navigation': '73c7671e4207d4dd48c2f577f83a8cb402ade0d7',
                        'cob_people_perception': 'bdb24d2982d9a145105d153bdbc4c6a9cba93f0a',
                        'cob_perception_common': 'f1159e562a383369243c6f479306f5eea18b4766',
                        'cob_robots': 'a5aa1387ba56e117e85ad07d177bf6399da3a8d2',
                        'cob_simulation': 'aa31e8c360bb974dd7727c08a3c6c2ccfb856a26',
                        'cob_supported_robots': 'eba093bc9af4d9a3b50a2ccd6f5c67f78c791341',
                        'executive_smach': '16a8f5206481bd10825b621260415ee411f03188',
                        'ipa_license_verifier': '71cf565c10c955fa6ccfd115b07c69c1b6205304',
                        'ipa_maven': '8a8019828398cde0e3554660952d184c079a979b',
                        'ipa_navigation': 'a769b14afc832f7bd548f946d0efa1a6606f4abf',
                        'ipa_navigation_common': 'a8db5bda44538df56ce16dd572032b6c0492d7c0',
                        'ipa_navigation_driver': '8d2dd079ea3c77eaf8dc60fe235282b1d850267c',
                        'ipa_navigation_localization': '9234db9890c242f0c45c88f52e1810502bb1bf1a',
                        'ipa_navigation_perception': 'dbe9f13661deb59dd8403032a51e689414f6712f',
                        'ipa_navigation_planning': '8dfe92a7223c6b7ceed6e0df154ec3b8aa43799d',
                        'ipa_navigation_sandbox': '2d6578e1f5d22ac60e07d0b8afd74c6f2907d1f0',
                        'msh': '73faf52c209aa61ff194c7bba6b551546eb38dc7',
                        'msh_common': '1f997d5c44aa6a9ed7884391ade6689012a8d9ee',
                        'msh_gui': '0850f28e93d89448dafc2075e35d3243ace4673c',
                        'msh_voice_control': '2ee81d88f48809713e49957f65351668f9a25fd5',
                        'open_karto': '44b26863e25e4c134af08e8733c7a7a7f898f997',
                        'pointcloud_to_laserscan': 'bcff028edf46554d8f3a079eb3deaa34e37715c4',
                        'ros_canopen': '3aa3ebbf4c2cc843c78813171854258692fe2606',
                        'schunk_modular_robotics': 'f636e7c01a36069b32959f737a8d6f608bb90bbd',
                        'sick_flexisoft': 'eafb5f9194cb9b96009a7d5e117128ffde9d5c3e',
                        'sick_visionary_t': '23032464427f470d36c30a676c1d8e9dfabfc091',
                        'slack-ros-pkg': '1b388e1ce31b1dc32ad67350430b25b44a15502d',
                        'sparse_bundle_adjustment': '713e3e9163c52fea35e6756513f390aa46c2be4e',
                        'universal_robot': '68d76b249e062ba8477b9b2e51f6a716a6017b02',
                        'zbar_ros': '01eb91d030e3bc29692f4a1e11c172de2bb2faf8'}

    def get_commit(self):
        print '=' * 80
        print self.directories
        print '=' * 80

        commits = {}
        for pth in self.directories:
            print pth
            os.chdir(self.dir_path + '/' + pth)
            # get all commit numbers from the catkin pkgs
            p = subprocess.Popen(["git log | grep 'commit' | awk '{print $2}' | head -n 1"], stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, stdin=subprocess.PIPE, shell=True)
            final_output = p.communicate()[0]
            commits[pth] = final_output[:-1]
            print final_output
            # p1 = subprocess.Popen("git log", stdout=subprocess.PIPE, shell=True)
            # p2 = subprocess.Popen("grep 'commit'", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True)
            # p3 = subprocess.Popen("awk '{print $2}'", stdin=p2.stdout, stdout=subprocess.PIPE, shell=True)
            # p4 = subprocess.Popen("head -n 1", stdin=p3.stdout, shell=True)
            # final_output = p4.communicate()[0]
        self.compare(commits)

    def set_commit(self):
        for pth in self.directories:
            print pth
            os.chdir(self.dir_path + '/' + pth)
            startstring = 'git checkout ' + self.commits[pth]
            # os.system(startstring)
            print startstring

    def compare(self, dict):
        if self.commits == dict:
            print '=' * 80
            print 'True'
            print '=' * 80
        else:
            print '=' * 80
            print 'False'
            print '=' * 80


if __name__ == '__main__':
    gc = GetCommit()
    gc.get_commit()
