#!/usr/bin/python

"""
Created on Dec 11, 2017

@author: flg-ma
@attention: Set the time metrics to true if better than expected
@contact: albus.marcel@gmail.com (Marcel Albus)
@version: 1.1.0


#############################################################################################

History:
- v1.1.0:   * wrapped notification txt in try-except block
            * added filepath if/else in __init__
- v1.0.0: first push
"""

true = True
false = False

import os
import yaml


class SetTimeTrue:
    def __init__(self, filepath=None):
        print '\033[94m' + '=' * 100 + '\033[0m'
        print '\033[94m' + '=' * 43 + ' Set Time True ' + '=' * 42 + '\033[0m'
        print '\033[94m' + '=' * 100 + '\033[0m'
        try:
            if filepath is None:
                self.pth = raw_input('Please enter Path to generated testcase output (e.g: \'/home/flg-ma/Test/\'): ')
            else:
                self.pth = filepath
            if os.path.exists(self.pth + 'Dataframe.csv'):
                exit('\'Datframe.csv\' found in given path, please delete file first and start again')
                pass
            else:
                print 'Collecting directories in given path...'
                self.directories = os.walk(self.pth).next()[1]
        except StopIteration:  # catch error when there is no valid directory given
            exit('The directory path does not exist')

        print '\033[92m' + '=' * 30 + ' Set Time True ' + '=' * 30 + '\033[0m'
        print '\033[92m' + '=' * 30 + ' ' + self.pth + ' ' + '=' * 30 + '\033[0m'
        print '=' * 100

        self.yaml_directory = 'results_yaml'  # output 'yaml'-directory
        self.yaml_name = 'ts0_c0_r0_e0_0.yaml'  # output 'yaml'-name

    def set_yaml_time_true(self):
        '''
        collect the yaml files in the given path directory
        return: --
        '''
        data_dict = {}
        for folder in self.directories:
            filepath = self.pth + folder + '/' + self.yaml_directory + '/' + self.yaml_name
            if os.path.exists(filepath):  # save the data from the 'yaml' if there is an output file
                stream = file(filepath, 'r')  # open filestream for yaml
                data_dict = yaml.load(stream)  # save yaml in dict
                # if data of time is below groundtruth
                if data_dict['testblock_nav']['time'][0]['data'] <= data_dict['testblock_nav']['time'][0][
                    'groundtruth']:
                    # set 'groundtruth_result' to true
                    data_dict['testblock_nav']['time'][0]['groundtruth_result'] = true
                    print '=' * 80
                    print 'Data:', data_dict['testblock_nav']['time'][0]['data'], '<\t Groundtruth:', \
                        data_dict['testblock_nav']['time'][0]['groundtruth']
                    print filepath
                    print '=' * 80
                    # save 'yaml'-file in same folder as old, and overwrite old
                    stream = file(filepath, 'w')
                    # generate 'yaml' file
                    yaml.dump(data_dict, stream, default_flow_style=False)
                    stream.close()
        # generated README file to indicate that files have been corrected
        os.chdir(self.pth)
        try:
            os.mknod('SETTIMETRUE.txt')
            stream = file(self.pth + '/SETTIMETRUE.txt', 'w')
            stream.write('Every \'YAML\' in this directory was edited with the \'set_time_true.py\'-script.')
            stream.write(
                '\nThe \'groundtruth_result\' for the \'time\'-metrics is set to \'true\' if the \'data\' is below the \'groundtruth_data\'')
            stream.close()
        except OSError as e:
            print '\033[93m' + 'SETTIMETRUE.txt Error:', e, '\033[0m'
            print '=' * 80


if __name__ == '__main__':
    stt = SetTimeTrue()
    stt.set_yaml_time_true()
