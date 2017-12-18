#!/usr/bin/python

"""
Created on Dec 18, 2017

@author: flg-ma
@attention: compare the output results of the ATF tests
@contact: albus.marcel@gmail.com (Marcel Albus)
@version: 1.0.0


#############################################################################################

History:
- v1.0.0: first push
"""

from compare_results import CompareResults
from compare_config_parameter import CompareConfig
from set_time_true import SetTimeTrue
import os

class Evaluate:
    def __init__(self):
        print '=' * 100
        try:
            self.pth = raw_input('Please enter Path to generated testcase output (e.g: \'/home/flg-ma/Test/\'): ')
            print 'Starting evaluation...'
        except StopIteration:  # catch error when there is no valid directory given
            exit('The directory path does not exist')
        print '=' * 100



    def main(self):
        stt = SetTimeTrue(filepath=self.pth)
        stt.set_yaml_time_true()

        cr = CompareResults(filepath=self.pth)
        cr.main()

        cc = CompareConfig(filepath=self.pth)
        cc.main()
        print '\033[92m' + '=' * 100 + '\033[0m'
        print '\033[92m' + '=' * 39 + ' Finished evaluating ' + '=' * 40 + '\033[0m'
        print '\033[92m' + '=' * 100 + '\033[0m'


if __name__ == '__main__':
    e = Evaluate()
    e.main()