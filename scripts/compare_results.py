#!/usr/bin/python

"""
Created on Nov 22, 2017

@author: flg-ma
@attention: compare the output results of the ATF tests
@contact: albus.marcel@gmail.com (Marcel Albus)
@version: 1.0.0
"""

import yaml
import shutil
import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import os


class Goal:
    DATA = 0


class CompareResults:
    def __init__(self):
        self.pth = '/home/flg-ma/Test/'
        self.yaml_directory = 'results_yaml'
        self.yaml_name = 'ts0_c0_r0_e0_0.yaml'
        self.directories = os.walk(self.pth).next()[1]
        print self.directories

    def read_yaml(self):
        data_dict = {}
        for folder in self.directories:
            number = int(filter(str.isdigit, folder))
            print number
            filepath = self.pth + folder + '/' + self.yaml_directory + '/' + self.yaml_name
            print filepath
            stream = file(filepath, 'r')
            data_dict[number] = yaml.load(stream)
            print data_dict[452]['testblock_nav']['goal'][0]['data']
        for key, value in data_dict.iteritems():
            print key, value
            data_dict[key] = value['testblock_nav']
            print data_dict
            # return data_dict

    def create_heatmap(self, data_dict):
        sns.set()
        uniform_data = np.random.rand(10, 12)
        ax = sns.heatmap(uniform_data)
        plt.show()

    def main(self):
        self.read_yaml()


if __name__ == '__main__':
    cr = CompareResults()
    cr.main()
