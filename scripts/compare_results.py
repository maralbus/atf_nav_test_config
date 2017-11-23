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
import matplotlib as mpl
import os


class Goal:
    DATA = 0


class CompareResults:
    def __init__(self):
        self.pth = '/home/flg-ma/Test/'
        self.yaml_directory = 'results_yaml'
        self.yaml_name = 'ts0_c0_r0_e0_0.yaml'
        self.directories = os.walk(self.pth).next()[1]
        self.testcases = ['line_passage',  # 0
                          'line_passage_obstacle',  # 1
                          'line_passage_person_moving',  # 2
                          'line_passage_spawn_obstacle',  # 3
                          'narrow_passage_2_cone',  # 4
                          't_passage',  # 5
                          't_passage_obstacle']  # 6

    def read_yaml(self):
        data_dict = {}
        for items in self.testcases:
            data_dict[items] = {}
        for folder in self.directories:
            # number of testcase
            number = int(filter(str.isdigit, folder))
            # print 'Number:', number
            if 'narrow' in folder:
                # save testcase name from folder name
                testcase_name = folder[: -(len(str(number)))]
                number = int(str(number)[1:])
            else:
                # save testcase name from folder name
                testcase_name = folder[: -(len(str(number)) + 1)]
            # print testcase_name
            filepath = self.pth + folder + '/' + self.yaml_directory + '/' + self.yaml_name
            # print 'Filepath:', filepath
            if os.path.exists(filepath):
                # load_dict = {}
                stream = file(filepath, 'r')
                data_dict[testcase_name][number] = yaml.load(stream)
                stream.close()
            else:
                data_dict[testcase_name][number] = None

        for key, values in data_dict['line_passage'].iteritems():
            print key, values
        # for key, values in  data_dict[35].iteritems():
        #     print key, values

            # return data_dict

    def create_heatmap(self, data_dict):
        d = {'col1': [True, False], 'col2': [False, True]}
        df = pd.DataFrame(data=d)

        sns.set()
        uniform_data = np.random.rand(10, 12)
        ax = sns.heatmap(df, vmin=False, vmax=True, cbar=False)
        plt.show()

    def main(self):
        self.read_yaml()
        # self.create_heatmap(self.read_yaml())


if __name__ == '__main__':
    cr = CompareResults()
    cr.main()

    # # random data
    # x = np.random.random_integers(0, 1, (10, 10))
    #
    # fig, ax = plt.subplots()
    #
    # # define the colors
    # cmap = mpl.colors.ListedColormap(['r', 'k'])
    # # create a normalize object the describes the limits of each color
    # bounds = [0., 0.5, 1.]
    # norm = mpl.colors.BoundaryNorm(bounds, cmap.N)
    #
    # # plot it
    # ax.imshow(x, interpolation='none', cmap=cmap, norm=norm)
    # plt.show()
