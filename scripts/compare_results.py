#!/usr/bin/python

"""
Created on Nov 22, 2017

@author: flg-ma
@attention: compare the output results of the ATF tests
@contact: albus.marcel@gmail.com (Marcel Albus)
@version: 2.0.0
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
        self.metrics = ['path_length',
                        'goal',
                        'jerk',
                        'time']

    def test_pivot(self):
        columns = ['testcase', 'test_number', 'bool']
        columns.extend(self.metrics)
        df = pd.DataFrame([], columns=columns)

        i = 0
        j = 0
        for n in xrange(0, 20):
            data = {'testcase': self.testcases[i],
                    'test_number': j,
                    'bool': True if np.random.random() > 0.5 else False,
                    'path_length': np.random.random(),
                    'goal': np.random.random(),
                    'jerk': np.random.random(),
                    'time': np.random.random()}
            df = df.append(data, ignore_index=True)
            i += 1
            if i >= 6:
                j += 1
                i = 0

        print df.pivot(index='testcase', columns='test_number', values='jerk')
        # print df

    def read_yaml(self):
        data_dict = {}
        columns = ['testcase', 'test_number', 'bool']
        columns.extend(self.metrics)
        df = pd.DataFrame([], columns=columns)
        for items in self.testcases:
            data_dict[items] = {}
        for folder in self.directories:
            # number of testcase
            testcase_number = int(filter(str.isdigit, folder))

            if 'narrow' in folder:
                # save testcase name from folder name
                testcase_name = folder[: -(len(str(testcase_number)))]
                testcase_number = int(str(testcase_number)[1:])
            else:
                # save testcase name from folder name
                testcase_name = folder[: -(len(str(testcase_number)) + 1)]

            filepath = self.pth + folder + '/' + self.yaml_directory + '/' + self.yaml_name
            if os.path.exists(filepath):
                stream = file(filepath, 'r')
                data_dict[testcase_name][testcase_number] = yaml.load(stream)
                # print data_dict[testcase_name][number]

                testcase_bool = data_dict[testcase_name][testcase_number]['testblock_nav']['path_length'][0][
                                    'groundtruth_result'] and \
                                data_dict[testcase_name][testcase_number]['testblock_nav']['goal'][0][
                                    'groundtruth_result'] and \
                                data_dict[testcase_name][testcase_number]['testblock_nav']['jerk'][0][
                                    'groundtruth_result'] and \
                                data_dict[testcase_name][testcase_number]['testblock_nav']['time'][0][
                                    'groundtruth_result']
                testcase_bool = float(testcase_bool)
                data = {'testcase': testcase_name,
                        'test_number': testcase_number,
                        'bool': testcase_bool,
                        'path_length': data_dict[testcase_name][testcase_number]['testblock_nav']['path_length'][0][
                            'data'],
                        'goal': data_dict[testcase_name][testcase_number]['testblock_nav']['goal'][0]['data'],
                        'jerk': data_dict[testcase_name][testcase_number]['testblock_nav']['jerk'][0]['data'],
                        'time': data_dict[testcase_name][testcase_number]['testblock_nav']['time'][0]['data']}
                df = df.append(data, ignore_index=True)
                stream.close()
            else:
                data = {'testcase': testcase_name,
                        'test_number': testcase_number,
                        'bool': 0.0,
                        'goal': None,
                        'jerk': None,
                        'time': None}
                df = df.append(data, ignore_index=True)
                data_dict[testcase_name][testcase_number] = None

        df = df.pivot(index='testcase', columns='test_number', values='bool')
        print df.head(10)

        return df

    def create_heatmap(self, dataframe):
        d = {'col1': [True, False], 'col2': [False, True]}
        df = pd.DataFrame(data=d)

        sns.set()
        uniform_data = np.random.rand(10, 12)
        # ax = sns.heatmap(df, vmin=False, vmax=True, cbar=False)

        ax = sns.heatmap(dataframe, linewidths=.2, cbar=False, cmap=mpl.colors.ListedColormap(['red', 'yellow', 'green']))
        plt.show()


    def main(self):

        self.create_heatmap(self.read_yaml())
        # self.test_pivot()
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

# test = {'testblock_nav': {'path_length': [{'groundtruth_epsilon': 0.3, 'groundtruth_result': True, 'data': 5.363, 'details': {'measured_frame': '/base_link', 'root_frame': '/map'}, 'groundtruth': 5.5}], 'goal': [{'groundtruth_epsilon': 0.3, 'groundtruth_result': True, 'data': 0.1685706102085626, 'details': {'topic': '/move_base/goal'}, 'groundtruth': 0.0}], 'jerk': [{'groundtruth_epsilon': 4.0, 'groundtruth_result': True, 'data': 1.2369029983827926, 'details': {'topic': '/base/odometry_controller/odometry'}, 'groundtruth': 0.0}], 'time': [{'groundtruth_epsilon': 1.0, 'groundtruth_result': False, 'data': 9.864, 'details': None, 'groundtruth': 14.0}]}}
