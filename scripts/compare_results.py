#!/usr/bin/python

"""
Created on Nov 22, 2017

@author: flg-ma
@attention: compare the output results of the ATF tests
@contact: albus.marcel@gmail.com (Marcel Albus)
@version: 3.0.0
"""

import yaml
import shutil
import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib as mpl
import os
from matplotlib.patches import Rectangle
from matplotlib.offsetbox import AnchoredOffsetbox, TextArea, DrawingArea, HPacker


class Goal:
    DATA = 0


class CompareResults:
    def __init__(self):
        self.pth = '/home/flg-ma/Test/'  # filepath where the generated 'yaml'-directories are saved
        self.yaml_directory = 'results_yaml'  # output 'yaml'-directory
        self.yaml_name = 'ts0_c0_r0_e0_0.yaml'  # output 'yaml'-name
        self.directories = os.walk(self.pth).next()[1]
        # testcases for ATF
        self.testcases = ['line_passage',  # 0
                          'line_passage_obstacle',  # 1
                          'line_passage_person_moving',  # 2
                          'line_passage_spawn_obstacle',  # 3
                          'narrow_passage_2_cone',  # 4
                          't_passage',  # 5
                          't_passage_obstacle']  # 6
        # metrics for ATF
        self.metrics = ['path_length',
                        'goal',
                        'jerk',
                        'time']

    def test_pivot(self):
        '''
        test function for the pivot ability of a dataframe from pandas
        :return: -
        '''
        columns = ['testcase', 'test_number', 'bool']  # columns for dataframe...
        columns.extend(self.metrics)  # ... extended with the metrics
        df = pd.DataFrame([], columns=columns)  # setup dataframe

        i = 0
        j = 0
        # generate 20 dummy data-sets
        for n in xrange(0, 20):
            data = {'testcase': self.testcases[i],
                    'test_number': j,
                    # create 'True' or 'False' dummy data
                    'bool': True if np.random.random() > 0.5 else False,
                    'path_length': np.random.random(),
                    'goal': np.random.random(),
                    'jerk': np.random.random(),
                    'time': np.random.random()}
            df = df.append(data, ignore_index=True)  # append to dataframe
            i += 1  # increase testcase counter
            if i >= 6:
                j += 1  # increase testnumber if all testcases are saved once
                i = 0  # reset testcase counter

        print df.pivot(index='testcase', columns='test_number', values='jerk')
        # print df

    def read_yaml(self):
        '''
        reads the output yaml-files of the ATF and saves them as a pandas dataframe
        :return: pandas dataframe with bool values for successful tests
        '''
        data_dict = {}
        columns = ['testcase', 'test_number', 'bool']  # columns of the pandas dataframe...
        columns.extend(self.metrics)  # ... extended with the metrics
        df = pd.DataFrame([], columns=columns)  # setup dataframe with the columns above
        for items in self.testcases:
            data_dict[items] = {}  # create a dict inside a dict for all testcases
        for folder in self.directories:
            testcase_number = int(filter(str.isdigit, folder))  # number of testcase is saved
            if 'narrow' in folder:
                # save testcase name from folder name without number
                testcase_name = folder[: -(len(str(testcase_number)))]
                # narrow_passage_2_cone would save the '2' from the name as number --> not needed
                testcase_number = int(str(testcase_number)[1:])
            else:
                # save testcase name from folder name without number when it's not 'narrow_passage_2_cone'
                testcase_name = folder[: -(len(str(testcase_number)) + 1)]

            # create filepath where the 'yaml'-file is stored
            filepath = self.pth + folder + '/' + self.yaml_directory + '/' + self.yaml_name
            if os.path.exists(filepath):  # save the data from the 'yaml' if there is an output file
                stream = file(filepath, 'r')  # open filestream for yaml
                data_dict[testcase_name][testcase_number] = yaml.load(stream)  # save yaml in dict
                # print data_dict[testcase_name][number]

                # create one bool with logic && from the 'groundtruth_results'
                testcase_bool = data_dict[testcase_name][testcase_number]['testblock_nav']['path_length'][0][
                                    'groundtruth_result'] and \
                                data_dict[testcase_name][testcase_number]['testblock_nav']['goal'][0][
                                    'groundtruth_result'] and \
                                data_dict[testcase_name][testcase_number]['testblock_nav']['jerk'][0][
                                    'groundtruth_result'] and \
                                data_dict[testcase_name][testcase_number]['testblock_nav']['time'][0][
                                    'groundtruth_result']
                # save the bool as float for the heatmap colour output
                testcase_bool = float(testcase_bool)
                # create a data dict to append at the dataframe
                data = {'testcase': testcase_name,
                        'test_number': testcase_number,
                        'bool': testcase_bool,
                        'path_length': data_dict[testcase_name][testcase_number]['testblock_nav']['path_length'][0][
                            'data'],
                        'goal': data_dict[testcase_name][testcase_number]['testblock_nav']['goal'][0]['data'],
                        'jerk': data_dict[testcase_name][testcase_number]['testblock_nav']['jerk'][0]['data'],
                        'time': data_dict[testcase_name][testcase_number]['testblock_nav']['time'][0]['data']}
                df = df.append(data, ignore_index=True)  # append data to dataframe
                stream.close()  # close filestream
            else:  # if there is no generated output 'yaml'-file, save only the testcase name and number
                data = {'testcase': testcase_name,
                        'test_number': testcase_number,
                        'bool': 0.0,
                        'goal': None,  # set values to 'None'
                        'jerk': None,
                        'time': None}
                df = df.append(data, ignore_index=True)  # append data to dataframe
                data_dict[testcase_name][testcase_number] = None

        df = df.pivot(index='testcase', columns='test_number', values='bool')  # create the desired table output

        formatted_testcases = ['Line Passage',
                               'Line Passage Obstacle',
                               'Line Passage Person Moving',
                               'Line Passage Spawn Obstacle',
                               'Narrow Passage Two Cone',
                               'T Passage',
                               'T Passage Obstacle']

        i = 0
        for cases in self.testcases:  # rename the row indices to nice, formatted names
            df = df.rename(index={cases: formatted_testcases[i]})
            i += 1
        print df.head(10)  # print the firs 'n' numbers of the table
        return df  # returns the dataframe

    def create_heatmap(self, dataframe):
        '''
        creates a seaborn heatmap with the provided dataframe
        :param dataframe: pandas dataframe with the heatmap data
        :return: -
        '''
        fig = plt.figure(1, figsize=(35.0, 10.0))
        sns.set()  # setup seaborn
        # create heatmap
        ax = sns.heatmap(dataframe, linewidths=.2, cbar=False,
                         cmap=mpl.colors.ListedColormap(['red', 'yellow', 'green']), square=True)
        # bugfix for the 'bbox_inches=tight' layout, otherwise the label will be cut away
        plt.title('$\quad$', fontsize=25)
        plt.xlabel('Testcase number', fontsize=15)
        plt.ylabel('Testcase', fontsize=15)
        self.plot_rectangle(figure=fig, axis=ax)
        plt.savefig(self.pth + 'Test.pdf', bbox_inches='tight')
        plt.show()  # show heatmap

    def plot_rectangle(self, figure, axis):
        '''
        plots the legend rectangle above the left corner of the figure
        :param figure: figure on which to add the label
        :param axis: axis on which to add the label
        :return: -
        '''

        box1 = TextArea(" True: \n False:", textprops=dict(color="k", size=10))

        box2 = DrawingArea(20, 27.5, 0, 0)
        el1 = Rectangle((5, 15), width=10, height=10, angle=0, fc="g")
        el2 = Rectangle((5, 2.5), width=10, height=10, angle=0, fc="r")
        box2.add_artist(el1)
        box2.add_artist(el2)

        box = HPacker(children=[box1, box2],
                      align="center",
                      pad=0, sep=5)

        anchored_box = AnchoredOffsetbox(loc=3,
                                         child=box, pad=0.,
                                         frameon=True,
                                         bbox_to_anchor=(0., 1.02),
                                         bbox_transform=axis.transAxes,
                                         borderpad=0.,
                                         )

        axis.add_artist(anchored_box)

        figure.subplots_adjust(top=0.8)

    def main(self):

        self.create_heatmap(self.read_yaml())
        # self.create_heatmap(self.read_yaml())


if __name__ == '__main__':
    cr = CompareResults()
    cr.main()