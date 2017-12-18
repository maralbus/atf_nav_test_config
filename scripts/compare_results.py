#!/usr/bin/python

"""
Created on Nov 22, 2017

@author: flg-ma
@attention: compare the output results of the ATF tests
@contact: albus.marcel@gmail.com (Marcel Albus)
@version: 4.4.0


#############################################################################################

History:
- v4.4.0: added filepath if/else in __init__
- v4.3.0:   * updated commandline output
            * create dataframe list to generate all threshold plots in one run
            * deleted 'threshold' parameter in 'drop_threshold' func because all threshold plots are generated at once
- v4.2.0: added errorbar plot
- v4.1.0:   * wrap every input in 'try'-'except' statements
            * added 'drop_treshold' func
            * added heatmap commandline output
            * figuresize now dependent on dataframe length instead of directory length (because of threshold drop func)
- v3.1.0:   * commandline output when directories are collected
            * figuresize now dependent on directory length
            * added rectangles as legend in heatmap plot
- v3.0.0:   * added comments
            * renamed dataframe columns for heatmap output
            * added heatmap plot func
- v2.0.0:   * updated 'read_yaml' func to collect bool data from testcases
            * saved yaml dict in dataframe
            * added 'pivot_test' func
- v1.0.0: first push
"""

import yaml
import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib as mpl
import os
from matplotlib.patches import Rectangle
from matplotlib.offsetbox import AnchoredOffsetbox, TextArea, DrawingArea, HPacker


class CompareResults:
    def __init__(self, filepath=None):
        # self.pth = '/home/flg-ma/Test/'  # filepath where the generated 'yaml'-directories are saved
        print '\033[94m' + '=' * 100 + '\033[0m'
        print '\033[94m' + '=' * 42 + ' Compare Results ' + '=' * 41 + '\033[0m'
        print '\033[94m' + '=' * 100 + '\033[0m'
        try:
            if filepath is None:
                self.pth = raw_input('Please enter Path to generated testcase output (e.g: \'/home/flg-ma/Test/\'): ')
            else:
                self.pth = filepath
            if os.path.exists(self.pth + 'Dataframe.csv'):
                pass
            else:
                print 'Collecting directories in given path...'
                self.directories = os.walk(self.pth).next()[1]
        except StopIteration:  # catch error when there is no valid directory given
            exit('The directory path does not exist')

        print '=' * 100

        # try:
        #     self.threshold = int(raw_input(
        #         'Please enter threshold to drop results with less successful tests than given threshold: '))  # threshold for 'self.drop_threshold' func
        # except (NameError, ValueError) as e:  # catch wrong integer input
        #     exit(e)
        # print '=' * 100

        self.yaml_directory = 'results_yaml'  # output 'yaml'-directory
        self.yaml_name = 'ts0_c0_r0_e0_0.yaml'  # output 'yaml'-name
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
        if os.path.exists(self.pth + 'Dataframe.csv'):
            df = pd.read_csv(self.pth + 'Dataframe.csv')
        else:
            data_dict = {}
            columns = ['testcase', 'test_number', 'bool']  # columns of the pandas dataframe...
            columns.extend(self.metrics)  # ... extended with the metrics
            df = pd.DataFrame([], columns=columns)  # setup dataframe with the columns above
            for items in self.testcases:
                data_dict[items] = {}  # create a dict inside a dict for all testcases
            counter = 0
            for folder in self.directories:
                try:  # if the dataname includes no number...
                    except_flag = False
                    testcase_number = int(filter(str.isdigit, folder))  # number of testcase is saved
                except ValueError as e:
                    except_flag = True
                    print 'No number in testcase found, assuming only one test was made...'
                if ('narrow' in folder) and (not except_flag) and (testcase_number != 2):
                    # save testcase name from folder name without number
                    testcase_name = folder[: -(len(str(testcase_number)))]
                    # narrow_passage_2_cone would save the '2' from the name as number --> not needed
                    testcase_number = int(str(testcase_number)[1:])
                elif not except_flag and not ('narrow' in folder):
                    # save testcase name from folder name without number when it's not 'narrow_passage_2_cone'
                    testcase_name = folder[: -(len(str(testcase_number)) + 1)]
                else:
                    testcase_number = 0  # ... zero is saved as number
                    testcase_name = folder

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
                            'path_length_err': abs(
                                data_dict[testcase_name][testcase_number]['testblock_nav']['path_length'][0][
                                    'groundtruth'] -
                                data_dict[testcase_name][testcase_number]['testblock_nav']['path_length'][0]['data']),
                            'path_length_gte':
                                data_dict[testcase_name][testcase_number]['testblock_nav']['path_length'][0][
                                    'groundtruth_epsilon'],
                            'goal': data_dict[testcase_name][testcase_number]['testblock_nav']['goal'][0]['data'],
                            'goal_err': abs(
                                data_dict[testcase_name][testcase_number]['testblock_nav']['goal'][0]['groundtruth'] -
                                data_dict[testcase_name][testcase_number]['testblock_nav']['goal'][0]['data']),
                            'goal_gte': data_dict[testcase_name][testcase_number]['testblock_nav']['goal'][0][
                                'groundtruth_epsilon'],
                            'jerk': data_dict[testcase_name][testcase_number]['testblock_nav']['jerk'][0]['data'],
                            'jerk_err': abs(
                                data_dict[testcase_name][testcase_number]['testblock_nav']['jerk'][0]['groundtruth'] -
                                data_dict[testcase_name][testcase_number]['testblock_nav']['jerk'][0]['data']),
                            'jerk_gte': data_dict[testcase_name][testcase_number]['testblock_nav']['jerk'][0][
                                'groundtruth_epsilon'],
                            'time': data_dict[testcase_name][testcase_number]['testblock_nav']['time'][0]['data'],
                            'time_err': abs(
                                data_dict[testcase_name][testcase_number]['testblock_nav']['time'][0]['groundtruth'] -
                                data_dict[testcase_name][testcase_number]['testblock_nav']['time'][0]['data']),
                            'time_gte': data_dict[testcase_name][testcase_number]['testblock_nav']['time'][0][
                                'groundtruth_epsilon']}
                    df = df.append(data, ignore_index=True)  # append data to dataframe
                    stream.close()  # close filestream
                else:  # if there is no generated output 'yaml'-file, save only the testcase name and number
                    data = {'testcase': testcase_name,
                            'test_number': testcase_number,
                            'bool': 0.0,
                            'path_length': np.nan,  # set values to 'np.nan'
                            'path_length_err': np.nan,
                            'path_length_gte': np.nan,
                            'goal': np.nan,
                            'goal_err': np.nan,
                            'goal_gte': np.nan,
                            'jerk': np.nan,
                            'jerk_err': np.nan,
                            'jerk_gte': np.nan,
                            'time': np.nan,
                            'time_err': np.nan,
                            'time_gte': np.nan}
                    df = df.append(data, ignore_index=True)  # append data to dataframe
                    data_dict[testcase_name][testcase_number] = None
                # increase counter
                counter += 1
                #
                if counter % 20 == 0:
                    print 'Directories saved: ' + str(counter) + ' / ' + str(self.directories.__len__())

        self.dataframe = df.copy()  # save dataframe globally
        self.dataframe.to_csv(self.pth + 'Dataframe.csv', index=False)
        print 'save \'dataframe\' as \'csv\''
        print '=' * 100
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
        print '=' * 100
        return df  # returns the dataframe

    def plot_heatmap(self, dataframe_list):
        '''
        creates a seaborn heatmap with the provided dataframe
        :param dataframe_list: a list with pandas dataframe including the heatmap data
        :return: -
        '''
        # fig = plt.figure(1, figsize=(self.directories.__len__() / 20.0, self.directories.__len__() / 90.0)) # stupid
        # fig = plt.figure(1, figsize=(dataframe.columns.__len__() / 3.0, dataframe.columns.__len__() / 7.0)) # more line output

        for n in xrange(1, dataframe_list.__len__()):
            x_width = (dataframe_list[n].columns.__len__() / 3.0) if (dataframe_list[
                                                                          n].columns.__len__() / 3.0) > 7.0 else 7.0
            y_height = (dataframe_list[n].columns.__len__() / 7.0) if (dataframe_list[
                                                                           n].columns.__len__() / 7.0) > 3.0 else 3.0
            fig = plt.figure(n, figsize=(x_width, y_height))
            # fig = plt.figure(1, figsize=(7.0, 3.0)) # one line output
            # fig = plt.figure(1, figsize=(200.0, 50.0))

            # setup seaborn for grey output as np.NaN values and no lines indicating the row and columns
            sns.set()  # setup seaborn

            # create heatmap
            ax = sns.heatmap(dataframe_list[n], linewidths=.3, cbar=False,
                             cmap=mpl.colors.ListedColormap(['red', 'yellow', 'green']), square=True, annot=False,
                             vmax=1.0,
                             vmin=0.0)
            plt.xticks(rotation=90)
            # bugfix for the 'bbox_inches=tight' layout, otherwise the label will be cut away
            plt.title('$\quad$', fontsize=60)
            # plt.title('$\quad$', fontsize=35)
            plt.xlabel('Testcase number', fontsize=15)
            plt.ylabel('Testcase', fontsize=15)
            self.plot_rectangle(figure=fig, axis=ax)
            plt.savefig(self.pth + 'Heatmap_Threshold_' + str(n) + '.pdf', bbox_inches='tight')
            fig.clf()
            sns.reset_orig()
            # plt.show()  # show heatmap

    def drop_threshold(self, dataframe_bool):
        '''
        drops the columns when the number of 'True'-bool parameters (given as float) are below the threshold
        the dataframe contains only the bool values (stored as float)
        :param dataframe_bool: dataframe to drop column
        :return: dataframe list with all the dataframes including less columns
        '''
        dataframe_list = []
        for threshold in xrange(0, 8):
            df = pd.DataFrame.copy(dataframe_bool)
            counter = 0
            # bool values in dataframe are stored as float
            for column in df:  # go through each column
                for value in df[column]:  # get values in each column
                    counter += value  # add all values
                if counter < threshold:  # compare with threshold
                    df = df.drop([column], axis=1)  # drop if below threshold
                counter = 0  # reset counter
            dataframe_list.append(df)
        return dataframe_list

    def plot_rectangle(self, figure, axis):
        '''
        plots the legend rectangle above the left corner of the figure
        :param figure: figure on which to add the label
        :param axis: axis on which to add the label
        :return: -
        '''

        box1 = TextArea(" True: \n False: \n NaN: ", textprops=dict(color="k", size=10))

        # box2 = DrawingArea(20, 27.5, 0, 0)
        # el1 = Rectangle((5, 15), width=10, height=10, angle=0, fc="g")
        # el2 = Rectangle((5, 2.5), width=10, height=10, angle=0, fc="r")
        box2 = DrawingArea(20, 45, 0, 0)
        el1 = Rectangle((5, 30), width=10, height=10, angle=0, fc="g")
        el2 = Rectangle((5, 18.5), width=10, height=10, angle=0, fc="r")
        el3 = Rectangle((5, 7), width=10, height=10, angle=0, fc='#d3d3d3')
        box2.add_artist(el1)
        box2.add_artist(el2)
        box2.add_artist(el3)

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

    def plot_error_bar(self):

        df = self.dataframe.copy()
        df = df.drop(['test_number',
                      'bool',
                      'path_length_err',
                      'path_length_gte',
                      'goal_err',
                      'goal_gte',
                      'jerk_err',
                      'jerk_gte',
                      'time_err',
                      'time_gte'], axis=1)
        # df = df.rename(columns={'path_length': 'path length'})
        print df.head(5)  # print first 'n' rows
        print '=' * 100
        gp = df.groupby(['testcase'])

        # means = gp.mean().rename(index={'line_passage': 'Line Passage',
        #                                 'line_passage_obstacle': 'Line Passage Obstacle',
        #                                 'line_passage_person_moving': 'Line Passage Person Moving',
        #                                 'line_passage_spawn_obstacle': 'Line Passage Spawn Obstacle',
        #                                 'narrow_passage_2_cone': 'Narrow Passage Two Cone',
        #                                 't_passage': 'T Passage',
        #                                 't_passage_obstacle': 'T Passage Obstacle'})
        means = gp.mean()  # first mean, then rename, otherwise no errorbars are shown
        print '\033[94m' + 'average values' + '\033[0m'
        print means
        print '=' * 100
        print '\033[94m' + 'standard deviation' + '\033[0m'
        print gp.std()

        scale_factor = 0.9
        fig = plt.figure(222, figsize=(13.8 * scale_factor, 8.6 * 2.2))
        ax1 = fig.add_subplot(411)
        means.plot.bar(yerr=gp.std(), ax=ax1, error_kw={'elinewidth': 2})
        # plt.xticks(rotation=45)
        plt.legend()
        plt.grid(True)
        plt.savefig(self.pth + 'Errorbar.pdf', bbox_inches='tight')
        fig.clf()
        # plt.show()

    def main(self):
        self.plot_heatmap(self.drop_threshold(dataframe_bool=self.read_yaml()))
        self.plot_error_bar()
        print '\033[92m' + '=' * 41 + ' Created Heatmap ' + '=' * 41 + '\033[0m'


if __name__ == '__main__':
    cr = CompareResults()
    cr.main()
