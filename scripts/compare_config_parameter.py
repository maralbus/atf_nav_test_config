#!/usr/bin/python

"""
Created on Dec 15, 2017

@author: flg-ma
@attention: compare the output results of the ATF tests
@contact: albus.marcel@gmail.com (Marcel Albus)
@version: 1.0.0


#############################################################################################

History:
- v1.0.0: first push
"""

import os
import yaml
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


class CompareConfig:
    def __init__(self):
        print '=' * 100
        try:
            self.pth = raw_input('Please enter Path to YAML files (e.g: \'/home/flg-ma/TestYaml/\'): ')
            print 'Collecting directories in given path...'
            self.directories = os.walk(self.pth).next()[1]
        except StopIteration:  # catch error when there is no valid directory given
            exit('The directory path does not exist')

        print '=' * 100
        self.yaml_name = 'move_base_eband_params.yaml'
        self.do_params = ['max_vel_lin',
                          'max_vel_th',
                          'max_translational_acceleration',
                          'max_rotational_acceleration',
                          'scaled_radius_factor',
                          'eband_min_relative_bubble_overlap',
                          'kinematics_string']

    def collect_dataframe(self):
        '''
        collect a pandas dataframe from all the 'move_base_eband_parameter.yaml' files in a given directory
        :return: --
        '''
        df = pd.DataFrame({})  # create empty dataframe
        counter = 0
        for folder in self.directories:  # walk through all directories
            with open(self.pth + folder + '/' + self.yaml_name, 'r') as f:
                yaml_dict = yaml.load(f)  # save all the yaml data in a dictionary
            df = df.append(yaml_dict, ignore_index=True)  # save the dict in the df
            # increase counter
            counter += 1
            if counter % 20 == 0:  # output every 20 directories
                print 'Directories saved: ' + str(counter) + ' / ' + str(self.directories.__len__())
        print df.mean()  # mean values of dataframe
        print '=' * 100
        print df.std()  # standard deviation of dataframe
        # df.to_csv(self.pth + 'Config.csv')
        self.df = df.copy()  # save dataframe

    def plot_error_bar(self):
        '''
        plot an errorbar chart to show all the changed params from the config in a given directory
        :return: --
        '''
        df = self.df.copy()  # copy df
        columns = df.columns.tolist()
        for params in self.do_params:
            columns.remove(params)  # delete all unnecessary entries from the columns list
        df = df.drop(columns, axis=1)  # drop every column not changed
        print df.head(5)  # print first 'n' rows
        print '=' * 100

        means = df.mean()  # means of all values
        print '\033[94m' + 'average values' + '\033[0m'
        print means
        print '=' * 100
        print '\033[94m' + 'standard deviation' + '\033[0m'
        print df.std()

        scale_factor = 1.0
        figsize_y = 16.0 # y-size of the canvas for the plot
        fig = plt.figure(222, figsize=(7.0 * scale_factor, figsize_y * scale_factor))
        ax1 = fig.add_subplot(411)
        means.plot.bar(yerr=df.std(), ax=ax1, error_kw={'capsize': figsize_y, 'elinewidth': 2.0})
        # plt.xticks(rotation=45)
        blue_patch = mpatches.Patch(color='blue', label='value')
        plt.legend(handles=[blue_patch])
        plt.grid(True)
        plt.savefig(self.pth + 'ChangedValues.pdf', bbox_inches='tight')
        # plt.show()
        fig.clf()
        print '\033[92m' + '=' * 41 + ' Created Errorbar ' + '=' * 41 + '\033[0m'

    def main(self):
        self.collect_dataframe()
        self.plot_error_bar()


if __name__ == '__main__':
    st = CompareConfig()
    st.main()
