#!/usr/bin/python

"""
Created on Nov 7, 2017

@author: flg-ma
@attention: Generate the 'move_base_eband_params.yaml' automatically
@contact: albus.marcel@gmail.com (Marcel Albus)
@version: 2.3.1


#############################################################################################

History:
- v2.3.1: updated 'step' values for new configurations
- v2.3.0: 'drive_residual_band' necessary to perform replanning --> must be true! --> set on dead_params list
- v2.2.0: Bugfix with 'use_local_replanning' --> use_local_replanning needs to be true!
- v2.1.0:
- v2.0.0:
- v1.0.0: first push
"""

import yaml
import shutil
import rospkg
import itertools

true = True
false = False


class EbandParams(object):
    def __init__(self):
        # self.do_params = {
        #     # Maximum linear velocity
        #     'max_vel_lin': 1.0,
        #     # Maximum rotational velocity
        #     'max_vel_th': 0.5,
        #     # Damp maximal translational acceleration
        #     'max_translational_acceleration': 0.4,
        #     # Damp maximal rotational acceleration
        #     'max_rotational_acceleration': 0.2,
        #     # Scaling factor for veocity calculation
        #     'scaled_radius_factor': 4.0,
        #     # Controller settings
        #     'k_prop': 4.0,
        #     'k_damp': 3.5,
        #     # Activate smoothing option at start and end
        #     'smoothing_enabled': true,
        #     # Minimum relative overlap two bubbles must have to be treated as connected
        #     'eband_min_relative_bubble_overlap': 0.7,
        #     'drive_residual_band': true,
        #     # -----------------------------------------------------------------
        #     # select kinematics: {omnidirectional, differential}
        #     # -----------------------------------------------------------------
        #     'kinematics_string': 'omnidirectional'
        # }

        # old version 07.12.2017
        #self.do_params = {
        #    # Maximum linear velocity
        #    'max_vel_lin': {'value': [0.0, 1.0, 2.0], 'step': [0.75, 1.25]},
        #    # Maximum rotational velocity
        #    'max_vel_th': {'value': [0.0, 0.5, 2.0], 'step': [0.2, 1.5]},
        #    # Damp maximal translational acceleration
        #    'max_translational_acceleration': {'value': [0.0, 0.4, 1.5], 'step': [0.3, 1.0]},
        #    # Damp maximal rotational acceleration
        #    'max_rotational_acceleration': {'value': [0.0, 0.2, 1.0], 'step': [0.4, 0.8]},
        #    # Scaling factor for veocity calculation
        #    'scaled_radius_factor': {'value': [0.0, 4.0, 7.0], 'step': [2.0, 6.0]},
        #    # Controller settings
        #    'k_prop': {'value': [0.0, 4.0, 10.0], 'step': [2.0, 7.0]},
        #    'k_damp': {'value': [0.0, 3.5, 10.0], 'step': [2.0, 7.0]},
        #    # Activate smoothing option at start and end
        #    'smoothing_enabled': {'value': [true, true, false], 'step': [true, false]},
        #    # Minimum relative overlap two bubbles must have to be treated as connected
        #    'eband_min_relative_bubble_overlap': {'value': [0.5, 0.7, 1.0], 'step': [0.5, 0.9]},
        #    'drive_residual_band': {'value': [true, true, false], 'step': [true, false]},
        #    # -----------------------------------------------------------------
        #    # select kinematics: {omnidirectional, differential}
        #    # -----------------------------------------------------------------
        #    'kinematics_string': {'value': ['omnidirectional', 'omnidirectional', 'differential'],
        #                          'step': ['omnidirectional', 'differential']}
        #}

        # new version 07.12.2017
        self.do_params = {
            # Maximum linear velocity
            'max_vel_lin': {'value': [0.0, 1.0, 2.0], 'step': [0.75, 1.0, 1.10, 1.25]},
            # Maximum rotational velocity
            'max_vel_th': {'value': [0.0, 0.5, 2.0], 'step': [0.5, 1.0, 1.5]},
            # Damp maximal translational acceleration
            'max_translational_acceleration': {'value': [0.0, 0.4, 1.5], 'step': [0.4, 0.75, 1.0]},
            # Damp maximal rotational acceleration
            'max_rotational_acceleration': {'value': [0.0, 0.2, 1.0], 'step': [0.2, 0.5, 0.8]},
            # Scaling factor for veocity calculation
            'scaled_radius_factor': {'value': [0.0, 4.0, 7.0], 'step': [4.0, 6.0]},
            # Minimum relative overlap two bubbles must have to be treated as connected
            'eband_min_relative_bubble_overlap': {'value': [0.5, 0.7, 1.0], 'step': [0.5, 0.7, 0.8, 0.9]},
            # -----------------------------------------------------------------
            # select kinematics: {omnidirectional, differential}
            # -----------------------------------------------------------------
            'kinematics_string': {'value': ['omnidirectional', 'omnidirectional', 'differential'],
                                  'step': ['omnidirectional', 'differential']}
        }

        self.maybe_params = {
            # gain for internal forces (Elasticity of Band)
            'eband_internal_force_gain': 1.0,
            # gain for external forces (Penalty on low distance to obstacles)
            'eband_external_force_gain': 2.0,
            # gain for external forces when the robot is allmost in collision
            'eband_external_force_near_obtacle_gain': 2.0,
            'min_dist_to_last_feasible_bubble': 1.0
        }

        self.dead_params = {
            'carlike': False,
            'trjOrientation': False,
            'eband_equilibrium_approx_max_recursion_depth': 4,
            'reach_for_soft_goal': True,
            'orientation_in_driving_direction': False,
            'use_local_replanning': True,
            'eband_equilibrium_relative_overshoot': 0.75,
            'eband_significant_force_lower_bound': 0.15,
            'force_tangential_orientation': True,
            'marker_lifetime': 0.5,
            'costmap_parameter_source': '/local_costmap_node/costmap',
            'catch_moving_goal': False,
            'soft_goal_wait_threshold': 30,
            'allow_global_replanning': False,
            'trjFollowingMeasure': 0.75,
            'trjFlag': False,
            'influence_radius': 1.5,
            'eband_tiny_bubble_expansion': 0.01,
            'rotational_goal_tolerance': 0.15,
            'soft_translational_goal_tolerance': 3.0,
            'wait_for_recovery': True,
            'translational_goal_tolerance': 0.1,
            'num_iterations_eband_optimization': 3,
            'stop_threshold': 0.18,
            'number_of_bubbles': 1,
            'remove_tolerance': 0.8,
            'fill_tolerance': 0.3,
            'trjTollerance': 0.01,
            # new 15.12.17
            'drive_residual_band': True,
        }

        self.all_params = {
            'PID/d': 0.0,
            'PID/i': 0.0,
            'PID/p': 1.0,
            'allow_global_replanning': true,
            'allow_rotation_in_place': false,
            'catch_moving_goal': false,
            'center_ax_dist': 0.228,
            'costmap_parameter_source': '/local_costmap_node/costmap',
            'ctrl_rate': 10.0,
            'differential_linear_deceleration': 0.5,
            'drive_residual_band': true,
            'eband_equilibrium_approx_max_recursion_depth': 4,
            'eband_equilibrium_relative_overshoot': 0.75,
            'eband_external_force_gain': 2.0,
            'eband_external_force_near_obtacle_gain': 2.0,
            'eband_internal_force_gain': 1.0,
            'eband_min_relative_bubble_overlap': 0.7,
            'eband_significant_force_lower_bound': 0.15,
            'eband_tiny_bubble_distance': 0.01,
            'eband_tiny_bubble_expansion': 0.01,
            'eband_turning_circle_force_gain': 1.5,
            'fill_tolerance': 0.3,
            'force_rotate_in_place_angular_thresh': 0.5,
            'force_rotate_in_place_expansion_thresh': 0.2,
            'force_tangential_orientation': true,
            'in_place_trans_vel': 0.0,
            'influence_radius': 1.5,
            'k_damp': 3.5,
            'k_prop': 4.0,
            'kinematics_string': 'omnidirectional',
            'marker_lifetime': 0.5,
            'max_rotational_acceleration': 0.2,
            'max_steering_angle': 0.7,
            'max_translational_acceleration': 0.4,
            'max_vel_lin': 1.0,
            'max_vel_th': 0.5,
            'min_dist_to_last_feasible_bubble': 1.0,
            'min_in_place_vel_th': 0.0,
            'min_vel_lin': 0.1,
            'min_vel_th': 0.0,
            'moving_goal_dist_lower_thresh': 0.01,
            'moving_goal_dist_upper_thresh': 0.2,
            'moving_goal_vel_increase_factor': 1.2,
            'num_iterations_eband_optimization': 3,
            'number_of_bubbles': 1,
            'orientation_in_driving_direction': true,
            'reach_for_soft_goal': false,
            'remove_tolerance': 0.8,
            'rotation_correction_threshold': 0.5,
            'rotational_goal_tolerance': 0.04,
            'scaled_radius_factor': 4.0,
            'smoothing_enabled': true,
            'soft_goal_wait_threshold': 3,
            'soft_translational_goal_tolerance': 3.0,
            'start_smoothing_border': 10,
            'stop_smoothing_dist_to_goal': 0.1,
            'stop_threshold': 0.18,
            'translational_goal_tolerance': 0.1,
            'trjFlag': false,
            'trjFollowingMeasure': 0.75,
            'trjOrientation': false,
            'trjTollerance': 0.01,
            'use_local_replanning': true,
            'virtual_mass': 0.75,
            'wait_for_recovery': false
        }

        self.rospack = rospkg.RosPack()  # get path for ROS package
        self.dst_path = self.rospack.get_path('msh_navigation_config')
        self.dst_path = self.dst_path + '/robots/cob4-2/nav'

    def create_value_dict(self, do_param_dict):
        '''
        creates a dictionary with every key:value pair with step distance changed
        i.e. one parameter is changed and all the other params are untouched and from this set a dictionary is created
        :param do_param_dict: parameter dictionary
        :return: dictionary with all sets of key:value pairs
        '''
        value_count_list = [a['step'].__len__() for a in sorted(do_param_dict.values())]
        value_count_dict = {key: value['step'].__len__() for key, value in do_param_dict.iteritems()}

        key_values_list = []
        key_possibilities_list = []

        # build list with all possibilities of one 'key' and store it inside another list
        for key, value in sorted(do_param_dict.iteritems()):
            for j in xrange(value_count_dict[key]):
                key_possibilities_list.append([key, value['step'][j]])
            key_values_list.append(key_possibilities_list)
            key_possibilities_list = []

        cartesian_product_list = []

        # multiply every single 'key'-list of 'key_values_list' with each other to create every possibilty as 'key'-'value' pair
        # 'cartesian product'
        for element in itertools.product(*key_values_list):
            cartesian_product_list.append(element)

        # create 'key'-'value' dictionaries inside list
        output_list = [{data[0]: data[1] for data in data_tuples} for data_tuples in cartesian_product_list]

        #	for i in output_list:
        #		print i

        print '=' * 80
        print 'Dictionarie length:', output_list.__len__()
        print '=' * 80

        counter = 1
        for dict in output_list:
            filename = 'move_base_eband_params' + '_' + str(counter) + '.yaml'
            stream = file('yaml_files_new/' + filename, 'w')
            for key, value in dict.iteritems():
                # print key, ':', value
                self.all_params[key] = value
            # generate 'yaml' file
            yaml.dump(self.all_params, stream, default_flow_style=False)
            stream.close()
            counter += 1

        print '=' * 80
        print '\'yaml\' generated'
        print '=' * 80

        return output_list

    def change_value(self):
        '''
        changes the value of a entered parameter name
        :return: --
        '''
        print '=' * 100
        print 'Which value do you want to change?\nPossible Options: '
        for par in self.do_params:
            print '- ' + par
        for par in self.maybe_params:
            print '- ' + par
        print '=' * 100
        print 'If you enter: \'c\' the \'yaml\' will be generated & the program exits.'

        # enter name of parameter to be changed
        par_name = raw_input('Enter:')
        if par_name == 'c':
            self.generated_yaml()
            exit()
        # if parameter should not be changed output an error
        if par_name not in self.do_params.keys():
            print '=' * 100
            print '\033[91m' + 'Wrong User Input' + '\033[0m'
            self.change_value()
        # print old value of parameter
        print '{}: {}'.format(par_name, self.do_params[par_name])
        # enter new parameter value
        par_value = input('New value:')
        self.do_params[par_name] = par_value
        # print changed value
        print '{}: {}'.format(par_name, self.do_params[par_name])
        # repeat for other values
        self.change_value()

    def copy_yaml(self, filepath):
        shutil.copy2('move_base_eband_params.yaml', filepath)

    def generated_yaml(self):
        '''
        generates a 'yaml' file with all parameters
        :return: --
        '''
        stream = file('move_base_eband_params.yaml', 'w')
        for key, value in self.do_params.iteritems():
            # print key, ':', value
            if type(value) is dict:
                self.all_params[key] = value['value'][1]
            else:
                self.all_params[key] = value
        # generate 'yaml' file
        yaml.dump(self.all_params, stream, default_flow_style=False)
        print '=' * 100
        print '\'yaml\' generated'
        print '=' * 100
        # TODO: where has the yaml-file be copied to?
        # copy 'yaml' file to desired destination
        # self.copy_yaml(self.dst_path)
        # print '\'yaml\' copied to: \'' + self.dst_path + '\''
        # print '=' * 100

    def main(self):
        # self.change_value()
        self.create_value_dict(self.do_params)


if __name__ == '__main__':
    ep = EbandParams()
    ep.main()
