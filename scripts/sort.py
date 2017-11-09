#!/usr/bin/python

import yaml
import itertools

true = True
false = False

params_all_2 = {
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
		'drive_residual_band': false,
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
		'use_local_replanning': false,
		'virtual_mass': 0.75,
		'wait_for_recovery': false}


MIN = 0
DEFAULT = 1
MAX = 2

do_params = {
            # Maximum linear velocity
            'max_vel_lin': {'value':[0.0, 1.0, 2.0], 'step': [0.75, 1.25]},
            # Maximum rotational velocity
            'max_vel_th': {'value':[0.0, 0.5, 2.0], 'step': [0.2, 1.5]},
            # Damp maximal translational acceleration
            'max_translational_acceleration': {'value':[0.0, 0.4, 1.5],'step': [0.3, 1.0]},
            # Damp maximal rotational acceleration
            'max_rotational_acceleration': {'value':[0.0, 0.2, 1.0],'step': [0.4, 0.8]},
            # Scaling factor for veocity calculation
            'scaled_radius_factor': {'value':[0.0, 4.0, 7.0],'step': [2.0, 6.0]},
            # Controller settings
            'k_prop': {'value':[0.0, 4.0, 10.0],'step': [2.0, 7.0]},
            'k_damp': {'value':[0.0, 3.5, 10.0],'step': [2.0, 7.0]},
            # Activate smoothing option at start and end
            'smoothing_enabled': {'value':[true, true, false], 'step': [true, false]},
            # Minimum relative overlap two bubbles must have to be treated as connected
            'eband_min_relative_bubble_overlap': {'value':[0.5, 0.7, 1.0],'step': [0.5, 0.9]},
            'drive_residual_band': {'value':[true, true, false], 'step': [true, false]},
            # -----------------------------------------------------------------
            # select kinematics: {omnidirectional, differential}
            # -----------------------------------------------------------------
            'kinematics_string': {'value':['omnidirectional', 'omnidirectional', 'differential'], 'step': ['omnidirectional', 'differential']}
        }


# for param, value_dict in do_params.iteritems():
#	print value_dict['step']

list1 = ['test', 'test2', 'test3']
list2 = [5, 6, 7]
list3 = ['Ni', 'Nini', 'Ninini']
dict1 = {'eins': {'values': [1,2]},
		 'zwei': {'values': [11,22,33]},
		 'drei': {'values': [111,222]}}

value_count_list = [a['values'].__len__() for a in sorted(dict1.values())]

value_count_dict = {key: value['values'].__len__() for key, value in dict1.iteritems()}

# print value_count_dict
# print value_count_list

key_values_list = []
key_possibilities_list = []

# build list with all possibilities of one 'key' and store it inside another list
for key, value in sorted(dict1.iteritems()):
 	for j in xrange(value_count_dict[key]):
 		key_possibilities_list.append([key, value['values'][j]])
 	key_values_list.append(key_possibilities_list)
 	key_possibilities_list = []

cartesian_product_list = []

# multiply every single 'key'-list of 'key_values_list' with each other to create every possibilty as 'key'-'value' pair
# 'cartesian product'
for element in itertools.product(*key_values_list):
	cartesian_product_list.append(element)

# create 'key'-'value' dictionaries inside list
output_list = [{data[0]: data[1] for data in data_tuples} for data_tuples in cartesian_product_list]

for i in output_list:
	print i
print '=' * 80
print output_list