#!/usr/bin/python

"""
Created on Nov 7, 2017

@author: flg-ma
@attention: Generate the 'move_base_eband_params.yaml' automatically
@contact: albus.marcel@gmail.com (Marcel Albus)
@version: 1.0.0
"""

import yaml

true = True
false = False

class EbandParams(object):
    """docstring for EbandParams"""

    def __init__(self):
        self.params = {
            # Maximum linear velocity
            'max_vel_lin': 1.0,

            # Maximum rotational velocity
            'max_vel_th': 0.5,

            # Damp maximal translational acceleration
            'max_translational_acceleration': 0.4,

            # Damp maximal rotational acceleration
            'max_rotational_acceleration': 0.2,

            # Scaling factor for veocity calculation
            'scaled_radius_factor': 4.0,

            # Controller settings
            'k_prop': 4.0,
            'k_damp': 3.5,

            # -----------------------------------------------------------------
            # eband_trajectory_controller_params for smoothing
            # -----------------------------------------------------------------
            # Activate smoothing option at start and end
            'smoothing_enabled': true,
            # -----------------------------------------------------------------
            # eband params classic
            # -----------------------------------------------------------------

            # Minimum relative overlap two bubbles must have to be treated as connected
            'eband_min_relative_bubble_overlap': 0.7,

            # gain for internal forces (Elasticity of Band)
            'eband_internal_force_gain': 1.0,

            # gain for external forces (Penalty on low distance to obstacles)
            'eband_external_force_gain': 2.0,

            # gain for external forces when the robot is allmost in collision
            'eband_external_force_near_obtacle_gain': 2.0}

        self.dst_path = ''

    def generated_yaml(self):
        stream = file('move_base_eband_params.yaml', 'w')
        # yaml.dump(self.params, stream)
        print yaml.dump(self.params)

    def main(self):
        self.generated_yaml()


if __name__ == '__main__':
    ep = EbandParams()
    ep.main()