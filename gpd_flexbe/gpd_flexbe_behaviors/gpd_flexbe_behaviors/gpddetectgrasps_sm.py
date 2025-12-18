#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2025 Brian Flynn
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define GPDDetectGrasps.

Using Grasp Pose Detection (GPD) grasp solver, determine candidate grasp poses

Created on Thu Nov 20 2025
@author: Brian Flynn
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_core import initialize_flexbe_core
from gpd_flexbe_states.detect_grasps_service_state import DetectGraspsServiceState
from gpd_flexbe_states.gpd_grasp_poses_service_state import GPDGraspPosesServiceState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class GPDDetectGraspsSM(Behavior):
    """
    Define GPDDetectGrasps.

    Using Grasp Pose Detection (GPD) grasp solver, determine candidate grasp poses
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'GPDDetectGrasps'

        # parameters of this behavior

        # Initialize ROS node information
        initialize_flexbe_core(node)

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]


        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create state machine."""
        # Root state machine
        # x:823 y:160, x:377 y:346
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['cloud_in', 'camera_source', 'view_points', 'indices'], output_keys=['grasp_approach_poses', 'grasp_target_poses', 'grasp_retreat_poses', 'grasp_waypoints'])
        _state_machine.userdata.cloud_in = 0
        _state_machine.userdata.camera_source = 0
        _state_machine.userdata.view_points = 0
        _state_machine.userdata.indices = []
        _state_machine.userdata.grasp_configs = 0
        _state_machine.userdata.grasp_approach_poses = 0
        _state_machine.userdata.grasp_target_poses = 0
        _state_machine.userdata.grasp_retreat_poses = 0
        _state_machine.userdata.grasp_waypoints = 0

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:185 y:98
            OperatableStateMachine.add('DetectGrasps',
                                       DetectGraspsServiceState(service_timeout=5.0,
                                                                service_name='/detect_grasps'),
                                       transitions={'done': 'ComputePoses'  # 375 110 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 263 242 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud': 'cloud_in',
                                                  'camera_source': 'camera_source',
                                                  'view_points': 'view_points',
                                                  'indices': 'indices',
                                                  'grasp_configs': 'grasp_configs'})

            # x:428 y:98
            OperatableStateMachine.add('ComputePoses',
                                       GPDGraspPosesServiceState(service_timeout=5.0,
                                                                 service_name='/compute_grasp_poses'),
                                       transitions={'done': 'finished'  # 720 129 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 505 265 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'grasp_configs': 'grasp_configs',
                                                  'grasp_target_poses': 'grasp_target_poses',
                                                  'grasp_approach_poses': 'grasp_approach_poses',
                                                  'grasp_retreat_poses': 'grasp_retreat_poses',
                                                  'grasp_waypoints': 'grasp_waypoints'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
