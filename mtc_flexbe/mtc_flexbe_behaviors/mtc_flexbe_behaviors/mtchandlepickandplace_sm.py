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
Define MTCHandlePickAndPlace.

Use MoveIt Task Constructor (MTC) tasks to plan and execute two separate
operations plus gripper operation: an approach and retreat task followed buy a
close gripper command, then a retreat and place task

Created on Thu Nov 20 2025
@author: Brian Flynn
"""


from end_effector_flexbe_states.gripper_command_action_state import GripperCommandActionState
from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_core import initialize_flexbe_core
from flexbe_states.wait_state import WaitState
from mtc_flexbe_states.mtc_approach_and_pick_action_state import MTCApproachAndPickActionState
from mtc_flexbe_states.mtc_retreat_and_place_action_state import MTCRetreatAndPlaceActionState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class MTCHandlePickAndPlaceSM(Behavior):
    """
    Define MTCHandlePickAndPlace.

    Use MoveIt Task Constructor (MTC) tasks to plan and execute two separate
    operations plus gripper operation: an approach and retreat task followed buy a
    close gripper command, then a retreat and place task
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'MTCHandlePickAndPlace'

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
        # x:1055 y:176, x:484 y:318
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['grasp_approach_poses', 'grasp_target_poses', 'grasp_retreat_poses'])
        _state_machine.userdata.grasp_approach_poses = 0
        _state_machine.userdata.grasp_target_poses = 0
        _state_machine.userdata.grasp_retreat_poses = 0
        _state_machine.userdata.object_id = 0
        _state_machine.userdata.approach_and_pick_index = 0
        _state_machine.userdata.retreat_and_place_index = 0
        _state_machine.userdata.robot_name = 'panda'
        _state_machine.userdata.close = 0.0
        _state_machine.userdata.max_effort = 40.0

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:116 y:70
            OperatableStateMachine.add('ApproachAndPick',
                                       MTCApproachAndPickActionState(timeout_sec=5.0,
                                                                     action_name='mtc_approach_and_pick',
                                                                     num_attempts=10),
                                       transitions={'success': 'WaitToClose'  # 352 78 -1 -1 -1 -1
                                                    , 'next': 'RetryWait'  # 299 170 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 359 252 -1 -1 -1 -1
                                                    },
                                       autonomy={'success': Autonomy.Off,
                                                 'next': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'grasp_approach_poses': 'grasp_approach_poses',
                                                  'grasp_target_poses': 'grasp_target_poses',
                                                  'grasp_retreat_poses': 'grasp_retreat_poses',
                                                  'object_id': 'object_id',
                                                  'grasp_index': 'approach_and_pick_index'})

            # x:624 y:70
            OperatableStateMachine.add('CloseDelay',
                                       WaitState(wait_time=0.5),
                                       transitions={'done': 'RetreatAndPlace'  # 741 82 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off})

            # x:452 y:176
            OperatableStateMachine.add('CloseGripper',
                                       GripperCommandActionState(timeout_sec=5.0,
                                                                 action_name_fmt='/hand_controller/gripper_cmd'),
                                       transitions={'success': 'CloseDelay'  # 654 173 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 524 270 -1 -1 -1 -1
                                                    },
                                       autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'robot_name': 'robot_name',
                                                  'position': 'close',
                                                  'max_effort': 'max_effort'})

            # x:780 y:66
            OperatableStateMachine.add('RetreatAndPlace',
                                       MTCRetreatAndPlaceActionState(timeout_sec=5.0,
                                                                     action_name='mtc_retreat_and_place',
                                                                     num_attempts=10),
                                       transitions={'success': 'finished'  # 1001 111 -1 -1 -1 -1
                                                    , 'next': 'RetryWait2'  # 964 166 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 713 233 -1 -1 -1 -1
                                                    },
                                       autonomy={'success': Autonomy.Off,
                                                 'next': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'grasp_approach_poses': 'grasp_approach_poses',
                                                  'grasp_target_poses': 'grasp_target_poses',
                                                  'grasp_retreat_poses': 'grasp_retreat_poses',
                                                  'object_id': 'object_id',
                                                  'grasp_index': 'retreat_and_place_index'})

            # x:160 y:185
            OperatableStateMachine.add('RetryWait',
                                       WaitState(wait_time=0.5),
                                       transitions={'done': 'ApproachAndPick'  # 111 169 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off})

            # x:820 y:179
            OperatableStateMachine.add('RetryWait2',
                                       WaitState(wait_time=0.5),
                                       transitions={'done': 'RetreatAndPlace'  # 773 177 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off})

            # x:383 y:66
            OperatableStateMachine.add('WaitToClose',
                                       WaitState(wait_time=0.5),
                                       transitions={'done': 'CloseGripper'  # 437 161 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
