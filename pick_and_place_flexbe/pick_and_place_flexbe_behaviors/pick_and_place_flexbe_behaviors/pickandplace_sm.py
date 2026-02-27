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
Define PickAndPlacePipelineGPD.

Perform a pick and place operation using PCL filtering perception (passthrough,
plane segmentation, euclidean clustering), a grasp solving algorithm (such as
GPD), and MoveIt Task Constructor (MTC)-based motion planning and execution
nodes

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
from gpd_flexbe_behaviors.gpddetectgrasps_sm import GPDDetectGraspsSM
from move_group_flexbe_states.move_to_named_pose_service_state import MoveToNamedPoseServiceState
from mtc_flexbe_behaviors.mtchandlepickandplace_sm import MTCHandlePickAndPlaceSM
from pcl_flexbe_behaviors.euclideanclusterextraction_sm import EuclideanClusterExtractionSM
from pcl_flexbe_states.get_point_cloud_service_state import GetPointCloudServiceState
from planning_scene_flexbe_states.add_collision_object_state import AddCollisionObjectServiceState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class PickAndPlacePipelineGPDSM(Behavior):
    """
    Define PickAndPlacePipelineGPD.

    Perform a pick and place operation using PCL filtering perception (passthrough,
    plane segmentation, euclidean clustering), a grasp solving algorithm (such as
    GPD), and MoveIt Task Constructor (MTC)-based motion planning and execution
    nodes
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'PickAndPlacePipelineGPD'

        # parameters of this behavior

        # Initialize ROS node information
        initialize_flexbe_core(node)

        # references to used behaviors
        self.add_behavior(EuclideanClusterExtractionSM, 'EuclideanClusterExtraction', node)
        self.add_behavior(GPDDetectGraspsSM, 'GPDDetectGrasps', node)
        self.add_behavior(MTCHandlePickAndPlaceSM, 'MTCHandlePickAndPlace', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]


        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create state machine."""
        # Root state machine
        # x:1908 y:114, x:610 y:377
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.camera_pose = 0
        _state_machine.userdata.cloud_frame = 0
        _state_machine.userdata.scene_pointcloud = 0
        _state_machine.userdata.ready_pose = 'ready'
        _state_machine.userdata.filtered_cloud = 0
        _state_machine.userdata.indices = []
        _state_machine.userdata.camera_source = 0
        _state_machine.userdata.inspect_pose = 'inspect'

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:44 y:103
            OperatableStateMachine.add('AddGroundPlaneCollision',
                                       AddCollisionObjectServiceState(service_name='/add_collision_object',
                                                                      x=0.0,
                                                                      y=0.0,
                                                                      z=-0.005,
                                                                      l=1.0,
                                                                      w=1.0,
                                                                      h=0.01),
                                       transitions={'finished': 'MoveInspect'  # 251 118 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 127 327 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

            # x:813 y:91
            OperatableStateMachine.add('EuclideanClusterExtraction',
                                       self.use_behavior(EuclideanClusterExtractionSM, 'EuclideanClusterExtraction'),
                                       transitions={'finished': 'GPDDetectGrasps'  # 1045 104 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 752 273 874 150 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Inherit,
                                                 'failed': Autonomy.Inherit},
                                       remapping={'cloud_in': 'scene_pointcloud',
                                                  'camera_pose': 'camera_pose',
                                                  'filtered_cloud': 'filtered_cloud'})

            # x:1095 y:89
            OperatableStateMachine.add('GPDDetectGrasps',
                                       self.use_behavior(GPDDetectGraspsSM, 'GPDDetectGrasps'),
                                       transitions={'finished': 'MoveReady',
                                                    'failed': 'failed'  # 1040 270 1143 148 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Inherit,
                                                 'failed': Autonomy.Inherit},
                                       remapping={'cloud_in': 'filtered_cloud',
                                                  'camera_source': 'camera_source',
                                                  'view_points': 'camera_pose',
                                                  'indices': 'indices',
                                                  'grasp_approach_poses': 'grasp_approach_poses',
                                                  'grasp_target_poses': 'grasp_target_poses',
                                                  'grasp_retreat_poses': 'grasp_retreat_poses',
                                                  'grasp_waypoints': 'grasp_waypoints'})

            # x:1604 y:87
            OperatableStateMachine.add('MTCHandlePickAndPlace',
                                       self.use_behavior(MTCHandlePickAndPlaceSM, 'MTCHandlePickAndPlace'),
                                       transitions={'finished': 'finished'  # 1823 115 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 1536 280 1636 146 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Inherit,
                                                 'failed': Autonomy.Inherit},
                                       remapping={'grasp_approach_poses': 'grasp_approach_poses',
                                                  'grasp_target_poses': 'grasp_target_poses',
                                                  'grasp_retreat_poses': 'grasp_retreat_poses'})

            # x:298 y:102
            OperatableStateMachine.add('MoveInspect',
                                       MoveToNamedPoseServiceState(service_timeout=5.0,
                                                                   service_name='/move_to_named_pose'),
                                       transitions={'finished': 'TakeSnapshot'  # 516 118 -1 -1 -1 -1
                                                    , 'failure': 'failed'  # 388 309 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failure': Autonomy.Off},
                                       remapping={'target_pose_name': 'inspect_pose'})

            # x:1337 y:88
            OperatableStateMachine.add('MoveReady',
                                       MoveToNamedPoseServiceState(service_timeout=5.0,
                                                                   service_name='/move_to_named_pose'),
                                       transitions={'finished': 'MTCHandlePickAndPlace',
                                                    'failure': 'failed'  # 1261 281 1383 141 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failure': Autonomy.Off},
                                       remapping={'target_pose_name': 'ready_pose'})

            # x:567 y:100
            OperatableStateMachine.add('TakeSnapshot',
                                       GetPointCloudServiceState(service_timeout=5.0,
                                                                 service_name='/get_point_cloud',
                                                                 camera_topic='/camera/depth/points',
                                                                 target_frame='panda_link0'),
                                       transitions={'finished': 'EuclideanClusterExtraction'  # 762 113 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 589 284 632 153 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'camera_pose': 'camera_pose',
                                                  'cloud_out': 'scene_pointcloud',
                                                  'cloud_frame': 'cloud_frame'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
