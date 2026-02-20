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
Define EuclideanClusterExtraction.

Filter the incoming cloud by extracting the indices of clusters within the
scene, then publish that cloud on 'target_object' topic for awareness

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
from pcl_flexbe_states.euclidean_clustering_service_state import EuclideanClusteringServiceState
from pcl_flexbe_states.filter_by_indices_service_state import FilterByIndicesServiceState
from pcl_flexbe_states.passthrough_filter_service_state import PassthroughServiceState
from pcl_flexbe_states.plane_segmentation_service_state import PlaneSegmentationServiceState
from pcl_flexbe_states.publish_point_cloud_state import PublishPointCloudState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class EuclideanClusterExtractionSM(Behavior):
    """
    Define EuclideanClusterExtraction.

    Filter the incoming cloud by extracting the indices of clusters within the
    scene, then publish that cloud on 'target_object' topic for awareness
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'EuclideanClusterExtraction'

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
        # x:1307 y:214, x:389 y:348
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['cloud_in', 'camera_pose'], output_keys=['filtered_cloud'])
        _state_machine.userdata.cloud_in = 0
        _state_machine.userdata.camera_pose = 0
        _state_machine.userdata.target_cluster_indices = 0
        _state_machine.userdata.obstacle_cluster_indices = 0
        _state_machine.userdata.filtered_cloud = 0
        _state_machine.userdata.plane_indices = 0
        _state_machine.userdata.plane_coefficients = 0
        _state_machine.userdata.plane_inlier_count = 0

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:61 y:96
            OperatableStateMachine.add('FilterBelowTable',
                                       PassthroughServiceState(service_timeout=5.0,
                                                               service_name='/passthrough_filter',
                                                               lower_limit=0.005,
                                                               upper_limit=0.25,
                                                               field='z'),
                                       transitions={'finished': 'SegmentTablePlane'  # 241 115 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 162 266 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'cloud_in',
                                                  'cloud_filtered': 'filtered_cloud'})

            # x:531 y:94
            OperatableStateMachine.add('EuclideanClustering',
                                       EuclideanClusteringServiceState(service_timeout=5.0,
                                                                       service_name='/euclidean_clustering',
                                                                       cluster_tolerance=0.02,
                                                                       min_cluster_size=100,
                                                                       max_cluster_size=25000),
                                       transitions={'finished': 'FilterByIndices'  # 741 106 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 620 250 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'filtered_cloud',
                                                  'camera_pose': 'camera_pose',
                                                  'target_cluster_indices': 'target_cluster_indices',
                                                  'obstacle_cluster_indices': 'obstacle_cluster_indices'})

            # x:786 y:91
            OperatableStateMachine.add('FilterByIndices',
                                       FilterByIndicesServiceState(service_timeout=5.0,
                                                                   service_name='/filter_by_indices'),
                                       transitions={'finished': 'PublishCloud'  # 970 103 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 835 261 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'filtered_cloud',
                                                  'target_indices': 'target_cluster_indices',
                                                  'cloud_out': 'filtered_cloud'})

            # x:1013 y:90
            OperatableStateMachine.add('PublishCloud',
                                       PublishPointCloudState(pub_topic='/filtered_cloud/target_object'),
                                       transitions={'done': 'finished'  # 1250 141 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 1056 266 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'filtered_cloud'})

            # x:284 y:95
            OperatableStateMachine.add('SegmentTablePlane',
                                       PlaneSegmentationServiceState(service_timeout=5.0,
                                                                     service_name='/plane_segmentation',
                                                                     use_voxel=True,
                                                                     leaf_size=0.01,
                                                                     distance_threshold=0.01,
                                                                     max_iterations=1000),
                                       transitions={'finished': 'EuclideanClustering'  # 498 113 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 390 272 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'filtered_cloud',
                                                  'without_plane': 'without_plane',
                                                  'plane_indices': 'plane_indices',
                                                  'plane_coefficients': 'plane_coefficients',
                                                  'inlier_count': 'inlier_count'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
