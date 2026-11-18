#!/usr/bin/env python3

# Copyright 2023 Christopher Newport University
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

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from planning_scene_ros2.srv import LoadPlanningScene as SrvType


class LoadPlanningSceneServiceState(EventState):
    """Call the load_planning_scene service to reset the world collision scene.

    -- scene_name       str     Scene filename in the planning_scene_ros2 scenes directory
    -- service_name     str     Service name (default: '/load_planning_scene')

    <= finished                 Scene was loaded successfully
    <= failed                   Service was unavailable or the scene failed to load
    """

    def __init__(self, scene_name='', service_name='/load_planning_scene'):
        super().__init__(outcomes=['finished', 'failed'])
        self._scene_name = scene_name
        self._service_name = service_name

        # Create proxy service caller to handle rclpy node
        self._srv = ProxyServiceCaller({self._service_name: SrvType})

        # result storage
        self._res = None
        self._had_error = False

    def execute(self, userdata):
        # Execute this method periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self._had_error or self._res is None:
            return 'failed'

        Logger.loginfo(
            f'[{type(self).__name__}] Service response: {self._res.message}')
        if not self._res.success:
            Logger.logerr(
                f'[{type(self).__name__}] Failed to load scene: {self._res.message}')
            return 'failed'
        return 'finished'

    def on_enter(self, userdata):
        # Call this method a single time when the state becomes active, when a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        self._res = None
        self._had_error = False

        request = SrvType.Request()
        request.scene_name = self._scene_name

        if not self._srv.is_available(self._service_name):
            Logger.logerr(
                f"[{type(self).__name__}] Service '{self._service_name}' not available.")
            self._had_error = True
            return

        try:
            self._res = self._srv.call(self._service_name, request)
            Logger.loginfo(
                f"[{type(self).__name__}] Called service '{self._service_name}' "
                f"for scene '{self._scene_name}'.")
        except Exception as exc:
            Logger.logerr(
                f'[{type(self).__name__}] Service call failed: {exc}')
            self._had_error = True


    def on_exit(self, userdata):
        # Call this method when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.

        # No-op: template hook
        pass

    def on_start(self):
        # Call this method when the behavior is instantiated on board.
        # If possible, it is generally better to initialize used resources in the constructor
        #   because if anything failed, the behavior would not even be started.

        # No-op: template hook
        pass

    def on_stop(self):
        # Call this method whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up things like claimed resources.

        # No-op: template hook
        pass
