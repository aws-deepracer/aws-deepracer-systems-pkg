#!/usr/bin/env python

#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

"""
scheduler.py

This module creates the Scheduler class which is responsible to run background
daemon thread that reads from a queue and executes the function calls in a separate
thread.
"""

import threading
import queue

from deepracer_systems_pkg import constants

#########################################################################################
# Scheduler class.


class Scheduler:
    """Class responsible to run background daemon thread and schedule function calls.
    """
    def __init__(self, logger):
        """Create the Scheduler object.

        Args:
            logger (rclpy.rclpy.impl.rcutils_logger.RcutilsLogger):
                Logger object of the software_update_node.
        """
        self.stop = threading.Event()
        self.queue = queue.Queue()
        self.logger = logger
        thread = threading.Thread(target=self.loop)
        thread.start()

    def loop(self):
        """Main daemon loop that reads from a queue and executes the function call.
        """
        self.logger.info('Entering daemon loop.')
        if constants.POST_LOOP_BREAK:
            self.schedule_action(self.kill)

        while not self.stop.isSet():
            action, keyworded_args = self.queue.get()
            if len(keyworded_args):
                action(keyworded_args)
            else:
                action()

        self.logger.info('Exiting daemon loop.')

    def schedule_action(self, action, **keyworded_args):
        """Helper method to add the function to the queue.

        Args:
            action (function): The function that is to be added to the execution queue.
        """
        self.queue.put((action, keyworded_args))

    def schedule_exit(self, **keyworded_args):
        """Helper method to stop the scheduler after executing all current calls in queue.
        """
        self.schedule_action(self.kill)

    def kill(self):
        """Wrapper method to set the stop event.
        """
        self.stop.set()
