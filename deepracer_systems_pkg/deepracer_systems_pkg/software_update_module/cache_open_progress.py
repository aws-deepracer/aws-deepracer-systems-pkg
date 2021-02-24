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
cache_open_progress.py

This module creates the CacheOpenProgress class inherited from apt.progress.base.OpProgress
responsible for logging the progress of cache open operation.
"""

import apt


#########################################################################################
# CacheOpenProgress class.

class CacheOpenProgress(apt.progress.base.OpProgress):
    """Subclass that tracks the cache open operation.

    Args:
        apt.progress.base.OpProgress: Inherits and overrides functions from OpProgress class.
    """
    def __init__(self, logger):
        """Create the CacheOpenProgress object.

        Args:
            logger (rclpy.rclpy.impl.rcutils_logger.RcutilsLogger):
                Logger object of the software_update_node.
        """
        apt.progress.base.OpProgress.__init__(self)
        self.logger = logger

    def update(self):
        """ Overridden method that updates the progress of all the cache open process.
        """
        try:
            self.logger.info(f"* Cache opening progress{self.percent}")
        except Exception as error:
            self.logger.error(f"Error while updating cache progress: {error}")

    def done(self):
        """ Overridden method that is called when cache open process is done.
        """
        self.logger.info("* Cache successfully opened.")
