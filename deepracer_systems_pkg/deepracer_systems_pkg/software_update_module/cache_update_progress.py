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
cache_update_progress.py

This module creates the CacheUpdateProgress class inherited from
apt.progress.base.AcquireProgress responsible for logging the progress of cache
update operation.
"""

import apt


#########################################################################################
# CacheUpdateProgress class.

class CacheUpdateProgress(apt.progress.base.AcquireProgress):
    """Subclass that tracks the cache update operation.

    Args:
        apt.progress.base.AcquireProgress:
            Inherits and overrides functions from AcquireProgress class.
    """
    def __init__(self, logger):
        """Create the CacheUpdateProgress object.

        Args:
            logger (rclpy.rclpy.impl.rcutils_logger.RcutilsLogger):
                Logger object of the software_update_node.
        """
        apt.progress.base.AcquireProgress.__init__(self)
        self.logger = logger

    def pulse(self, owner):
        """Overridden method from base class, periodically checks the number of items
            downloaded in the cache.

        Args:
            owner (apt_pkg.Acquire): apt_pkg.Acquire object invoking the method. See:
                                     https://apt-team.pages.debian.net/python-apt/libr .

        Returns:
            bool: Returns True (required).
        """
        try:
            self.logger.info(f"* Cache update has recieved {self.current_items} "
                             f"items out of {self.total_items} total")
        except Exception as error:
            self.logger.error(f"Error while obtaining cache update fetch progress: {error}")

        return True

    def start(self):
        """Overridden method that is invoked when the cache update begins.
        """
        try:
            self.logger.info("* Cache update started")
        except Exception as error:
            self.logger.error(f"Error while starting cache update: {error}")

    def stop(self):
        """Overridden method that is invoked cache update is stopped.
        """
        try:
            self.logger.info("* Cache update stopped")
        except Exception as error:
            self.logger.error(f"Error while stopping cache update: {error}")

    def done(self, item):
        """Overridden method that is invoked when cache update is fetched and completed.

        Args:
            item (apt_pkg.AcquireItemDesc): Stores information about the cache.
        """
        try:
            self.logger.info(f"* Cache update completed: {item.description}")
        except Exception as error:
            self.logger.error(f"Error while completing cache update: {error}")

    def fail(self, item):
        """Overridden method that is invoked when cache update fails.

        Args:
            item (apt_pkg.AcquireItemDesc): Stores information about the cache.
        """
        self.logger.error(f"* Cache update failed: {item.description}")
