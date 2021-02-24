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
fetch_progress.py

This module creates the FetchProgress class inherited from
apt.progress.base.AcquireProgress responsible for logging and reporting the progress
of package download operation.
"""

import apt
from deepracer_systems_pkg.software_update_module import software_update_config


#########################################################################################
# FetchProgress class.

class FetchProgress(apt.progress.base.AcquireProgress):
    """Subclass that tracks the progress of package downloads.

    Args:
        apt.progress.base.AcquireProgress:
            Inherits and overrides functions from AcquireProgress class.
    """
    def __init__(self, pct_dict_db, logger):
        """Create the FetchProgress object.

        Args:
            pct_dict_db (DoubleBuffer): Reference to a double buffer passed as reference
                                        containing the current update percentage,
                                        which will be updated as download progresses.
            logger (rclpy.rclpy.impl.rcutils_logger.RcutilsLogger):
                Logger object of the software_update_node.
        """
        apt.progress.base.AcquireProgress.__init__(self)
        self.pct_dict_db = pct_dict_db
        self.logger = logger

    def pulse(self, owner):
        """Overridden method from base class, periodically checks the number of packages
            downloaded.

        Args:
            owner (apt_pkg.Acquire): apt_pkg.Acquire object invoking the method. See:
                                     https://apt-team.pages.debian.net/python-apt/libr .

        Returns:
            bool: Returns True (required).
        """
        try:
            if self.total_items != 0:
                self.set_update_pct(software_update_config.PACKAGE_DOWNLOAD_PCT *
                                    self.current_items / self.total_items)
                self.logger.info(f"Fetch has recieved {self.current_items} "
                                 f"items out of {self.total_items} total")
        except KeyError as error:
            self.logger.error(f"Missing pct_dict key {error}")

        return True

    def start(self):
        """Overridden method that is invoked when the package download begins.
        """
        try:
            self.logger.info("Package download started")
            # Set state to downloading
            self.set_update_pct(0.0)
        except KeyError as error:
            self.logger.error(f"Missing pct_dict key {error}")

    def stop(self):
        """Overridden method that is invoked when all of the packages are downloaded.
        """
        try:
            self.logger.info("Package download complete")
            self.set_update_pct(software_update_config.PACKAGE_DOWNLOAD_PCT)
        except KeyError as error:
            self.logger.error(f"Missing pct_dict key {error}")

    def set_update_pct(self, pct):
        """Setter method that sets the publishes update percentage.

        Args:
            pct (float): Current download percentage.
        """
        self.pct_dict_db.put({software_update_config.PROG_STATE_KEY:
                              software_update_config.PROGRESS_STATES[3],
                              software_update_config.PROG_PCT_KEY: pct})
