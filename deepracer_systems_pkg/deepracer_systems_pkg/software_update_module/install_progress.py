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
install_progress.py

This module creates the InstallProgress class inherited from
apt.progress.base.InstallProgress responsible for logging and reporting the progress
of package installation operation.
"""

import apt
from deepracer_systems_pkg.software_update_module import software_update_config


#########################################################################################
# InstallProgress class.

class InstallProgress(apt.progress.base.InstallProgress):
    """Subclass that tracks the progress of package installations.

    Args:
        apt.progress.base.InstallProgress:
            Inherits and overrides functions from InstallProgress class.
    """
    def __init__(self, pct_dict_db, logger):
        """Create the InstallProgress object.

        Args:
            pct_dict_db (DoubleBuffer): Reference to a double buffer passed as reference
                                        containing the current update percentage,
                                        which will be updated as download progresses.
            logger (rclpy.rclpy.impl.rcutils_logger.RcutilsLogger):
                Logger object of the software_update_node.
        """
        apt.progress.base.InstallProgress.__init__(self)
        self.pct_dict_db = pct_dict_db
        self.logger = logger

    def error(self, pkg, errormsg):
        """Overridden method that is invoked when error is detected during installation.

        Args:
            pkg (apt_pkg.Package): Stores information about the package.
            errormsg (str): Error message.
        """
        self.logger.error(f"Error in installation for package {pkg}: {errormsg}")

    def status_change(self, pkg, percent, status):
        """Overridden method that updates the progress of all the package installations
           pkg - Current package being installed.

        Args:
            pkg (apt_pkg.Package): Current package being installed.
            percent (float): Overall installation progress percentage.
            status (str): Current status in readable format.
        """
        try:
            pct = min(software_update_config.PACKAGE_DOWNLOAD_PCT
                      + software_update_config.PACKAGE_INSTALL_PCT * percent / 100.0,
                      software_update_config.INSTALL_UPDATE_MAX_PCT)
            self.logger.info(f"Package {pkg} is being installed, "
                             f"installation percentation: {percent}; "
                             f"overall percentage: {pct}")
            self.set_update_pct(pct)
        except KeyError as error:
            self.logger.error(f"Missing pct_dict key {error}")

    def start_update(self):
        """Overridden method that is invoked when the package installation begins.
        """
        try:
            self.logger.info("Pacakge installation started")
            # Set state to installing
            self.set_update_pct(software_update_config.PACKAGE_DOWNLOAD_PCT)
        except KeyError as error:
            self.logger.error(f"Missing pct_dict key {error}")

    def finish_update(self):
        """Overridden method that is invoked when all of the packages are installed.
        """
        try:
            self.logger.info("Package installation complete")
            self.set_update_pct(software_update_config.INSTALL_UPDATE_MAX_PCT)
        except KeyError as error:
            self.logger.error(f"Missing pct_dict key {error}")

    def set_update_pct(self, pct):
        """Setter method that sets the publishes update percentage.

        Args:
            pct (float): Current installation percentage.
        """
        self.pct_dict_db.put({software_update_config.PROG_STATE_KEY:
                              software_update_config.PROGRESS_STATES[4],
                              software_update_config.PROG_PCT_KEY: pct})
