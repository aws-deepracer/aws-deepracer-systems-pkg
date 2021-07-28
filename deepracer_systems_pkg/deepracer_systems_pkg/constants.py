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

import pwd
import grp
import os
from enum import Enum

#########################################################################################
# DeepRacer Systems Scripts Constants

# Timer period (seconds) for the dependency function callback
CHK_DEP_TIMER_PERIOD = 600

RSYSLOG_CONFIG_FILE = "/etc/rsyslog.d/50-default.conf"
RSYSLOG_RULE = "if ($programname == \"kernel\" and $msg contains \"uvcvideo: Buffer is NULL\") then stop\n"
RSYSLOG_RESTART_CMD = "sudo service rsyslog restart"


#########################################################################################
# USBFileSystem services.

USB_FILE_SYSTEM_NOTIFICATION_TOPIC = "/usb_monitor_pkg/usb_file_system_notification"
USB_FILE_SYSTEM_SUBSCRIBE_SERVICE_NAME = "/usb_monitor_pkg/usb_file_system_subscribe"
USB_MOUNT_POINT_MANAGER_SERVICE_NAME = "/usb_monitor_pkg/usb_mount_point_manager"


#########################################################################################
# LED services.

LED_BLINK_SERVICE_NAME = "/status_led_pkg/led_blink"
LED_SOLID_SERVICE_NAME = "/status_led_pkg/led_solid"


#########################################################################################
# File system constants.

NOBODY_UID = pwd.getpwnam("nobody").pw_uid
NOGROUP_GID = grp.getgrnam("nogroup").gr_gid


#########################################################################################
# Scheduler configuration.

POST_LOOP_BREAK = False


#########################################################################################
# LED indices (0..2).


class LEDIndex():
    POWER_LED = 2
    WIFI_LED = 0


class LEDColor():
    RED = "red"
    GREEN = "green"
    BLUE = "blue"
    BLACK = "black"
    WHITE = "white"
    NO_COLOR = "no_color"
    ON = "on"

#########################################################################################
# Common configuration.


BASE_PATH = "/opt/aws/deepracer/"
# Day 0 / Mandatory Software Update status
SOFTWARE_UPDATE_STATUS_PATH = os.path.join(BASE_PATH, "software_update_status.json")
TEMP_DIRECTORY = os.path.join(os.sep, "var", "tmp")
KERNEL_CONFIG_DIRECTORY = os.path.join(os.sep, "sys", "kernel", "config")


class SensorInputKeys(Enum):
    """Enum mapping sensor inputs keys(str) passed in model_metadata.json to int values,
       as we add sensors we should add inputs. This is also important for networks
       with more than one input.
    """
    observation = 1
    LIDAR = 2
    SECTOR_LIDAR = 3
    LEFT_CAMERA = 4
    FRONT_FACING_CAMERA = 5
    STEREO_CAMERAS = 6

    @classmethod
    def has_member(cls, input_key):
        """Check if the input_key passed as parameter is one of the class members.

        Args:
            input_key (str): String containing sensor input key to check.

        Returns:
            bool: Returns True if the sensor input key is supported, False otherwise.
        """
        return input_key in cls.__members__


class TrainingAlgorithms(Enum):
    """Enum mapping training algorithm value passed in model_metadata.json to int values.
    """
    clipped_ppo = 1
    sac = 2

    @classmethod
    def has_member(cls, training_algorithm):
        """Check if the training_algorithm passed as parameter is one of the class members.

        Args:
            training_algorithm (str): String containing training algorithm to check.

        Returns:
            bool: Returns True if the training algorithm is supported, False otherwise.
        """
        return training_algorithm in cls.__members__


class ModelMetadataKeys():
    """Class with keys in the model metadata.json
    """
    SENSOR = "sensor"
    LIDAR_CONFIG = "lidar_config"
    TRAINING_ALGORITHM = "training_algorithm"
    NUM_LIDAR_SECTORS = "num_sectors"
    USE_LIDAR = "use_lidar"


# Default Lidar configuration values
DEFAULT_LIDAR_CONFIG = {
    # Number of lidar sectors to feed into network
    ModelMetadataKeys.NUM_LIDAR_SECTORS: 64,
}

# Default Sector Lidar configuration values
DEFAULT_SECTOR_LIDAR_CONFIG = {
    # Number of lidar sectors to feed into network
    ModelMetadataKeys.NUM_LIDAR_SECTORS: 8
}
