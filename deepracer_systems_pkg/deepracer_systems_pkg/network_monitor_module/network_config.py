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

import os
import re


NETWORK_CONNECTION_STATUS_TOPIC_NAME = "network_connection_status"

#########################################################################################
# Network manager configuration.

ENABLE_NETWORK_LED_UPDATE = True
NETWORK_UPDATE_LED_PERIOD_IN_SECONDS = 2

ENABLE_REPORT_STATE_UPDATE = True
REPORT_STATE_PERIOD_IN_SECONDS = (60 * 5)

NETWORK_CONNECTION_PUBLISHER_PERIOD_IN_SECONDS = 10

# WiFi configuration file.
WIFI_CONFIG_NAME = "wifi-creds.txt"


# Keys expected in the WiFi configuration file.
class WiFiConfigKeys():
    SSID = "ssid"
    PASSWORD = "password"
    CONNECTION_NAME = "connection"
    FORCE_CONNECT = "forceConnect"


# Default values for the Wifi configuration fields.
WIFI_CONFIG_DEFAULTS = {
    WiFiConfigKeys.FORCE_CONNECT: "false",
    WiFiConfigKeys.CONNECTION_NAME: "",
    WiFiConfigKeys.PASSWORD: ""
}

# Device status file.
DEVICE_STATUS_NAME = "device-status.txt"

# Set to True to set hostname to the chassis serial number.
SET_HOSTNAME_TO_CHASSIS_SERIAL_NUMBER = True
CHASSIS_SERIAL = os.path.join(os.sep, "sys", "class", "dmi", "id", "chassis_serial")

SCHEDULE_CONFIG_UPDATE_CB = "schedule_config_update"

#########################################################################################
# WiFi management.

ANSI_REGEX = r"\x1b(" \
            r"(\[\??\d+[hl])|" \
            r"([=<>a-kzNM78])|" \
            r"([\(\)][a-b0-2])|" \
            r"(\[\d{0,2}[ma-dgkjqi])|" \
            r"(\[\d+;\d+[hfy]?)|" \
            r"(\[;?[hf])|" \
            r"(#[3-68])|" \
            r"([01356]n)|" \
            r"(O[mlnp-z]?)|" \
            r"(/Z)|" \
            r"(\d+)|" \
            r"(\[\?\d;\d0c)|" \
            r"(\d;\dR))"

ANSI_ESCAPE = re.compile(ANSI_REGEX, flags=re.IGNORECASE)
