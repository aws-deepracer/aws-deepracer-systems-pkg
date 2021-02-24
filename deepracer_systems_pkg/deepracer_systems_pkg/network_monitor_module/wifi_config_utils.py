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

import subprocess
import operator

from deepracer_systems_pkg.network_monitor_module import network_config


#########################################################################################


def remove_ANSI_seq(string):
    """Remove the ANSI escape sequences from the string.

    Args:
        string (str): Input string

    Returns:
        str: Valid string after removing ANSI escape characters.
    """
    return network_config.ANSI_ESCAPE.sub("", string)


def clean_nmcli_response(raw_response):
    """Clean up the response obtained after running the nmcli command.

    Args:
        raw_response (str): Raw response obtained after running the nmcli commands.

    Returns:
        str: Cleaned up response.
    """
    response = list()

    raw_response = remove_ANSI_seq(raw_response.decode("utf-8"))
    raw_response = raw_response.replace("\r", "\n")
    raw_response = raw_response.split("\n")

    for line in raw_response:
        line = line.strip()
        if line != "":
            response.append(line)

    return response

#########################################################################################
# WiFi configuration file.


def parse_assignment(line):
    """Helper method to parse the lines with assignment(=) operator in them.

    Args:
        line (str): Line read from the WiFi configuration file.

    Returns:
        tuple: Tuple with name and value strings.
    """
    while True:
        # Ignore comments.
        if line.startswith("#"):
            break

        # Separate the variable from the value.
        assignment = line.split(":", 1)
        if len(assignment) != 2:
            break

        # Extract the variable name.
        name = assignment[0].strip()
        if name == "":
            break

        # Extract the value and stript leading and trailing quotes if any.
        value = assignment[1].strip()
        if value.startswith("\'") and value.endswith("\'"):
            value = value[1:-1]
        if value.startswith("\"") and value.endswith("\""):
            value = value[1:-1]

        return str(name), str(value)

    return None, None


def parse_wifi_config(config_file_path):
    """Parse the WiFi configuration file.

    Args:
        config_file_path (str): Path to the WiFi configuration file.

    Returns:
        dict: All the entries in the file which are of the form name: value.
    """
    wifi_config = None

    try:
        with open(config_file_path) as config_file:
            config_lines = [line.strip() for line in config_file.readlines()]

        wifi_config = dict()

        for line in config_lines:
            line = line.strip()

            # Parse the line as an assignment.
            name, value = parse_assignment(line)
            if name is not None:
                # Store names in lower case; we are ignoring the case of the variables.
                wifi_config[name.lower()] = value

    except Exception as ex:
        return {}

    return wifi_config


def get_available_wifi_networks():
    """Returns available WiFi networks as a list sorted by signal strength.

    Returns:
        list: List of available WiFi networks.
    """
    try:
        cmd = ["nmcli",
               "-terse",
               "-colors", "no",
               "-fields", "ssid,signal",
               "-escape", "no",
               "device", "wifi"]

        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = p.communicate()
        if (p.returncode != 0) or (err != ""):
            raise Exception(err)

        network_dict = dict()
        for line in out.splitlines():
            try:
                network, signal = line.split(":", 1)
                if (network not in network_dict) or (int(signal) > network_dict[network]):
                    network_dict[network] = int(signal)
            except Exception as ex:
                pass

        return sorted(network_dict.items(), key=operator.itemgetter(1), reverse=True)

    except Exception as ex:
        print(f"Failed to retrieve available WiFi networds: {ex}")
        return list()


def connect_wifi(ssid, password):
    """Attempts to connect to the specified WiFi network.

    Args:
        ssid (str): WiFI network name to connect to.
        password (str): WiFi password.

    Returns:
        tuple: Success flag and list of the command response lines.
    """
    cmd = ["nmcli",
           "device"]

    # Add SSID to connect to.
    cmd.extend(["wifi", "connect", ssid])

    # Add password if specified.
    if password != "":
        cmd.extend(["password", password])

    try:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = p.communicate()
        returncode = p.returncode

    except Exception as ex:
        out = ""
        err = f"Error while executing nmcli: {ex}"
        returncode = -1

    finally:
        out_response = clean_nmcli_response(out)
        err_response = clean_nmcli_response(err)
        response = out_response + err_response

        success = (returncode == 0)
        result = list()

        for line in response:
            if line == "":
                continue
            result.append(line)
            if "error" in line.lower():
                success = False

        if success:
            result.insert(0, str(f"Successfully connected to \"{ssid}\":"))
        else:
            result.insert(0, str(f"({returncode}) Failed to connect to \"{ssid}\":"))

        return success, result
