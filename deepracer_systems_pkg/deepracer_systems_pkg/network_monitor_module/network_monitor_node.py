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
network_monitor_node.py

This module creates the network_monitor_node which is responsible to monitor and
manage network connection to the device. It provides services and functions to connect
to WiFi based on the content of the WiFi configuration file read from usb, report
the status of the connection attempt back in the device status file created on the
USB when updating WiFi configuration, manage the status LED light to indicate the network
connection status and broadcast the network connection status as a message.

The node defines:
    update_status_LED_timer: A timer created to periodically call the status led services
                             to update the WIFI_LED light with right color.
    report_state_timer: A timer created to periodically call the report state function that
                        attempts to write the connection report details to the USB connected.
    status_publisher_timer: A timer create to periodically call the
                            network_status_message_publisher.
    network_status_message_publisher: A publisher created to broadcast the current network
                                      connection status.
"""

import os
import time
import threading
import unidecode
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import (ReentrantCallbackGroup,
                                   MutuallyExclusiveCallbackGroup)

from deepracer_interfaces_pkg.msg import (NetworkConnectionStatus,
                                          USBFileSystemNotificationMsg)
from deepracer_interfaces_pkg.srv import (SetStatusLedBlinkSrv,
                                          SetStatusLedSolidSrv,
                                          USBFileSystemSubscribeSrv,
                                          USBMountPointManagerSrv)
from deepracer_systems_pkg import (file_system_utils,
                                   constants,
                                   scheduler)
from deepracer_systems_pkg.network_monitor_module import (network_utils,
                                                          network_config,
                                                          wifi_config_utils)


#########################################################################################
# Network monitor class.


class NetworkMonitorNode(Node):
    """Node responsible to monitor and manage network connection of the device.
    """

    def __init__(self):
        """Create a NetworkMonitorNode.
        """
        super().__init__("network_monitor_node")
        self.get_logger().info("network_monitor_node started")

        self.state_dir = ""
        # Threading Event object to stop calling status LED update until the WiFi configuration
        # update is completed.
        self.led_update_pause = threading.Event()
        # Scheduler to queue the function calls and run them in a separate thread.
        self.scheduler = scheduler.Scheduler(self.get_logger())

        # Timer to periodically update the WIFI_LED light with right color.
        if network_config.ENABLE_NETWORK_LED_UPDATE:
            self.update_status_LED_timer = self.create_timer(
                                            network_config.NETWORK_UPDATE_LED_PERIOD_IN_SECONDS,
                                            self.schedule_update_status_LED)

        # Timer to periodically attempt to write the connection status report to USB.
        if network_config.ENABLE_REPORT_STATE_UPDATE:
            self.report_state_timer = self.create_timer(
                                            network_config.REPORT_STATE_PERIOD_IN_SECONDS,
                                            self.schedule_report_state)

        # Publisher that broadcasts network connection status.
        self.network_status_pub_cb = ReentrantCallbackGroup()
        self.network_status_message_publisher = \
            self.create_publisher(NetworkConnectionStatus,
                                  network_config.NETWORK_CONNECTION_STATUS_TOPIC_NAME,
                                  1,
                                  callback_group=self.network_status_pub_cb)

        # Timer to periodically publish the network connection status.
        self.status_publisher_timer = self.create_timer(network_config.NETWORK_CONNECTION_PUBLISHER_PERIOD_IN_SECONDS,
                                                        self.publish_network_connection_status,
                                                        callback_group=self.network_status_pub_cb)

        # Clients to Status LED services that are called to indicate progress/success/failure
        # status while loading model.
        self.led_cb_group = MutuallyExclusiveCallbackGroup()
        self.led_blink_service = self.create_client(SetStatusLedBlinkSrv,
                                                    constants.LED_BLINK_SERVICE_NAME,
                                                    callback_group=self.led_cb_group)
        self.led_solid_service = self.create_client(SetStatusLedSolidSrv,
                                                    constants.LED_SOLID_SERVICE_NAME,
                                                    callback_group=self.led_cb_group)
        while not self.led_blink_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Led blink service not available, waiting again...")
        while not self.led_solid_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Led solid service not available, waiting again...")
        self.led_blink_request = SetStatusLedBlinkSrv.Request()
        self.led_solid_request = SetStatusLedSolidSrv.Request()

        # Client to USB File system subscription service that allows the node to connect to WiFi
        # based on the contents in the WiFi configuration file in the USB. The usb_monitor_node
        # will trigger notification if it finds the WiFi configuration file from the watchlist
        # in the USB drive.
        self.usb_sub_cb_group = ReentrantCallbackGroup()
        self.usb_file_system_subscribe_client = self.create_client(USBFileSystemSubscribeSrv,
                                                                   constants.USB_FILE_SYSTEM_SUBSCRIBE_SERVICE_NAME,
                                                                   callback_group=self.usb_sub_cb_group)
        while not self.usb_file_system_subscribe_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("File System Subscribe not available, waiting again...")

        # Client to USB Mount point manager service to indicate that the usb_monitor_node can safely
        # decrement the counter for the mount point once the action function for the WiFi configuration
        # file being watched by network_monitor_node is succesfully executed.
        self.usb_mpm_cb_group = ReentrantCallbackGroup()
        self.usb_mount_point_manager_client = self.create_client(USBMountPointManagerSrv,
                                                                 constants.USB_MOUNT_POINT_MANAGER_SERVICE_NAME,
                                                                 callback_group=self.usb_mpm_cb_group)
        while not self.usb_mount_point_manager_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("USB mount point manager service not available, waiting again...")

        # Subscriber to USB File system notification publisher to recieve the broadcasted messages
        # with file/folder details, whenever a watched file is identified in the USB connected.
        self.usb_notif_cb_group = ReentrantCallbackGroup()
        self.usb_file_system_notification_sub = self.create_subscription(USBFileSystemNotificationMsg,
                                                                         constants.USB_FILE_SYSTEM_NOTIFICATION_TOPIC,
                                                                         self.usb_file_system_notification_cb,
                                                                         10,
                                                                         callback_group=self.usb_notif_cb_group)

        # Add the "wifi-creds.txt" file to the watchlist.
        usb_file_system_subscribe_request = USBFileSystemSubscribeSrv.Request()
        usb_file_system_subscribe_request.file_name = network_config.WIFI_CONFIG_NAME
        usb_file_system_subscribe_request.callback_name = network_config.SCHEDULE_CONFIG_UPDATE_CB
        usb_file_system_subscribe_request.verify_name_exists = True
        self.usb_file_system_subscribe_client.call_async(usb_file_system_subscribe_request)

        # Heartbeat timer.
        self.timer_count = 0
        self.timer = self.create_timer(5.0, self.timer_callback)

        self.get_logger().info("Network Monitor node successfully created")

    def timer_callback(self):
        """Heartbeat function to keep the node alive.
        """
        self.get_logger().debug(f"Timer heartbeat {self.timer_count}")
        self.timer_count += 1

    def __enter__(self):
        """Called when the node object is created using the 'with' statement.

        Returns:
           NetworkMonitorNode : self object returned.
        """
        self.ssid = network_utils.get_current_SSID()
        self.hostname = network_utils.get_hostname()
        self.chassis_serial = network_utils.get_chassis_serial()

        if network_config.SET_HOSTNAME_TO_CHASSIS_SERIAL_NUMBER \
           and (self.hostname != self.chassis_serial):
            self.get_logger().info("Changing hostname to the chassis serial number: "
                                   f"{self.chassis_serial}")
            self.hostname = network_utils.set_hostname(self.chassis_serial)

        self.get_logger().info(f"Hostname: {self.hostname}")

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Called when the object is destroyed.
        """
        self.led_solid_request.led_index = constants.LEDIndex.WIFI_LED
        self.led_solid_request.color = constants.LEDColor.NO_COLOR
        self.led_solid_request.hold = 0.0
        self.led_solid_service.call_async(self.led_solid_request)

    def usb_file_system_notification_cb(self, notification_msg):
        """Callback for messages triggered whenever usb_monitor_node identifies a file/folder
           thats being tracked.

        Args:
            notification_msg (USBFileSystemNotificationMsg): Message containing information about the
                                                             file identified by the usb_monitor_node.
        """
        self.get_logger().info("File system notification:"
                               f" {notification_msg.path}"
                               f" {notification_msg.file_name}"
                               f" {notification_msg.node_name}"
                               f" {notification_msg.callback_name}")
        if notification_msg.file_name == network_config.WIFI_CONFIG_NAME and \
           notification_msg.callback_name == network_config.SCHEDULE_CONFIG_UPDATE_CB:
            self.schedule_config_update(path=notification_msg.path,
                                        name=notification_msg.file_name,
                                        node_name=notification_msg.node_name)

    def publish_network_connection_status(self):
        """Wrapper function to publish the network connection status.
        """
        msg = NetworkConnectionStatus()
        msg.network_connected = self.is_connected()
        self.network_status_message_publisher.publish(msg)

    def schedule_update_status_LED(self):
        """Wrapper function to schedule the update_status_LED function if there is no update
           configuration call going on.
        """
        if not self.led_update_pause.isSet():
            self.scheduler.schedule_action(self.update_status_LED)

    def schedule_report_state(self):
        """Wrapper function to schedule the update_status function.
        """
        self.get_logger().info("Scheduling report_state.")
        self.scheduler.schedule_action(self.report_state)

    def schedule_config_update(self, path, name, node_name):
        """Wrapper function to schedule the update_configuration function with the required
           arguments.

        Args:
            path (str): File path where the WiFi configuration file is present on the USB.
            name (str): Name of the WiFi configuration file (wifi-cred.txt).
            node_name (str): Filesystem where the USB was mounted.
        """
        self.scheduler.schedule_action(self.update_configuration,
                                       path=path,
                                       name=name,
                                       node_name=node_name)

    def update_status_LED(self):
        """Wrapper method to call the status LED services to update the WiFi LED light
           based on the network connection status.
        """
        if network_utils.get_current_SSID() != "":
            self.led_solid_request.color = constants.LEDColor.BLUE
        else:
            self.led_solid_request.color = constants.LEDColor.NO_COLOR
        self.led_solid_request.led_index = constants.LEDIndex.WIFI_LED
        self.led_solid_request.hold = 0.0
        self.led_solid_service.call_async(self.led_solid_request)

    def update_configuration(self, keyworded_args):
        """Main function to parse the WiFi configuration file, connect to WiFi,
           update WiFi LED status during the process and publish the final network
           connection status.

        Args:
            keyworded_args (dict): Keyworded arguments passed to the function while scheduling.
        """
        self.get_logger().info(f"Update Wifi Configuration: {keyworded_args}")
        connected = False
        connection_report = list()

        # Enable WiFi LED effect.
        self.led_update_pause.set()
        self.led_blink_request.led_index = constants.LEDIndex.WIFI_LED
        self.led_blink_request.color1 = constants.LEDColor.BLUE
        self.led_blink_request.color2 = constants.LEDColor.NO_COLOR
        self.led_blink_request.delay = 0.2
        self.led_blink_service.call_async(self.led_blink_request)

        base_path = keyworded_args.get("path", "")
        config_name = keyworded_args.get("name", "")
        node_name = keyworded_args.get("node_name", None)

        full_name = os.path.join(base_path, config_name)

        # Store and use base path as the target path to write status file to.
        self.state_dir = base_path

        # Read the configuration file.
        self.get_logger().info(f"Reading WiFi configuration {full_name}")
        config_dict = wifi_config_utils.parse_wifi_config(full_name)
        self.get_logger().info(f"Config dictionary read from file: {config_dict}")
        if config_dict is None:
            connection_report.append(f"{config_name}: failed to read WiFi configuration file.")

        if network_config.WiFiConfigKeys.SSID not in config_dict:
            connection_report.append(f"{config_name}: No SSID found in WiFi configuration file.")
        else:
            connected = self.connect_wifi_using_config(config_dict, connection_report)
        self.report_state(connection_report)

        # Unmount the media.
        if node_name is not None:
            mount_point_mgr_request = USBMountPointManagerSrv.Request()
            mount_point_mgr_request.node_name = node_name
            mount_point_mgr_request.action = 0
            self.usb_mount_point_manager_client.call_async(mount_point_mgr_request)
            self.state_dir = ""

        # Turn on alarm effect if failed.
        if not connected:
            self.led_solid_request.led_index = constants.LEDIndex.WIFI_LED
            self.led_solid_request.color = constants.LEDColor.RED
            self.led_solid_request.hold = 2.0
            self.led_solid_service.call_async(self.led_solid_request)

        # Set the correct LED color.
        self.update_status_LED()
        self.led_update_pause.clear()

        # Publish the network connection status
        self.publish_network_connection_status()
        self.get_logger().info("Updating WiFi configuration completed")

    def connect_wifi_using_config(self, config_dict, connection_report):
        """Helper method to attempt to connect to WiFi using the details passed in the
           config_dict and append the results of the execution ot the connection report.

        Args:
            config_dict (dict): Dictionary with the SSID, Password and connection name details.
            connection_report (list): Report that is passed by reference to which the connection
                                      results are appended.

        Returns:
            bool: True if the connection was successful else False.
        """
        # Extract configuration fields.
        ssid = config_dict.get(network_config.WiFiConfigKeys.SSID)
        password = \
            config_dict.get(network_config.WiFiConfigKeys.PASSWORD,
                            network_config.WIFI_CONFIG_DEFAULTS[network_config.WiFiConfigKeys.PASSWORD])
        connection_name = \
            config_dict.get(network_config.WiFiConfigKeys.CONNECTION_NAME,
                            network_config.WIFI_CONFIG_DEFAULTS[network_config.WiFiConfigKeys.CONNECTION_NAME])
        self.get_logger().info(f"{ssid}, {password}, {connection_name}")

        # Ignore if already connected to the same.
        if (ssid == network_utils.get_current_SSID()):
            self.get_logger().info("No change in configuration found, ignoring.")
            connection_report.append("No change in configuration found, ignoring.")
            return True

        # No need to clear the previous connections, just Attempt to connect to the new SSID.
        # If new SSID fails,Ubuntu will revert to the previous SSID
        # Else will connect to the new SSID
        connection_successful, connection_response = wifi_config_utils.connect_wifi(ssid, password)
        self.get_logger().error(f"First attempt at connection: {connection_successful} {connection_response}")
        connected = connection_successful
        if not connection_successful:
            self.get_logger().error(f"Connection was not successful: {connection_response}")
            connection_report.extend(connection_response)
            # Verify whether we failed because of the wrong password.
            bad_pass = ("secrets were required" in line for line in connection_response)

            # Match the input SSID to one of the available networks.
            if not bad_pass:
                # Get the list of active networks.
                networks = wifi_config_utils.get_available_wifi_networks()
                matched_network = None
                self.get_logger().info(f"Available networks: {networks}")
                for network, _ in networks:
                    try:
                        # Convert the names to ASCII.
                        a_ssid = unidecode.unidecode(ssid.decode("utf-8"))
                        if a_ssid == unidecode.unidecode(network.decode("utf-8")):
                            matched_network = network
                            break
                    except Exception as ex:
                        pass
                if matched_network is not None:
                    self.get_logger().info(f"Second attempt to connect to a matched network: "
                                           f"{matched_network}")
                    connection_report.append("Second attempt to connect to a matched network: "
                                             f"{matched_network}")
                    connection_successful, connection_response = \
                        wifi_config_utils.connect_wifi(matched_network, password)
                    self.get_logger().error("Second attempt at connection: "
                                            f"{connection_successful} {connection_response}")
                    connected = connection_successful
                    connection_report.extend(connection_response)
                else:
                    self.get_logger().error("Could not find a matching network for: "
                                            f"{ssid}")
                    connection_report.append("Could not find a matching network for: "
                                             f"{ssid}")

        else:
            self.get_logger().info(f"Successfully connected to: {ssid}")
            connection_report.extend(connection_response)
        return connected

    def report_state(self, message=None):
        """Write the network connection report to the state_dir if the path is set.

        Args:
            message ([str, tuple, list], optional): Message that needs to be written to
                                                    the device connection status file.
                                                    Defaults to None.
        """
        self.get_logger().info(f"report_state: {message}")
        state_directory = ""
        if self.state_dir != "":
            state_directory = self.state_dir

        if state_directory != "":
            state_filename = os.path.join(state_directory, network_config.DEVICE_STATUS_NAME)
            state_file_lines = list()
            # First read the state file if exists and remove the state for this host.
            try:
                skipping = False
                for line in open(state_filename):
                    line = line.rstrip()
                    if skipping and (line == ""):
                        skipping = False
                        continue
                    if line.lstrip().startswith("Hostname:"):
                        hostname = line.split(":")[1].strip()
                        skipping = (hostname == self.hostname)

                    if not skipping:
                        state_file_lines.append(line)
            except Exception:
                pass

            try:
                with open(state_filename, 'w') as state_file:
                    # Write back the state line for other hosts.
                    for line in state_file_lines:
                        print(line, file=state_file)

                    # Write the state for this host.
                    print(f"Hostname: {self.hostname}", file=state_file)
                    print(f"Timestamp: {time.strftime('%x %X')}", file=state_file)

                    if type(message) in (list, tuple):
                        for line in message:
                            self.get_logger().info(line)
                            print(line, file=state_file)

                    elif type(message) is str:
                        self.get_logger().info(message)
                        print(message, file=state_file)

                    ssid = network_utils.get_current_SSID()
                    if ssid == "":
                        ssid_message = "Not connected to WiFi network."
                    else:
                        ssid_message = f"Connected to WiFi network: {ssid}"

                    self.get_logger().info(ssid_message)
                    print(ssid_message, file=state_file)

                    connections = network_utils.get_net_connections()
                    if len(connections) != 0:
                        print("Active connections:", file=state_file)

                        for interface, ip in connections.items():
                            print(f"  {interface}: {ip}", file=state_file)

                    print("", file=state_file)

            except Exception as ex:
                self.get_logger().error(f"Failed to write state file {state_filename}: {ex}")

            file_system_utils.make_writable(state_filename)

    def is_connected(self):
        """Helper method to return the network connection status.

        Returns:
            bool: True if network is connected else False
        """
        return (len(network_utils.get_net_connections()) != 0)


def main(args=None):
    rclpy.init(args=args)
    with NetworkMonitorNode() as network_monitor_node:
        executor = MultiThreadedExecutor()
        rclpy.spin(network_monitor_node, executor)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        network_monitor_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
