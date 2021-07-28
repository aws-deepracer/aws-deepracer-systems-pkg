# AWS DeepRacer systems package

## Overview

The AWS DeepRacer systems ROS package creates the `software_update_node`, `model_loader_node`, `otg_control_node`, `network_monitor_node` and `deepracer_systems_scripts_node`, which are part of the core AWS DeepRacer application and launch from the `deepracer_launcher`. For more information about the application and the components, see the [aws-deepracer-launcher repository](https://github.com/aws-deepracer/aws-deepracer-launcher).

These nodes are responsible for managing some system-level functionalities required in the AWS DeepRacer application. 

### `software_update_node`

This node is responsible for managing the update system for AWS DeepRacer packages. It provides services and functions that
check for software updates, execute software updates, provide the status of the current software update state and installation state, and install signed packages from a USB drive.

### `model_loader_node`

This node is responsible for managing AWS DeepRacer reinforcement learning models in the `/opt/aws/deepracer/artifacts` directory on the AWS DeepRacer device. It provides services and functions that load .tar.gz files with models from USB, extract .tar.gz files with models uploaded in the console, list models in the `/opt/aws/deepracer/artifacts` directory, verify model readiness through a checksum file check, and delete models through the console.

### `otg_control_node`

This node is responsible for checking for the micro-USB connection and enabling or disabling the ethernet-over-USB feature whenever there is a connection change. It provides services and functions that execute scripts to initialize, enable, and disable the ethernet-over-USB feature and detect the current micro-USB cable connection status.

### `network_monitor_node`

This node is responsible for monitoring and managing the network connection to the device. It provides services and functions that connect to Wi-Fi based on the content of the Wi-Fi configuration file read from USB, report the status of the connection attempt in the device status file created on the USB when updating the Wi-Fi configuration, manage the status LED light to indicate the network connection status, and broadcast the network connection status as a message.

### `deepracer_systems_scripts_node`

This node is responsible for running in background to verify and run dependency script commands in specific time intervals.


## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation
Follow these steps to install the AWS DeepRacer systems package.

### Prerequisites

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed to run the `deepracer_systems_pkg`. For more information about the preinstalled set of packages and libraries on the AWS DeepRacer and about installing the required build systems, see [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

The AWS `deepracer_systems_pkg` specifically depends on the following ROS 2 packages as build and run dependencies.

1. `deepracer_interfaces_pkg`: This package contains the custom message and service type definitions used across the AWS DeepRacer core application.
1. `status_led_pkg`: The AWS DeepRacer status LED ROS package creates the `status_led_node`, which is part of the core AWS DeepRacer application.
1. `usb_monitor_pkg`: The AWS DeepRacer USB monitor ROS package creates the `usb_monitor_node` which is part of the core AWS DeepRacer application.

## Downloading and building

Open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the `deepracer_systems_pkg` on the AWS DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-systems-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-systems-pkg
        rosws update

1. Resolve the `deepracer_systems_pkg` dependencies:

        cd ~/deepracer_ws/aws-deepracer-systems-pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `deepracer_systems_pkg`, `status_led_pkg`, `usb_monitor_pkg`, and `deepracer_interfaces_pkg`:

        cd ~/deepracer_ws/aws-deepracer-systems-pkg && colcon build --packages-select deepracer_systems_pkg status_led_pkg usb_monitor_pkg deepracer_interfaces_pkg

## Usage

These nodes provide basic system-level functionality for the AWS DeepRacer application to work. Although the nodes are built to work with the AWS DeepRacer application, you can run them independently for development, testing, and debugging purposes.

### Run the node

To launch the built nodes as the root user on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-systems-pkg/install/setup.bash

1. Launch the nodes using the launch script:

        ros2 launch deepracer_systems_pkg deepracer_systems_pkg_launch.py

## Launch files

The `deepracer_systems_pkg_launch.py`, included in this package, provides an example demonstrating how to launch the nodes independently from the core application.

    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='software_update_node',
                name='software_update_node'
            ),
            Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='model_loader_node',
                name='model_loader_node'
            ),
            Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='otg_control_node',
                name='otg_control_node'
            ),
            Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='network_monitor_node',
                name='network_monitor_node'
            ),
            Node(
                package='deepracer_systems_pkg',
                namespace='deepracer_systems_pkg',
                executable='deepracer_systems_scripts_node',
                name='deepracer_systems_scripts_node'
            )
        ])


## Node details

### software_update_node

#### Subscribed topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|/`usb_monitor_pkg`/`usb_file_system_notification`|`USBFileSystemNotificationMsg`|This message holds the file and directory details that are broadcasted whenever a watched file is identified in the USB connection.|
|/`deepracer_systems_pkg`/`network_connection_status`|`NetworkConnectionStatus`|This message provides the network connection status to indicate if the device is connected to the internet.|

#### Published topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|/`deepracer_systems_pkg`/`software_update_pct`|`SoftwareUpdatePctMsg`|Publish a message with the current software update percentage and status.|


#### Service clients

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
|/`status_led_pkg`/`led_blink`|`SetStatusLedBlinkSrv`|Client to the `status_led_blink` service that is called to indicate the progress of the software update operation.|
|/`status_led_pkg`/`led_solid`|`SetStatusLedSolidSrv`|Client to the `status_led_solid` service that is called to indicate the success or failure of the software update operation.|
|/`usb_monitor_pkg`/`usb_file_system_subscribe`|`USBFileSystemSubscribeSrv`|Client to the `usb_file_system_subscribe` service that adds an `update` directory to the watchlist and notifies if it finds it in the USB drive.|
|/`usb_monitor_pkg`/`usb_mount_point_manager`|`USBMountPointManagerSrv`|Client to the USB mount point manager service to indicate that the `usb_monitor_node` can safely decrement the counter for the mount point once the action function for the `update` folder file being watched by `software_update_node` has run successfully.|


#### Services

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
|`software_update_check`|`SoftwareUpdateCheckSrv`|A service that is called to check if a software update is available for `aws-deepracer-*` packages.|
|`begin_update`|`BeginSoftwareUpdateSrv`|A service that is called to execute the software update and install the latest AWS DeepRacer packages.|
|`software_update_state`|`SoftwareUpdateStateSrv`|A service that is called to get the current software update state on the vehicle. Values include [ UPDATE_UNKNOWN, UP_TO_DATE, UPDATE_AVAILABLE, UPDATE_PENDING, UPDATE_IN_PROGRESS ].|

### model_loader_node

#### Subscribed topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|/`usb_monitor_pkg`/`usb_file_system_notification`|`USBFileSystemNotificationMsg`|This message holds the file and directory details broadcasted whenever a watched file is identified on the USB connection.|

#### Service clients

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
|/`status_led_pkg`/`led_blink`|`SetStatusLedBlinkSrv`|Client to the `status_led_blink` service that is called to indicate the progress of the load model operation.|
|/`status_led_pkg`/`led_solid`|`SetStatusLedSolidSrv`|Client to the `status_led_solid` service that is called to indicate the success or failure of the load model operation.|
|/`usb_monitor_pkg`/`usb_file_system_subscribe`|`USBFileSystemSubscribeSrv`|Client to the `usb_file_system_subscribe` service to add a "models" directory to the watchlist and notify if it finds it on the USB drive.|
|/`usb_monitor_pkg`/`usb_mount_point_manager`|`USBMountPointManagerSrv`|Client to the usb_mount_point_manager service to indicate that the `usb_monitor_node` can safely decrement the counter for the mount point once the action function for the "models" directory file being watched by `model_loader_node` runs successfully.|
|/`model_optimizer_pkg`/`model_optimizer_server`|`ModelOptimizeSrv`|Client to the model_optimizer_service that is called to optimize the model loaded from the USB and to create the intermediate representation of the models.|

#### Services

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
|`verify_model_ready`|`VerifyModelReadySrv`|A service that is called when a new model is loaded to verify if the model was extracted successfully.|
|`console_model_action`|`ConsoleModelActionSrv`|A service that is called with actions to upload or delete models from device console. It supports actions to extract and copy a .tar.gz file with a model uploaded from the console or to delete a model that is present in the /`opt`/`aws`/`deepracer`/`artifacts` directory.|

### `network_monitor_node`

#### Subscribed topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|/`usb_monitor_pkg`/`usb_file_system_notification`|`USBFileSystemNotificationMsg`|This message holds the file and directory details broadcasted whenever a watched file is identified on the USB connection.|

#### Published topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|/`deepracer_systems_pkg`/`network_connection_status`|`NetworkConnectionStatus`|Publish a message with the network connection status to indicate if the device is connected to the internet.|

#### Service clients

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
|/`status_led_pkg`/`led_blink`|`SetStatusLedBlinkSrv`|Client to the `status_led_blink` service that is called to indicate the progress of the Wi-Fi connection operation.|
|/`status_led_pkg`/`led_solid`|`SetStatusLedSolidSrv`|Client to the `status_led_solid` service that is called to indicate the success or failure of the Wi-Fi connection operation.|
|/`usb_monitor_pkg`/`usb_file_system_subscribe`|`USBFileSystemSubscribeSrv`|Client to the `usb_file_system_subscribe` service to add  the `wifi-creds.txt` file to the watchlist and to notify if it finds it on the USB drive.|
|/`usb_monitor_pkg`/`usb_mount_point_manager`|`USBMountPointManagerSrv`|Client to the `usb_mount_point_manager` service to indicate that the `usb_monitor_node` can safely decrement the counter for the mount point once the action function for the for the Wi-Fi configuration file being watched by `network_monitor_node` runs successfully.|

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
