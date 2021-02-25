# DeepRacer Systems Package

## Overview

The DeepRacer Systems ROS package creates the *software_update_node, model_loader_node, otg_control_node, network_monitor_node* which are part of the core AWS DeepRacer application and will be launched from the deepracer_launcher. More details about the application and the components can be found [here](https://github.com/aws-racer/aws-deepracer-launcher).

These nodes are responsible for managing some system level functionalities required in the DeepRacer application. 

### software_update_node

This node is responsible for managing the update system for DeepRacer packages. It provides services and functions to
check for software update, execute software update, provide status of the current software update state and installation state, install signed packages from USB drive.

### model_loader_node

This node is responsible for managing DeepRacer reinforcement learning models in the */opt/aws/deepracer/artifacts* folder on the DeepRacer device. It provides services and functions to load tar.gz files with model from usb, extract tar.gz files with model uploaded in the console, list models in /opt/aws/deepracer/artifacts folder, verify model readiness through checksum file check and delete models through console.

### otg_control_node

This node is responsible responsible to check for the micro-USB connection and enable/disable ethernet-over-USB feature whenever there is a connection change. It provides services and functions to execute scripts to initialize, enable, disable ethernet-over-USB feature and find out about current the micro-USB cable connection status.

### network_monitor_node

This node is is responsible to monitor and manage network connection to the device. It provides services and functions to connect to WiFi based on the content of the WiFi configuration file read from usb, report the status of the connection attempt back in the device status file created on the USB when updating WiFi configuration, manage the status LED light to indicate the network connection status and broadcast the network connection status as a message.

## License

The source code is released under Apache 2.0 (https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The DeepRacer device comes with all the pre-requisite packages, build systems and libraries installed to build and run the deepracer_systems_pkg. More details about pre installed set of packages and libraries on the DeepRacer can be found in the [Getting Started](https://github.com/aws-racer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The deepracer_systems_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

1. *deepracer_interfaces_pkg* - This packages contains the custom message and service type definitions used across the AWS DeepRacer core application.
1. *status_led_pkg* - The DeepRacer Status LED ROS package creates the *status_led_node* which ** is part of the core AWS DeepRacer application.
1. *usb_monitor_pkg* - The DeepRacer USB Monitor ROS package creates the *usb_monitor_node* which ** is part of the core AWS DeepRacer application.

## Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Create a workspace directory for the package:

        mkdir deepracer_ws
        cd deepracer_ws

1. Clone the deepracer_systems_pkg, status_led_pkg, usb_monitor_pkg and the deepracer_interfaces_pkg on the DeepRacer device:

        git clone https://github.com/aws-racer/aws-deepracer-interfaces-pkg
        git clone https://github.com/aws-racer/aws-deepracer-status-led-pkg
        git clone https://github.com/aws-racer/aws-deepracer-usb-monitor-pkg
        git clone https://github.com/aws-racer/aws-deepracer-systems-pkg

1. Resolve the deepracer_systems_pkg dependencies:

        rosdep install -i --from-path . --rosdistro foxy -y

1. Build the deepracer_systems_pkg, status_led_pkg, usb_monitor_pkg and deepracer_interfaces_pkg:

    colcon build --packages-select deepracer_systems_pkg status_led_pkg usb_monitor_pkg deepracer_interfaces_pkg

## Usage

These nodes provide basic system level functionality for the AWS DeepRacer application to work. Although the nodes is built to work with the AWS DeepRacer application, it can be run independently for development/testing/debugging purposes.

### Run the node

To launch the built nodes as root user on the DeepRacer device open up another terminal on the DeepRacer device and run the following commands as root user:

1. Navigate to the deepracer workspace:

        cd deepracer_ws

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source install/setup.bash 

1. Launch the nodes using the launch script:

        ros2 launch deepracer_systems_pkg deepracer_systems_pkg_launch.py

## Launch Files

The  deepracer_systems_pkg_launch.py is also included in this package that gives an example of how to launch the nodes independently from the core application.

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
            )
        ])


## Node Details

### software_update_node

#### Subscribed Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|/usb_monitor_pkg/usb_file_system_notification|USBFileSystemNotificationMsg|This message holds the file/folder details that is broadcasted, whenever a watched file is identified in the USB connected.|
|/deepracer_systems_pkg/network_connection_status|NetworkConnectionStatus|This message with network connection status to indicate if the device is connected to the internet.|

#### Published Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|/deepracer_systems_pkg/software_update_pct|SoftwareUpdatePctMsg|Publish a message with current software update percentage and status.|


#### Service Clients

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|/status_led_pkg/led_blink|SetStatusLedBlinkSrv|Client to the status LED blink service that is called to indicate progress of software update operation.|
|/status_led_pkg/led_solid|SetStatusLedSolidSrv|Client to the status LED solid service that is called to indicate success/failure of software update operation.|
|/usb_monitor_pkg/usb_file_system_subscribe|USBFileSystemSubscribeSrv|Client to USB File system subscription service to add  "update" folder to watchlist and trigger notification if it finds it in the USB drive.|
|/usb_monitor_pkg/usb_mount_point_manager|USBMountPointManagerSrv|Client to USB Mount point manager service to indicate that the usb_monitor_node can safely decrement the counter for the mount point once the action function for the "update" folder file being watched by software_update_node is succesfully executed.|


#### Services

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|software_update_check|SoftwareUpdateCheckSrv|A service that is called to check if a software update is available for aws-deepracer-* packages.|
|begin_update|BeginSoftwareUpdateSrv|A service that is called to execute the software update and install latest deepracer packages.|
|software_update_state|SoftwareUpdateStateSrv|A service that is called to get the current software update state on the vehicle. Values include [ UPDATE_UNKNOWN, UP_TO_DATE, UPDATE_AVAILABLE, UPDATE_PENDING, UPDATE_IN_PROGRESS ].|

### model_loader_node

#### Subscribed Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|/usb_monitor_pkg/usb_file_system_notification|USBFileSystemNotificationMsg|This message holds the file/folder details that is broadcasted, whenever a watched file is identified in the USB connected.|

#### Service Clients

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|/status_led_pkg/led_blink|SetStatusLedBlinkSrv|Client to the status LED blink service that is called to indicate progress of load model operation.|
|/status_led_pkg/led_solid|SetStatusLedSolidSrv|Client to the status LED solid service that is called to indicate success/failure of load model operation.|
|/usb_monitor_pkg/usb_file_system_subscribe|USBFileSystemSubscribeSrv|Client to USB File system subscription service to add  "models" folder to watchlist and trigger notification if it finds it in the USB drive.|
|/usb_monitor_pkg/usb_mount_point_manager|USBMountPointManagerSrv|Client to USB Mount point manager service to indicate that the usb_monitor_node can safely decrement the counter for the mount point once the action function for the "models" folder file being watched by model_loader_node is succesfully executed.|
|/model_optimizer_pkg/model_optimizer_server|ModelOptimizeSrv|Client to model optimizer service that is called to optimize the models that is loaded from the USB and create the intermediate representation of the models.|

#### Services

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|verify_model_ready|VerifyModelReadySrv|A service that is called when a new model is loaded to verify if the model was extracted successfully.|
|console_model_action|ConsoleModelActionSrv|A service that is called with actions to upload/delete models from device console. It supports actions to extract and copy a tar.gz file with model that was uploaded from the console or delete a model that is present in the /opt/aws/deepracer/artifacts folder.|

### network_monitor_node

#### Subscribed Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|/usb_monitor_pkg/usb_file_system_notification|USBFileSystemNotificationMsg|This message holds the file/folder details that is broadcasted, whenever a watched file is identified in the USB connected.|

#### Published Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|/deepracer_systems_pkg/network_connection_status|NetworkConnectionStatus|Publish a message with network connection status to indicate if the device is connected to the internet.|

#### Service Clients

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|/status_led_pkg/led_blink|SetStatusLedBlinkSrv|Client to the status LED blink service that is called to indicate progress of WiFi connection operation.|
|/status_led_pkg/led_solid|SetStatusLedSolidSrv|Client to the status LED solid service that is called to indicate success/failure of WiFi connection operation.|
|/usb_monitor_pkg/usb_file_system_subscribe|USBFileSystemSubscribeSrv|Client to USB File system subscription service to add  wifi-creds.txt file to watchlist and trigger notification if it finds it in the USB drive.|
|/usb_monitor_pkg/usb_mount_point_manager|USBMountPointManagerSrv|Client to USB Mount point manager service to indicate that the usb_monitor_node can safely decrement the counter for the mount point once the action function for the for the WiFi configuration file being watched by network_monitor_node is succesfully executed.|

