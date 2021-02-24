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
model_loader_node.py

This module creates the model_loader_node which is responsible for managing
DeepRacer reinforcement learning models in the /opt/aws/deepracer/artifacts folder.
It provides services and functions to load tar.gz files with model from usb,
extract tar.gz files with model uploaded in the console, list models in
/opt/aws/deepracer/artifacts folder, verify model readiness through checksum file
check and delete models through console.

The node defines:
    verify_model_ready_service: A service that is called when a model is loaded
                                to verify if the model was extracted successfully.
    console_model_action_service: A service that is called with actions to upload/delete
                                  models from device console. It supports actions to
                                  extract and copy a tar.gz file with model that was
                                  uploaded from the console or delete a model that is
                                  present in the /opt/aws/deepracer/artifacts folder.
"""

import os
import glob
import shutil
import gzip
import tarfile
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import (ReentrantCallbackGroup,
                                   MutuallyExclusiveCallbackGroup)

from deepracer_interfaces_pkg.srv import (VerifyModelReadySrv,
                                          ConsoleModelActionSrv,
                                          ModelOptimizeSrv,
                                          SetStatusLedBlinkSrv,
                                          SetStatusLedSolidSrv,
                                          USBFileSystemSubscribeSrv,
                                          USBMountPointManagerSrv)
from deepracer_interfaces_pkg.msg import USBFileSystemNotificationMsg
from deepracer_systems_pkg import (file_system_utils,
                                   constants,
                                   scheduler,
                                   utility)

from deepracer_systems_pkg.model_loader_module import (model_loader_config,
                                                       model_install_state)


#########################################################################################
# Model Loader class.

class ModelLoaderNode(Node):
    """Node responsible for loading models from usb, extracting models from tar.gz files,
       listing models, verifying model readiness and deleting models.
    """

    def __init__(self):
        """Create a ModelLoaderNode.
        """
        super().__init__("model_loader_node")
        self.get_logger().info("model_loader_node started")

        self.models_in_progress = dict()
        # Threading lock object to safely perform the model operations.
        self.progress_guard = threading.Lock()
        # Scheduler to queue the function calls and run them in a separate thread.
        self.scheduler = scheduler.Scheduler(self.get_logger())

        # Flag to enable deleting all existing models in /opt/aws/deepracer/artifacts folder
        # before copying the models from USB.
        self.enable_model_wipe = model_loader_config.ENABLE_MODEL_WIPE

        # Flag to enable model optimizer while transferring the models. Default set to False.
        if model_loader_config.ENABLE_MODEL_OPTIMIZER:
            self.model_optimizer_client = self.create_client(ModelOptimizeSrv,
                                                             model_loader_config.MODEL_OPTIMIZER_SERVER_SERVICE)
        else:
            self.model_optimizer_client = None

        # Supported file extensions and their corresponding action functions.
        self.supported_exts = {
            ".pb": self.copymodel,
            ".json": self.copymodel,
            ".gz": self.unzip,
            ".tar": self.untar
        }

        # Supported model extension.
        self.model_file_extensions = (".pb")

        # Service that is called when a model is loaded to verify if the model
        # was extracted successfully.
        self.verify_model_ready_cb_group = ReentrantCallbackGroup()
        self.verify_model_ready_service = self.create_service(VerifyModelReadySrv,
                                                              model_loader_config.VERIFY_MODEL_READY_SERVICE_NAME,
                                                              self.verify_model_ready_cb,
                                                              callback_group=self.verify_model_ready_cb_group)

        # A service that is called to extract a tar.gz file with model uploaded from the console
        # or delete a model selected.
        self.console_model_action_cb_group = ReentrantCallbackGroup()
        self.console_model_action_service = self.create_service(ConsoleModelActionSrv,
                                                                model_loader_config.CONSOLE_MODEL_ACTION_SERVICE_NAME,
                                                                self.console_model_action_cb,
                                                                callback_group=self.console_model_action_cb_group)

        # Clients to Status LED services that are called to indicate progress/success/failure
        # status while loading model.
        self.led_cb_group = MutuallyExclusiveCallbackGroup()
        self.led_blink_client = self.create_client(SetStatusLedBlinkSrv,
                                                   constants.LED_BLINK_SERVICE_NAME,
                                                   callback_group=self.led_cb_group)
        self.led_solid_client = self.create_client(SetStatusLedSolidSrv,
                                                   constants.LED_SOLID_SERVICE_NAME,
                                                   callback_group=self.led_cb_group)
        while not self.led_blink_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Led blink service not available, waiting again...")
        while not self.led_solid_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Led solid service not available, waiting again...")
        self.led_blink_request = SetStatusLedBlinkSrv.Request()
        self.led_solid_request = SetStatusLedSolidSrv.Request()

        # Client to USB File system subscription service that allows the node to add the "models"
        # folder to the watchlist. The usb_monitor_node will trigger notification if it finds
        # the files/folders from the watchlist in the USB drive.
        self.usb_sub_cb_group = ReentrantCallbackGroup()
        self.usb_file_system_subscribe_client = self.create_client(USBFileSystemSubscribeSrv,
                                                                   constants.USB_FILE_SYSTEM_SUBSCRIBE_SERVICE_NAME,
                                                                   callback_group=self.usb_sub_cb_group)
        while not self.usb_file_system_subscribe_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("File System Subscribe not available, waiting again...")

        # Client to USB Mount point manager service to indicate that the usb_monitor_node can safely
        # decrement the counter for the mount point once the action function for the file/folder being
        # watched by model_loader_node is succesfully executed.
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

        # Add the "models" folder to the watchlist.
        usb_file_system_subscribe_request = USBFileSystemSubscribeSrv.Request()
        usb_file_system_subscribe_request.file_name = model_loader_config.MODEL_SOURCE_LEAF_DIRECTORY
        usb_file_system_subscribe_request.callback_name = model_loader_config.SCHEDULE_MODEL_LOADER_CB
        usb_file_system_subscribe_request.verify_name_exists = True
        self.usb_file_system_subscribe_client.call_async(usb_file_system_subscribe_request)

        # Heartbeat timer.
        self.timer_count = 0
        self.timer = self.create_timer(5.0, self.timer_callback)

        self.get_logger().info("Model Loader node successfully created")

    def timer_callback(self):
        """Heartbeat function to keep the node alive.
        """
        self.get_logger().debug(f"Timer heartbeat {self.timer_count}")
        self.timer_count += 1

    def set_enable_model_wipe_flag(self, enable_model_wipe):
        """Setter method to set the enable model wipe flag.

        Args:
            enable_model_wipe (bool): Set to true to delete all models in the /opt/aws/deepracer/artifacts folder
                                      while uploading models using USB.
        """
        self.enable_model_wipe = enable_model_wipe

    def verify_model_ready_cb(self, req, res):
        """Callback for the verify_model_ready service. Verifies if the model was successfully extracted.

        Args:
            req (VerifyModelReadySrv.Request): Request object with the model folder name in the
                                               /opt/aws/deepracer/artifacts folder.
            res (VerifyModelReadySrv.Response): Response object with response_status flag to indicate if the
                                                model is ready.

        Returns:
            VerifyModelReadySrv.Response: Response object with response_status flag to indicate if the
                                          model is ready.
        """
        self.get_logger().info(f"verify_model_ready_cb({req.model_name})")
        res.response_status = self.verify_model_ready(req.model_name)
        return res

    def console_model_action_cb(self, req, res):
        """Callback for the console_model_action service. Handles the upload model and delete model
           actions from the console.

        Args:
            req (ConsoleModelActionSrv.Request): Request object with the model directory
                                                 (/opt/aws/deepracer/artifacts/<<model_name>>).
            res (ConsoleModelActionSrv.Response): Response object with status(str) containing
                                                  the details about the action execution status.

        Returns:
            ConsoleModelActionSrv.Response: Response object with status(str) containing
                                            the details about the action execution status.
        """
        self.get_logger().info(f"console_model_action(directory-{req.model_path} action-{req.action})")
        if not os.path.isdir(req.model_path):
            self.get_logger().error(f"console_model_action path-{req.model_path} does-not-exist")
            res.status = "model-path-does-not-exist"
            return res
        # Do not wipe all the models when loading a new model from the cosole
        self.set_enable_model_wipe_flag(False)
        # upload the model(s) from the specified directory
        status = self.console_model_action(req.model_path, req.action)
        # Wipe all the models for USB loaded models
        self.set_enable_model_wipe_flag(True)
        self.get_logger().info(f"console_model_action({req.model_path,} action{req.action}) .status {status}")
        res.status = status
        return res

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
        if notification_msg.file_name == model_loader_config.MODEL_SOURCE_LEAF_DIRECTORY and \
           notification_msg.callback_name == model_loader_config.SCHEDULE_MODEL_LOADER_CB:
            self.schedule_model_loader(path=notification_msg.path,
                                       name=notification_msg.file_name,
                                       node_name=notification_msg.node_name)

    def schedule_model_loader(self, path, name, node_name):
        """Wrapper function to schedule the load_model function with the required arguments.

        Args:
            path (str): File path where the model is present in the USB
            name (str): Name of the model folder
            node_name (str): Filesystem where the USB was mounted.
        """
        self.scheduler.schedule_action(self.load_models,
                                       path=path,
                                       name=name,
                                       node_name=node_name)

    def copymodel(self, full_source_name, target_directory):
        """Action function called for a .pb or .json file to copy the files from
           source path to target directory.

        Args:
            full_source_name (str): .pb or .json file path.
            target_directory (str): Target directory path where these files are to be copied.

        Returns:
            str: Full target path of the file.
        """
        filename = os.path.basename(full_source_name)
        full_target_name = os.path.splitext(os.path.join(target_directory, filename))[0]
        self.get_logger().info(f"    copying to {target_directory}...")
        shutil.copy(full_source_name, target_directory)
        return full_target_name

    def unzip(self, full_source_name, target_directory):
        """Action function called for a .gz file to unzip the file and copy them from
           source path to target directory.

        Args:
            full_source_name (str): .gz file path.
            target_directory (str): Target directory path where these files are to be copied.

        Returns:
            str: Full target path of the place where the unzipped files are copied.
        """
        filename = os.path.basename(full_source_name)
        full_target_name = os.path.splitext(os.path.join(target_directory, filename))[0]
        self.get_logger().info(f"    unzipping to {target_directory}...")
        with gzip.open(full_source_name, "rb") as inFile:
            with open(full_target_name, "wb") as outFile:
                shutil.copyfileobj(inFile, outFile)
        return full_target_name

    def untar(self, full_source_name, target_directory):
        """Action function called for a .tar file to untar the file and copy them from
           source path to target directory.

        Args:
            full_source_name (str): .tar file path.
            target_directory (str): Target directory path where these files are to be copied.

        Returns:
            str: Full target path of the place where the untarred files are copied.
        """
        filename = os.path.basename(full_source_name)
        full_target_name = os.path.splitext(os.path.join(target_directory, filename))[0]
        self.get_logger().info(f"    untarring to {target_directory}...")
        with tarfile.TarFile(full_source_name, "r") as srcFile:
            srcFile.extractall(target_directory)
        return full_target_name

    def get_list_of_archives(self, search_path):
        """Helper function to get the list of files with the supported extensions
           in the search path.

        Args:
            search_path (str): Path of the folder to search the files.

        Returns:
            list: List of file name which have the supported extensions.
        """
        list_of_archives = list()

        try:
            for filename in os.listdir(search_path):
                if not os.path.isfile(os.path.join(search_path, filename)):
                    continue

                ext_list = list()

                # Separate all archive extensions into a list.
                name_no_ext = filename
                while True:
                    components = os.path.splitext(name_no_ext)
                    name_no_ext = components[0]
                    ext = components[1]
                    if ext not in self.supported_exts:
                        break
                    ext_list.append(ext)

                # Ignore files that we do not support.
                if len(ext_list) == 0:
                    continue

                list_of_archives.append(filename)

        except Exception as ex:
            self.get_logger().error(f"Failed to list {search_path}: {ex}")

        return list_of_archives

    def extract_archive(self, filepath, target_directory):
        """Helper function to recursively split the filename in the filepath and
           call the mapped action function for the extension found.
           If the file contains multiple extensions(.tar.gz) then the unzip is triggered
           first and then untar is triggered.

        Args:
            filepath (str): Path to the file that should be run through
                            action functions recursively.
            target_directory (str): Path where the target files are to be copied to.

        Returns:
            bool: True if successfully executed all actions else False.
        """
        try:
            while True:
                ext = os.path.splitext(filepath)[1]
                if ext not in self.supported_exts:
                    break
                action = self.supported_exts[ext]
                filepath = action(filepath, target_directory)

            return True

        except Exception as ex:
            self.get_logger().error(f"    failed to decompress {filepath}: {ex}")
            return False

    def get_model_list(self, directory):
        """Returns the list of models having the supported model file extension in the
           directory passed as a parameter.

        Args:
            directory (str): Full path of the directory in which the model file
                             is to searched.

        Returns:
            list: List of tuples with a potential models with valid model file extension
                  and its corresponding checksum data.
        """
        model_list = list()

        # Get the list of all files from the directory.
        files = file_system_utils.list_dir(directory)
        for potential_model in files:

            # Ignore the file if not a supported model file.
            ext = os.path.splitext(potential_model)[1]
            if ext not in self.model_file_extensions:
                continue

            # Find model checksum.
            checksum = file_system_utils.md5(potential_model)
            if checksum == "":
                continue

            model_list.append((potential_model, checksum))

        return model_list

    def get_installed(self):
        """Return the list of all the models in the /opt/aws/deepracer/artifacts folder.

        Returns:
            list: List of directories which contain a checksum file.
        """
        list_of_directories = list()

        try:
            for dir_name in os.listdir(model_loader_config.MODEL_INSTALL_ROOT_DIRECTORY):

                model_directory = os.path.join(model_loader_config.MODEL_INSTALL_ROOT_DIRECTORY, dir_name)
                if not os.path.isdir(model_directory):
                    continue

                if not os.path.isfile(os.path.join(model_directory, model_loader_config.MODEL_CHECKSUM_FILE)):
                    continue

                if model_loader_config.ENABLE_GOLDEN_MODEL \
                   and (dir_name == model_loader_config.GOLDEN_MODEL_TARGET_NAME):
                    continue

                list_of_directories.append(model_directory)

        except Exception as ex:
            self.get_logger().error(f"Failed to list {model_loader_config.MODEL_INSTALL_ROOT_DIRECTORY}: {ex}")

        return list_of_directories

    def load_models(self, keyworded_args):
        """Wrapper function to trigger the transfer models function to copy the model artifacts
           from USB to /opt/aws/deepracer/artifacts folder, after verifying that its safe
           to copy the files.

        Args:
            keyworded_args (dict): Keyworded arguments passed to the function while scheduling.
        """
        # Wait until all threads finished.
        for model in self.models_in_progress:
            state = self.models_in_progress[model]
            state.wait_complete()

        # Transfer models from media to local temporary directories.
        with utility.AutoLock(self.progress_guard):
            self.transfer_models(keyworded_args)

            # Start the threads.
            for model in self.models_in_progress:
                state = self.models_in_progress[model]
                state.start_install()

    def call_blink_led_service(self):
        """Helper method to call the led_blink service.
        """
        self.led_blink_request.led_index = constants.LEDIndex.POWER_LED
        self.led_blink_request.color1 = constants.LEDColor.BLUE
        self.led_blink_request.color2 = constants.LEDColor.NO_COLOR
        self.led_blink_request.delay = 0.2
        self.led_blink_client.call_async(self.led_blink_request)

    def call_solid_led_service(self):
        """Helper method to call the led_solid service.
        """
        self.led_solid_request.led_index = constants.LEDIndex.POWER_LED
        self.led_solid_request.color = constants.LEDColor.BLUE
        self.led_solid_request.hold = 0.0
        self.led_solid_client.call_async(self.led_solid_request)

    def wipe_existing_models(self):
        """Helper method to delete existing models.
        """
        installed_list = self.get_installed()
        installed_count = len(installed_list)
        if installed_count == 0:
            self.get_logger().info("No installed models to wipe detected...")
        else:
            self.get_logger().info(f"Wiping {installed_count} installed model(s)...")

        for installed in installed_list:
            self.get_logger().info(f"  * {os.path.basename(installed)}")
            file_system_utils.remove_dir_tree(installed)

    def transfer_models(self, keyworded_args):
        """Main function to identify, extract and copy files from directory path
           passed as parameter to the /opt/aws/deeprace/artifacts folder.

        Args:
            keyworded_args (dict): Keyworded arguments passed to the function while scheduling.
        """
        self.call_blink_led_service()
        base_path = keyworded_args.get("path", "")
        name = keyworded_args.get("name", "")
        node_name = keyworded_args.get("node_name", None)

        self.get_logger().info("Reading the source directory...")

        # Get the list of archives from the source directory.
        search_path = os.path.join(base_path, name)
        list_of_archives = self.get_list_of_archives(search_path)

        self.models_in_progress = dict()

        # Remove possible old remaining directories.
        for old_temp_directory in glob.glob(os.path.join(constants.TEMP_DIRECTORY,
                                                         f"{model_loader_config.MODEL_TEMP_LEAF_DIRECTORY}-*")):
            file_system_utils.remove_dir_tree(old_temp_directory)

        # Remove all installed models if requested.
        if self.enable_model_wipe:
            self.wipe_existing_models()

        source_count = len(list_of_archives)
        if source_count == 0:
            self.get_logger().info("No new models to install detected...")
        else:
            self.get_logger().info(f"Processing {source_count} potential model(s)...")

        for archive_name in list_of_archives:
            self.get_logger().info(f"  * processing {archive_name}...")

            # Determine the model name.
            model_name = archive_name
            dot_pos = model_name.find(".")
            if dot_pos != -1:
                model_name = model_name[:dot_pos]

            # Create the temp directory.
            model_temp_directory = os.path.join(constants.TEMP_DIRECTORY,
                                                f"{model_loader_config.MODEL_TEMP_LEAF_DIRECTORY}-{model_name}")
            if os.path.isdir(model_temp_directory):
                self.get_logger().info(f"    ! ignoring model with duplicate name: {model_name}")
                continue

            if not file_system_utils.create_dir(model_temp_directory):
                continue

            # Extract the archive.
            archive_path = os.path.join(search_path, archive_name)
            if not self.extract_archive(archive_path, model_temp_directory):
                file_system_utils.remove_dir_tree(model_temp_directory)
                continue

            # Get the list of models.
            model_list = self.get_model_list(model_temp_directory)

            if len(model_list) == 0:
                file_system_utils.remove_dir_tree(model_temp_directory)
                self.get_logger().info("    ! no models found in the archive, ignoring")
                continue

            if len(model_list) > 1:
                file_system_utils.remove_dir_tree(model_temp_directory)
                self.get_logger().info("    ! unexpected: more than one models found in the archive,"
                                       " ignoring all of them")
                continue

            # Golden model?
            if model_loader_config.ENABLE_GOLDEN_MODEL \
               and (model_name == model_loader_config.GOLDEN_MODEL_SOURCE_NAME):
                self.get_logger().info("    golden model detected")
                model_name = model_loader_config.GOLDEN_MODEL_SOURCE_NAME

            # Intel model optimizer does not handle spaces in the path correctly, replace with underscores.
            if model_loader_config.REPLACE_MODEL_NAMESPACES:
                model_name = model_name.replace(" ", "_")

            # Extract model info.
            model_file_path, model_checksum = model_list[0]
            model_install_directory = os.path.join(model_loader_config.MODEL_INSTALL_ROOT_DIRECTORY, model_name)
            checksum_path = os.path.join(model_install_directory, model_loader_config.MODEL_CHECKSUM_FILE)

            # Verify the checksum.
            if model_checksum == file_system_utils.read_line(checksum_path).strip():
                self.get_logger().info("    model already installed, ignoring")
                file_system_utils.remove_dir_tree(model_temp_directory)
                continue

            # Add to the dictionary to be processed later.
            self.get_logger().info("    scheduling for installation")
            self.models_in_progress[model_name] = model_install_state.ModelInstallState(model_temp_directory,
                                                                                        model_file_path,
                                                                                        model_checksum,
                                                                                        model_install_directory,
                                                                                        self.model_optimizer_client,
                                                                                        self.get_logger())

        # Unmount the media.
        if node_name is not None:
            mount_point_mgr_request = USBMountPointManagerSrv.Request()
            mount_point_mgr_request.node_name = node_name
            mount_point_mgr_request.action = 0

        self.call_solid_led_service()

    def verify_model_ready(self, model_name):
        """Helper function to wait for model loading to complete if its not already,
           else verify if the model folder has a checksum file created.

        Args:
            model_name (str): Model folder name in /opt/aws/deepracer/artifacts folder.

        Returns:
            bool: True if the model is properly loaded/has checksum.txt in the model folder else False.
        """
        self.get_logger().info(f"Verifying readiness of model \"{model_name}\"...")

        try:
            with utility.AutoLock(self.progress_guard):
                state = self.models_in_progress[model_name]

            try:
                # Wait for the model to become ready.
                self.get_logger().info(f"Making sure \"{model_name}\" thread is complete...")
                state.wait_complete()

                # Return install status.
                ready = state.installed.isSet()

                if ready:
                    self.get_logger().info(f"Model \"{model_name}\" is successfully installed.")

                else:
                    self.get_logger().info(f"Model \"{model_name}\" installation failed.")

            except Exception as ex:
                self.get_logger().error(f"Failed to wait for installation of \"{model_name}\": {ex}")
                ready = False

        except Exception as ex:
            self.get_logger().info(f"Model \"{model_name}\" not being currently installed,"
                                   " checking existing installation...")

            model_install_directory = os.path.join(model_loader_config.MODEL_INSTALL_ROOT_DIRECTORY, model_name)
            if not os.path.isdir(model_install_directory):
                self.get_logger().info(f"Model \"{model_name}\" name is not recognized.")
                ready = False

            else:
                checksum_path = os.path.join(model_install_directory, model_loader_config.MODEL_CHECKSUM_FILE)
                checksum = file_system_utils.read_line(checksum_path).strip()

                ready = (checksum != "")
                if ready:
                    self.get_logger().info(f"Model \"{model_name}\" is installed.")

                else:
                    self.get_logger().info(f"Model \"{model_name}\" installation appears incomplete.")

        return ready

    def console_model_action(self, model_path, action):
        """Function to delete the model from model_path if action == 0 or else transfer the model
           from model_path to /opt/aws/deepracer/artifacts.

        Args:
            model_path (str): Path where the model is located.
            action (int): 0 to delete the model and 1 to transfer the model.

        Returns:
            str: Status(str) containing the details about the action execution.
        """
        self.get_logger().info(f"console_model_action...model_path : {model_path} action : {action}")

        if action == 0:
            # Delete the installed model
            status = file_system_utils.remove_dir_tree(model_path)
            return "done-delete" if status else "failure-on-delete"

        # Else upload the model to the DeepRacer device
        # Wait until all threads finished.
        for model in self.models_in_progress:
            state = self.models_in_progress[model]
            state.wait_complete()

        self.get_logger().info("consoleloadModels... copy from model directory  to local temporary directories")

        # Transfer models from model directoy to local temporary directories.
        with utility.AutoLock(self.progress_guard):
            self.transfer_models({"path": model_path, "name": "", "mount_point": None})

            # Start the threads.
            for model in self.models_in_progress:
                state = self.models_in_progress[model]
                state.start_install()

        # Wait till the model upload is completed to show the proper size
        for model in self.models_in_progress:
            state = self.models_in_progress[model]
            state.wait_complete()

        return "done-upload"


def main(args=None):
    rclpy.init(args=args)
    model_loader_node = ModelLoaderNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(model_loader_node, executor)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    model_loader_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
