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
model_install_state.py

This module creates the ModelInstallState class which is responsible for creating the
model directory in the /opt/aws/deepracer/artifacts folder and copying the verified files
to it in a seperate thread. If the model_optimizer_client is passed, the class handles
calling model optimizer service with the relevant details to optimize the model.
"""

import os
import shutil
import threading
import time

from deepracer_interfaces_pkg.srv import ModelOptimizeSrv
from deepracer_systems_pkg import (constants,
                                   file_system_utils)
from deepracer_systems_pkg.model_loader_module import (model_loader_config,
                                                       model_metadata_file_utils)


#########################################################################################
# Model loader class.


class ModelInstallState:
    """Class responsible for copying the model files to install directory
       and optimizing them in a seperate thread.
    """
    def __init__(self,
                 temp_directory,
                 temp_file_path,
                 checksum,
                 install_directory,
                 model_optimizer_client,
                 logger):
        """Create the ModelInstallState object.

        Args:
            temp_directory (str): Temporary directory where the model contents are extracted.
            temp_file_path (str): Temporary filename of the model file.
            checksum (str): Md5 hexdigest of the model file.
            install_directory (str): Target directory where the model files are to be copied.
            model_optimizer_client (rclpy.client.Client): Model optimizer client object or None.
            logger (rclpy.rclpy.impl.rcutils_logger.RcutilsLogger): Logger object of the model_loader_node.
        """
        self.thread = threading.Thread(target=self.install_thread)
        self.installed = threading.Event()
        self.name = os.path.basename(install_directory)
        self.temp_directory = temp_directory
        self.temp_file_path = temp_file_path
        self.checksum = checksum
        self.install_directory = install_directory
        self.model_optimizer_client = model_optimizer_client
        self.logger = logger

    def start_install(self):
        """Start the install thread's activity.
        """
        self.thread.start()

    def wait_complete(self):
        """Blocking call to wait till the installation of the model is complete.
        """
        self.thread.join()

    def install_thread(self):
        """Main function to create the target directory, copy the model files and
           call the model optimizer if the client is passed as parameter to the class.
        """
        successfully_installed = False

        self.logger.info(f"Installing {self.name}...")

        while True:
            # Recreate model directory.
            file_system_utils.remove_dir_tree(self.install_directory)
            if not file_system_utils.create_dir(self.install_directory):
                break

            # Copy the model in place.
            try:
                if model_loader_config.USE_FIXED_TARGET_FILENAME:
                    model_ext = os.path.splitext(self.temp_file_path)[1]
                    target_name = \
                        os.path.join(self.install_directory,
                                     model_loader_config.TARGET_MODEL_FILENAME + model_ext)
                else:
                    target_name = self.install_directory

                self.logger.info(f"Copy the model in place: {self.temp_file_path} {target_name}")
                shutil.copy(self.temp_file_path, target_name)
            except Exception as ex:
                self.logger.error(f"Failed to install model {self.temp_file_path}: {ex}")
                break

            metadata_path = os.path.join(self.temp_directory,
                                         model_loader_config.MODEL_METADATA_NAME)
            # The navigation node assumes that the only json file transferred to the model directory
            # is the meta data file, if we add other json files to the model directory we need to
            # modify the navigation node accordingly.
            if os.path.isfile(metadata_path):
                shutil.copy(metadata_path, self.install_directory)
            else:
                self.logger.info("No model meta data file found")

            # Call model optimizer.
            if self.model_optimizer_client is not None:
                # Get the model file name.
                model_filename = os.path.basename(self.temp_file_path)
                model_filename = os.path.splitext(model_filename)[0]

                # Consruct the model file name for the optimizer.
                relative_model_name = os.path.join(self.name, model_filename)

                self.logger.info(f"calling optimizer for {relative_model_name}...")
                model_metatdata_file_path = os.path.join(self.install_directory,
                                                         model_loader_config.MODEL_METADATA_NAME)
                # Read the content of the model_metadata.json.
                self.logger.info(f"read model_metadata_file from {model_metatdata_file_path}...")
                err_code, err_msg, model_metadata_content = \
                    model_metadata_file_utils.read_model_metadata_file(model_metatdata_file_path)
                if err_code != 0:
                    self.logger.error("Error while reading from "
                                      f"{model_loader_config.MODEL_METADATA_NAME}: {err_msg}")
                    break

                # Get the sensor information of the model from the model_metadata.json.
                err_code,  err_msg, model_metadata_sensors = \
                    model_metadata_file_utils.get_sensors(model_metadata_content)
                if err_code != 0:
                    self.logger.error("Error while getting sensor names from "
                                      f"{model_metatdata_file_path}: {err_msg}")
                    break
                self.logger.info("Sensor names read from "
                                 f"{model_metatdata_file_path}: {model_metadata_sensors}")

                # Get the training algorithm information of the model from the model_metadata.json.
                err_code,  err_msg, training_algorithm = \
                    model_metadata_file_utils.get_training_algorithm(model_metadata_content)
                if err_code != 0:
                    self.logger.error(f"Error while getting training algorithm from "
                                      f"{model_metatdata_file_path}: {err_msg}")
                    break
                self.logger.info("Training algorithm read from "
                                 f"{model_metatdata_file_path}: {training_algorithm}")

                # Get the LiDAR configuration if passed for the model from the model_metadata.json.
                err_code,  err_msg, model_lidar_config = \
                    model_metadata_file_utils.load_lidar_configuration(model_metadata_sensors,
                                                                       model_metadata_content)
                if err_code != 0:
                    self.logger.error("Error while getting LiDAR configuration from "
                                      f"{model_metatdata_file_path}: {err_msg}")
                    break
                self.logger.info("LiDAR configuration read from "
                                 f"{model_metatdata_file_path}: {model_lidar_config}")

                # Create the ModelOptimizeSrv request object.
                model_optimizer_req = ModelOptimizeSrv.Request()
                model_optimizer_req.model_name = relative_model_name
                model_optimizer_req.model_metadata_sensors = model_metadata_sensors
                model_optimizer_req.training_algorithm = training_algorithm
                model_optimizer_req.img_format = "BGR"
                model_optimizer_req.width = 160
                model_optimizer_req.height = 120
                model_optimizer_req.num_channels = 1
                model_optimizer_req.lidar_channels = \
                    model_lidar_config[constants.ModelMetadataKeys.NUM_LIDAR_SECTORS.value]
                model_optimizer_req.platform = 1

                # Call the model optimizer service.
                future = self.model_optimizer_client.call_async(model_optimizer_req)
                elapsed_time = 0
                timeout = 300
                while not future.done():
                    self.logger.info("Service call not finished: "
                                     f"{self.model_optimizer_client.srv_name}")
                    time.sleep(2)
                    elapsed_time += 2
                    if elapsed_time >= timeout:
                        self.logger.info("Service call was not completed before timeout:"
                                         f" {self.model_optimizer_client.srv_name} {timeout}")
                        self.model_optimizer_client.remove_pending_request(future)
                        break

                # Response received from the model optimizer service.
                response = future.result()
                if response.error:
                    self.logger.error(f"Model optimizer for {self.name} failed.")
                    break

            # Write new checksum file.
            checksum_path = os.path.join(self.install_directory,
                                         model_loader_config.MODEL_CHECKSUM_FILE)
            if not file_system_utils.write_line(checksum_path, self.checksum):
                self.logger.info(f"Failed to write checksum {checksum_path}")
                break

            # Success.
            successfully_installed = True
            break

        if successfully_installed:
            self.logger.info(f"Installation of {self.name} complete.")
            self.installed.set()
        else:
            self.logger.error(f"Installation of {self.name} failed.")
            file_system_utils.remove_dir_tree(self.install_directory)

        file_system_utils.remove_dir_tree(self.temp_directory)
