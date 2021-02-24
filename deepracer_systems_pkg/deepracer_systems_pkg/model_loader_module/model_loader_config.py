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


#########################################################################################
# Model loading.

VERIFY_MODEL_READY_SERVICE_NAME = "verify_model_ready"
CONSOLE_MODEL_ACTION_SERVICE_NAME = "console_model_action"

ENABLE_MODEL_OPTIMIZER = False
MODEL_SOURCE_LEAF_DIRECTORY = "models"
MODEL_TEMP_LEAF_DIRECTORY = "aws-temp-model"
MODEL_INSTALL_ROOT_DIRECTORY = \
    os.path.join(os.sep, "opt", "aws", "deepracer", "artifacts")
MODEL_CHECKSUM_FILE = "checksum.txt"
# If this file name is changed on the service it must be changed here too
MODEL_METADATA_NAME = "model_metadata.json"

# If ENABLE_MODEL_WIPE is True, installed models will be deleted before the
# new ones are installed. The golden model is preserved.
ENABLE_MODEL_WIPE = True

ENABLE_GOLDEN_MODEL = True
GOLDEN_MODEL_SOURCE_NAME = "aws-golden-model-for-reinvent"
GOLDEN_MODEL_TARGET_NAME = "AWS Sample"

REPLACE_MODEL_NAMESPACES = True

# Use a fixed model name when copying to /opt/aws/deepracer/artifacts/<<model_name>>/model.pb
USE_FIXED_TARGET_FILENAME = True
TARGET_MODEL_FILENAME = "model"

SCHEDULE_MODEL_LOADER_CB = "schedule_model_loader"

# model_optimizer_pkg
MODEL_OPTIMIZER_PKG_NS = "/model_optimizer_pkg"
MODEL_OPTIMIZER_SERVER_SERVICE = f"{MODEL_OPTIMIZER_PKG_NS}/model_optimizer_server"
