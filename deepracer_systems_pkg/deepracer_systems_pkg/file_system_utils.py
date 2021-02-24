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
import shutil
import hashlib

from deepracer_systems_pkg import constants


#########################################################################################
# File system functions.

def make_writable(filename):
    """Helper method to make the file writable.

    Args:
        filename (str): Path to the file.
    """
    try:
        os.chmod(filename, 0o666)
        os.chown(filename, constants.NOBODY_UID, constants.NOGROUP_GID)
    except Exception as ex:
        pass


def read_line(filename):
    """Helper method to open a file and read a line from it.

    Args:
        filename (str): Path to the file.

    Returns:
        str: Line read from the file.
    """
    try:
        with open(filename) as text_file:
            return text_file.readline()
    except Exception:
        return ""


def write_line(filename, line=""):
    """Helper method to open a file a write a line to it.

    Args:
        filename (str): Path to the file
        line (str, optional): Line to write to the file. Defaults to "".

    Returns:
        bool: True if successfully written else False.
    """
    try:
        with open(filename, "w") as text_file:
            print(line, file=text_file)
        return True
    except Exception as e:
        print(f"Error while writing: {e}")
        return False


def md5(filename):
    """Helper method to find the Md5 checksum of the file. 

    Args:
        filename (str): Path to the file.

    Returns:
        str: Hexdigest of the file.
    """
    try:
        hash_md5 = hashlib.md5()

        with open(filename, "rb") as in_file:
            for chunk in iter(lambda: in_file.read(4096), b""):
                hash_md5.update(chunk)

        return hash_md5.hexdigest()
    except Exception as ex:
        print(f"Failed to compute checksum of {filename}: {ex}")
        return ""


def create_dir(directory_name):
    """Helper method to create a directory.

    Args:
        directory_name (str): Path of the new directory to be created.

    Returns:
        bool: True if successfully created else False.
    """
    try:
        if not os.path.isdir(directory_name):
            os.makedirs(directory_name)
        return True
    except Exception as ex:
        return False


def remove_dir_tree(directory_name):
    """Helper method to remove a directory tree.

    Args:
        directory_name (str): Path of the directory tree to be removed.

    Returns:
        bool: True if successfully removed else False.
    """
    try:
        shutil.rmtree(directory_name)
        return True
    except Exception:
        return False


def remove_dir(directory_name):
    """Helper method to remove a directory.

    Args:
        directory_name (str): Path of the directory to be removed.

    Returns:
        bool: True if successfully removed else False.
    """

    try:
        os.rmdir(directory_name)
        return True
    except Exception:
        return False


def list_dir(directory_name):
    """Helper method to list a directory.

    Args:
        directory_name (str): Path of the directory to be listed.

    Returns:
        list: List of the files/folders in the directory.
    """

    file_list = list()
    for dirpath, dirnames, filenames in os.walk(directory_name):
        for name in filenames:
            file_list.append(os.path.join(dirpath, name))
    return file_list
