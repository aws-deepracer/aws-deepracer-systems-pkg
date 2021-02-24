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

#########################################################################################
# GPG wrappers.


def verify_package(name):
    """Helper method to verify if the package is trusted.

    Args:
        name (str): Name of the package (*.deb.gpg).

    Returns:
        [type]: [description]
    """
    p = subprocess.Popen(["gpg", "--keyring", "/etc/apt/trusted.gpg",
                                 "--verify", name],
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    p.communicate()
    return (p.returncode == 0)


def decrypt_package(name, decrypted_name):
    """Helper method to decrypt the package.

    Args:
        name (str): Path to the package.
        decrypted_name (str): Destination path of the decrypted package.

    Returns:
        bool: True if successfully decrypted else False.
    """
    p = subprocess.Popen(["gpg", "--batch",
                                 "--keyring", "/etc/apt/trusted.gpg",
                                 "--output", decrypted_name,
                                 "--decrypt", name],
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    p.communicate()
    return (p.returncode == 0)


def install_debian(package_name):
    """Helper method to install debian packages.

    Args:
        package_name (str): Path to the package.

    Returns:
        bool: True if successfully installaed else False.
    """
    p = subprocess.Popen(["dpkg", "--install", package_name],
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    p.communicate()
    return (p.returncode == 0)
