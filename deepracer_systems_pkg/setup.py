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

from setuptools import setup
import os
import glob

package_name = 'deepracer_systems_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name,
              package_name + ".software_update_module",
              package_name + ".model_loader_module",
              package_name + ".otg_module",
              package_name + ".network_monitor_module",
              package_name + ".deepracer_systems_scripts_module"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), ['launch/deepracer_systems_pkg_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AWS DeepRacer',
    maintainer_email='aws-deepracer@amazon.com',
    description=('This package contains the DeepRacer system level packages responsible '
                 'for software update system, managing reinforcement learning models, '
                 'WiFi connection, ethernet over USB feature and running dependency script commands.'),
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'software_update_node = deepracer_systems_pkg.software_update_module.software_update_node:main',
            'model_loader_node = deepracer_systems_pkg.model_loader_module.model_loader_node:main',
            'otg_control_node = deepracer_systems_pkg.otg_module.otg_control_node:main',
            'network_monitor_node = deepracer_systems_pkg.network_monitor_module.network_monitor_node:main',
            'deepracer_systems_scripts_node = deepracer_systems_pkg.deepracer_systems_scripts_module.deepracer_systems_scripts_node:main'
        ],
    },
)
