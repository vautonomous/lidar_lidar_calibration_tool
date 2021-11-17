# Copyright 2021 Leo Drive Teknoloji A.Ş.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import launch
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

import os

pcd_extractor_node_pkg_prefix = get_package_share_directory('pcd_extractor')
pcd_extractor_node_param_file = os.path.join(pcd_extractor_node_pkg_prefix,
                                                  'param/params.yaml')

def generate_launch_description():
    pcd_extractor_node = Node(
        package='pcd_extractor',
        executable='pcd_extractor',
        namespace='pcd_extractor',
        parameters=[pcd_extractor_node_param_file],
        output='screen'
    )

    return launch.LaunchDescription([pcd_extractor_node])
