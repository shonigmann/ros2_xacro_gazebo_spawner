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

"""Script used to spawn a robot defined by an xacro file in an arbitrary position using ROS2 launch."""

import argparse
import os
import xml.etree.ElementTree as ET
import xacro

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import rclpy
from scipy.spatial.transform import Rotation

import subprocess


def main():
    # Get input arguments from user
    parser = argparse.ArgumentParser(description='Spawn Robot into Gazebo with navigation2')
    parser.add_argument('-n', '--robot_name', type=str, default='robot',
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='robot',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-x', type=float, default=0,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-y', type=float, default=0,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-z', type=float, default=0,
                        help='the z component of the initial position [meters]')
    parser.add_argument('-R', type=float, default=0,
                        help='the initial robot yaw position [radians]')
    parser.add_argument('-P', type=float, default=0,
                        help='the initial robot yaw position [radians]')
    parser.add_argument('-Y', type=float, default=0,
                        help='the initial robot yaw position [radians]')
    parser.add_argument('-k', '--timeout', type=float, default=10.0,
                        help="Seconds to wait. Block until the future is complete if negative. Don't wait if 0.")
    parser.add_argument('-u', '--urdf', type=str,
                        help="the path to the robot's model file (urdf)")
    parser.add_argument('-a', '--xacro_args', type=str,
                        help="xacro arguments to set in the document call")

    args, unknown = parser.parse_known_args()

    # Start node
    rclpy.init()
    node = rclpy.create_node('entity_spawner')

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')

    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info('...connected!')

    node.get_logger().info('spawning `{}` on namespace `{}` at {}, {}, {}, {}, {}, {}'.format(
        args.robot_name, args.robot_namespace, args.x, args.y, args.z, args.R, args.P, args.Y))

    description_path = args.urdf

    # We need to remap the transform (/tf) topic so each robot has its own.
    # We do this by adding `ROS argument entries` to the urdf file for
    # each plugin broadcasting a transform. These argument entries provide the
    # remapping rule, i.e. /tf -> /<robot_id>/tf

    # Convert the xacro file to xml first, to apply remapping

    # Old method:
    # This doesn't allow for the necessary xacro arg inputs
    # root = ET.fromstring(xacro.process_file(description_path).toxml())

    # New method using xacro cli
    xml_string = subprocess.check_output('xacro ' + description_path + ' ' + args.xacro_args, shell=True)
    root = ET.fromstring(xml_string)

    for plugin in root.iter('plugin'):
        # TODO - figure out how to generalize this to all robots that would need it
        if 'diff_drive' in plugin.attrib.values():  # or 'gazebo_ros_control' in plugin.attrib.values():
            # The only plugin we care for now is 'diff_drive' which is
            # broadcasting a transform between`odom` and `base_footprint`
            # TODO - VERIFY WHEN USING NAMESPACES! WITHOUT NAMESPACE, THIS SEEMS TO BREAK THE PLUGIN
            if args.robot_namespace is not None and args.robot_namespace != '':
                ros_params = plugin.find('ros')
                ros_tf_remap = ET.SubElement(ros_params, 'remapping')
                ros_tf_remap.text = '/tf:=/' + args.robot_namespace + '/tf'
            break

    # Set data for requestRequest
    request = SpawnEntity.Request()
    request.name = args.robot_name
    request.xml = ET.tostring(root, encoding='unicode')

    request.robot_namespace = args.robot_namespace
    request.initial_pose.position.x = float(args.x)
    request.initial_pose.position.y = float(args.y)
    request.initial_pose.position.z = float(args.z)

    # CONVERT DESIRED YAW TO QUATERNION FOR INITIAL POSE:
    eul_r = args.R
    eul_p = args.P
    eul_y = args.Y

    rot = Rotation.from_euler('xyz', [eul_r, eul_p, eul_y], degrees=False)
    quat = rot.as_quat()

    request.initial_pose.orientation.x = quat[0]
    request.initial_pose.orientation.y = quat[1]
    request.initial_pose.orientation.z = quat[2]
    request.initial_pose.orientation.w = quat[3]

    node.get_logger().info('Sending service request to `/spawn_entity`')
    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info('Done! Shutting down node.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
