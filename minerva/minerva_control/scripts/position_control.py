#!/usr/bin/env python3
# Copyright (c) 2020 The Plankton Authors.
# All rights reserved.
#
# This source code is derived from UUV Simulator
# (https://github.com/uuvsimulator/uuv_simulator)
# Copyright (c) 2016-2019 The UUV Simulator Authors
# licensed under the Apache license, Version 2.0
# cf. 3rd-party-licenses.txt file in the root directory of this source tree.
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

import numpy
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import geometry_msgs.msg as geometry_msgs
from nav_msgs.msg import Odometry

# Modules included in this package
from PID import PIDRegulator
import tf_quaternion.transformations as transf
from plankton_utils.time import time_in_float_sec_from_msg
from plankton_utils.time import is_sim_time


class PositionControllerNode(Node):
    def __init__(self, name, **kwargs):
        print('PositionControllerNode: initializing node')
        super().__init__(name, **kwargs)

        self.config = {}

        self.pos_des = numpy.zeros(3)
        self.rot_des = numpy.zeros(4)

        # Initialize pids with default parameters
        self.pid_rot = PIDRegulator(1, 0, 0, 1)
        self.pid_pos = PIDRegulator(1, 0, 0, 1)

        #Declared parameters are overriden with yaml values
        self._declare_and_fill_map("pos_p", 1., "p component of pid for position.", self.config)
        self._declare_and_fill_map("pos_i", 0.0, "i component of pid for position.", self.config)
        self._declare_and_fill_map("pos_d", 0., "d component of pid for position.", self.config)
        self._declare_and_fill_map("pos_sat", 1., "saturation of pid for position.", self.config)

        self._declare_and_fill_map("rot_p", 1., "p component of pid for rotation.", self.config)
        self._declare_and_fill_map("rot_i", 0.0, "i component of pid for rotation.", self.config)
        self._declare_and_fill_map("rot_d", 0.0, "d component of pid for rotation.", self.config)
        self._declare_and_fill_map("rot_sat", 3.0, "saturation of pid for rotation.", self.config)
        
        self.set_parameters_callback(self.callback_params)
        
        self.create_pids(self.config)

        # ROS infrastructure
        self.sub_cmd_vel = self.create_subscription(geometry_msgs.Pose, 'cmd_pos', self.cmd_pos_callback, 10)
        self.sub_odometry = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.pub_cmd_accel = self.create_publisher( geometry_msgs.Accel, 'cmd_accel', 10)

    #==============================================================================
    def cmd_pos_callback(self, msg):
        """Handle updated set velocity callback."""
        # Just store the desired velocity. The actual control runs on odometry callbacks
        v_p = msg.position
        v_o = msg.orientation
        self.pos_des = numpy.array([v_p.x, v_p.y, v_p.z])
        self.rot_des = numpy.array([v_o.x, v_o.y, v_o.z v_o.w])

    #==============================================================================
    def odometry_callback(self, msg):
        """Handle updated measured velocity callback."""
        if not bool(self.config):
            return

        pos = msg.pose.pose.position
        rot = msg.pose.pose.orientation
        position = numpy.array([pos.x, pos.y, pos.z])
        rotation = numpy.array([rot.x, rot.y, rot.z rot.w])

        # Compute compute control output:
        t = time_in_float_sec_from_msg(msg.header.stamp)
        
        e_position = (self.pos_des - position)
        e_rotation = (self.rot_des - rotation)
        
        a_linear = self.pid_pos.regulate(e_position, t)
        a_angular = self.pid_rot.regulate(e_rotation, t)

        # Convert and publish accel. command:
        cmd_accel = geometry_msgs.Accel()
        cmd_accel.linear = geometry_msgs.Vector3(x=a_linear[0], y=a_linear[1], z=a_linear[2])
        cmd_accel.angular = geometry_msgs.Vector3(x=a_angular[0], y=a_angular[1], z=a_angular[2])
        self.pub_cmd_accel.publish(cmd_accel)

    #==============================================================================
    def callback_params(self, data):
        for parameter in data:
            self.config[parameter.name] = parameter.value
        
        # config has changed, reset PID controllers
        self.create_pids(self.config)

        self.get_logger().warn("Parameters dynamically changed...")
        return SetParametersResult(successful=True)

    #==============================================================================
    def create_pids(self, config):
        self.pid_linear = PIDRegulator(
            config['pos_p'], config['pos_i'], config['pos_d'], config['pos_sat'])
        self.pid_angular = PIDRegulator(
            config['rot_p'], config['rot_i'], config['rot_d'], config['rot_sat'])

    #==============================================================================
    def _declare_and_fill_map(self, key, default_value, description, map):
        param = self.declare_parameter(key, default_value, ParameterDescriptor(description=description))
        map[key] = param.value


#==============================================================================
def main():
    print('Starting PositionControl.py')
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        node = PositionControllerNode('position_control', parameter_overrides=[sim_time_param])
        rclpy.spin(node)
    except Exception as e:
        print('Caught exception: ' + str(e))
    finally:
        rclpy.shutdown()
    print('Exiting')


#==============================================================================
if __name__ == '__main__':
    main()