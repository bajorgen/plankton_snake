#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Accel
import math

def quatToRPY(quat):
    w = quat.w
    x = quat.x
    y = quat.y
    z = quat.z
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(2*(w*y - z*x))
    yaw = math.atan2(2*(w*z + z*y), 1- 2*(y*y + z*z))
    return [roll*180/math.pi, pitch*180/math.pi, yaw*180/math.pi]


class MinervaOdometry(Node):
    def __init__(self):
        super().__init__('odometry')
        self.subscription = self.create_subscription(
            Odometry,
            '/minerva/pose_gt',
            self.listener_callback,
            10)


        self.thrust_subscription0 = self.create_subscription(WrenchStamped, '/minerva/thrusters/id_0/thrust_wrench', self.thruster0_callback, 10)
        self.thrust_subscription1 = self.create_subscription(WrenchStamped, '/minerva/thrusters/id_1/thrust_wrench', self.thruster1_callback, 10)
        self.thrust_subscription2 = self.create_subscription(WrenchStamped, '/minerva/thrusters/id_2/thrust_wrench', self.thruster2_callback, 10)
        self.thrust_subscription3 = self.create_subscription(WrenchStamped, '/minerva/thrusters/id_3/thrust_wrench', self.thruster3_callback, 10)
        self.thrust_subscription4 = self.create_subscription(WrenchStamped, '/minerva/thrusters/id_4/thrust_wrench', self.thruster4_callback, 10)
        self.thrust_subscription5 = self.create_subscription(WrenchStamped, '/minerva/thrusters/id_5/thrust_wrench', self.thruster5_callback, 10)
        self.thrust_subscription6 = self.create_subscription(WrenchStamped, '/minerva/thrusters/id_6/thrust_wrench', self.thruster6_callback, 10)
        self.thrust_subscription7 = self.create_subscription(WrenchStamped, '/minerva/thrusters/id_7/thrust_wrench', self.thruster7_callback, 10)
        self.thrust_subscription8 = self.create_subscription(WrenchStamped, '/minerva/thrusters/id_8/thrust_wrench', self.thruster8_callback, 10)
        self.thrust_subscription9 = self.create_subscription(WrenchStamped, '/minerva/thrusters/id_9/thrust_wrench', self.thruster9_callback, 10)
        self.thrust_subscription10 = self.create_subscription(WrenchStamped, '/minerva/thrusters/id_10/thrust_wrench', self.thruster10_callback, 10)
        #self.thrust_subscription11 = self.create_subscription(WrenchStamped, '/minerva/thrusters/id_11/thrust_wrench', self.thruster11_callback, 10)
        
        self.subscription  # prevent unused variable warning
        self.thrust_subscription0
        self.thrust_subscription1
        self.thrust_subscription2
        self.thrust_subscription3
        self.thrust_subscription4
        self.thrust_subscription5
        self.thrust_subscription6
        self.thrust_subscription7
        self.thrust_subscription8
        self.thrust_subscription9
        self.thrust_subscription10
        #self.thrust_subscription11

        #initialize thruster forces
        self.thruster0 = 0
        self.thruster1 = 0
        self.thruster2 = 0
        self.thruster3 = 0
        self.thruster4 = 0
        self.thruster5 = 0
        self.thruster6 = 0
        self.thruster7 = 0
        self.thruster8 = 0
        self.thruster9 = 0
        self.thruster10 = 0
        self.thruster11 = 0

    def thruster0_callback(self, msg):
        self.thruster0 = math.sqrt(msg.wrench.force.x **2 + msg.wrench.force.y **2) * math.copysign(1, msg.wrench.force.x)
    def thruster1_callback(self, msg):
        self.thruster1 = math.sqrt(msg.wrench.force.x **2 + msg.wrench.force.y **2) * math.copysign(1, msg.wrench.force.x)
    def thruster2_callback(self, msg):
        self.thruster2 = math.sqrt(msg.wrench.force.x **2 + msg.wrench.force.y **2) * math.copysign(1, msg.wrench.force.x)
    def thruster3_callback(self, msg):
        self.thruster3 = math.sqrt(msg.wrench.force.x **2 + msg.wrench.force.y **2) * math.copysign(1, msg.wrench.force.x)
    def thruster4_callback(self, msg):
        self.thruster4 = math.sqrt(msg.wrench.force.x **2 + msg.wrench.force.y **2) * math.copysign(1, msg.wrench.force.x)
    def thruster5_callback(self, msg):
        self.thruster5 = math.sqrt(msg.wrench.force.x **2 + msg.wrench.force.y **2) * math.copysign(1, msg.wrench.force.x)
    def thruster6_callback(self, msg):
        self.thruster6 = math.sqrt(msg.wrench.force.x **2 + msg.wrench.force.y **2) * math.copysign(1, msg.wrench.force.x)

    def thruster7_callback(self, msg):
        self.thruster8 = math.sqrt(msg.wrench.force.x **2 + msg.wrench.force.y **2) * math.copysign(1, msg.wrench.force.x)
    def thruster8_callback(self, msg):
        self.thruster9 = math.sqrt(msg.wrench.force.x **2 + msg.wrench.force.y **2) * math.copysign(1, msg.wrench.force.x)
    def thruster9_callback(self, msg):
        self.thruster10 = math.sqrt(msg.wrench.force.x **2 + msg.wrench.force.y **2) * math.copysign(1, msg.wrench.force.x)
    def thruster10_callback(self, msg):
        self.thruster11 = math.sqrt(msg.wrench.force.x **2 + msg.wrench.force.y **2) * math.copysign(1, msg.wrench.force.x)
    #def thruster11_callback(self, msg):
    #    self.thruster11 = math.sqrt(msg.wrench.force.x **2 + msg.wrench.force.y **2) * math.copysign(1, msg.wrench.force.x)
    



    def listener_callback(self, msg):
        rpyVector = quatToRPY(msg.pose.pose.orientation)
        x = round(msg.pose.pose.position.x, 4)
        y = round(msg.pose.pose.position.y, 4)
        depth = round(-msg.pose.pose.position.z, 4)
        roll = round(rpyVector[0], 4)
        pitch = round(rpyVector[1], 4)
        yaw = round(rpyVector[2], 4)

        x_dot = round(msg.twist.twist.linear.x, 4)
        y_dot = round(msg.twist.twist.linear.y, 4)
        depth_dot = round(-msg.twist.twist.linear.z, 4)
        roll_dot = round(msg.twist.twist.angular.x, 4)
        pitch_dot = round(msg.twist.twist.angular.y, 4)
        yaw_dot = round(-msg.twist.twist.angular.z, 4)

        time = (msg.header.stamp.sec + msg.header.stamp.nanosec/1000000000)

        self.get_logger().info('\n'
        +'x: %s' % round(msg.pose.pose.position.x, 4)
        +'\ny: %s' % round(msg.pose.pose.position.y, 4)
        +'\nDepth: %s' % round(-msg.pose.pose.position.z, 4)
        +'\nx_dot: %s' % round(msg.twist.twist.linear.x, 4)
        +'\ny_dot: %s' % round(msg.twist.twist.linear.y, 4)
        +'\nDepth_dot: %s' % round(-msg.twist.twist.linear.z, 4)
        +'\nx of quaternion: %s' % round(msg.pose.pose.orientation.x, 7)
        +'\ny of quaternion: %s' % round(msg.pose.pose.orientation.y, 7)
        +'\nz of quaternion: %s' % round(msg.pose.pose.orientation.z, 7)
        +'\nw of quaternion: %s' % round(msg.pose.pose.orientation.w, 7)
        +'\nRoll acceleration: %s' % round(msg.twist.twist.angular.x, 4)
        +'\nPitch acceleration: %s' % round(msg.twist.twist.angular.y, 4)
        +'\nYaw acceleration: %s' % round(msg.twist.twist.angular.z, 4)
        +'\nRoll: %s' % round(rpyVector[0], 4)
        +'\nPitch: %s' % round(rpyVector[1], 4)
        +'\nYaw: %s' % round(rpyVector[2], 4)
        +'\nTime: %s' % (msg.header.stamp.sec + msg.header.stamp.nanosec/1000000000)
        )

        #Append to file
        with open('s_heave.txt', 'a') as f:
            f.write('%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n' % (time, x, y, depth, self.thruster0, self.thruster1, self.thruster2, self.thruster3, 
                self.thruster4, self.thruster5, self.thruster6, self.thruster7, self.thruster8, self.thruster9, self.thruster10, self.thruster11, roll, pitch, yaw, x_dot, y_dot, depth_dot,
                roll_dot, pitch_dot, yaw_dot))



def main(args=None):

    #Wipe file from prev simulation
    f = open('s_heave.txt', 'r+')
    f.seek(0)
    f.truncate()
    f.close()

    rclpy.init()

    try:
        odometry_sub = MinervaOdometry()
        rclpy.spin(odometry_sub)

    except Exception as e:
        print('OdometryModule::Exception ' + str(e))
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    print('Odometry node shutting down')


if __name__ == '__main__':
    main()
