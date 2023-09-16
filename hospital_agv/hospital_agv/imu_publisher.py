import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, qos_profile_sensor_data

import serial, threading, math
import numpy as np

class IMUPublisher():

    def __init__(self, _node):
        self.node = _node
        
        self.imu = Imu()
        self.imu.header.frame_id = 'imu_link'
        self.imu.orientation.w = 1.0
        self.imu.orientation_covariance[0] = 1.0
        self.imu.orientation_covariance[3] = 1.0
        self.imu.orientation_covariance[6] = 1.0
        #self.imu.angular_velocity.x = 0.1
        #self.imu.linear_acceleration.y = 9.8
        
        self.mag = MagneticField()
        self.mag.header.frame_id = 'imu_link'
        
        self.publisher_imu = self.node.create_publisher(Imu, '/imu/imu', 10)
        self.publisher_mag = self.node.create_publisher(MagneticField, '/imu/mag', 10)
        timer_period = 0.01  # seconds
        self.timer = self.node.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200)
        self.recvThread = threading.Thread(target=self.recvData)
        self.recvThread.start()

    def recvData(self):
        while True:
            #try:
            line = self.ser.readline()
            #line = self.ser.readline()
            line = line.decode()
            line = list(filter(None, line.split('|')))
            #print(line)
            if len(line) != 5:
                continue
            
            for x in line[:-1]:
                mode = x[0]
                if x[1] == 'r':
                    break
                d = x[1:].split(',')
                d = [float(k) for k in d]
                print(d)
                if mode == 'O':
                    quat = self.euler_to_quaternion(d)
                    self.imu.orientation.x = quat[0]
                    self.imu.orientation.y = quat[1]
                    self.imu.orientation.z = quat[2]
                    self.imu.orientation.w = quat[3]
                elif mode == 'G':
                    self.imu.angular_velocity.x = d[0]
                    self.imu.angular_velocity.y = d[1]
                    self.imu.angular_velocity.z = d[2]
                elif mode == 'L':
                    self.imu.linear_acceleration.x = d[0]
                    self.imu.linear_acceleration.y = d[1]
                    self.imu.linear_acceleration.z = d[2]
                elif mode == 'M':
                    self.mag.magnetic_field.x = d[0]
                    self.mag.magnetic_field.y = d[1]
                    self.mag.magnetic_field.z = d[2]

            '''
            except Exception as e:
                print(e)
            '''

    def euler_to_quaternion(self, r):
        (yaw, pitch, roll) = ((r[0]-180)*math.pi/180, r[2]/180*math.pi, r[1]/180*math.pi)
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

    def timer_callback(self):
        self.publisher_imu.publish(self.imu)
        self.publisher_mag.publish(self.mag)
        if self.i%100 == 0:
            self.node.get_logger().info('publish_imu: "%s"' % self.i)
        self.i += 1

class OdomPublisher():
    def __init__(self, _node):
        self.node = _node
        self.odom_pub = self.node.create_publisher(Odometry, '/agv/odom', 10)
        self.timer = self.node.create_timer(1/100, self.publish_odom)
        self.i=0

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0
        self.odom_pub.publish(msg)
        if self.i%100 == 0:
            self.node.get_logger().info('publish_odom: "%s"' % self.i)
        self.i += 1

class MultiPublisher(Node):
    def __init__(self):
        super().__init__('multi_publisher')
    
    def init(self):
        self.imu_publisher = IMUPublisher(self)
        #self.odom_publisher = OdomPublisher(self)


def main(args=None):
    rclpy.init(args=args)
    
    nh = Node("imu_publisher")

    publisher = IMUPublisher(nh)
    #publisher.init()

    rclpy.spin(publisher.node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
