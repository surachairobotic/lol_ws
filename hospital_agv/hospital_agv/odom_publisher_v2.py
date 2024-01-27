import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Quaternion, TransformStamped
import tf2_ros
import serial, time, pygame, sys, threading, math
import numpy as np

class OdomPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')
                
        #self.wheel = Controller('/dev/ttyUSB0')
        self.L = 0.7
        self.current_cmd = Twist()
        self.cmdCalOdom = [0,0]
        self.cmdL = self.cmdR = 0
        self.currentOdom = [0,0]
        
        self.prev_t = time.time()
        self.prev_pose = Pose()
        self.prev_pose.position.x = 0.0
        self.prev_pose.position.y = 0.0
        self.prev_pose.position.z = 0.0
        self.prev_pose.orientation.x = 0.0
        self.prev_pose.orientation.y = 0.0
        self.prev_pose.orientation.z = 0.0
        self.prev_pose.orientation.w = 1.0

        self.subscription = self.create_subscription(Twist, '/robot_vel', self.robot_vel_callback, 10)
        self.subscription  # prevent unused variable warning

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        #self.vel_pub = self.create_publisher(Twist, '/cmd_vel', self.control_callback, 10)
        self.timer_odom = self.create_timer(1.0/100.0, self.publish_odom)
        self.tOdom = time.time()
        self.logOdom = open('odom_publish_time.txt', 'w')

        #self.timer_readWheel = self.create_timer(1/100, self.wheel.readInfo)
        #self.timer_controlWheel = self.create_timer(1/100, self.controlWheel)
        #self.timer_printInfo = self.create_timer(1/100, self.printInfo)
        
        # Create TF2 broadcaster for transforms
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
	
        
    def robot_vel_callback(self, msg):
        #print(msg)
        self.current_cmd = msg

    def publish_odom(self):
        tProcess = time.time()
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.twist.twist = self.current_cmd
        
        x = self.prev_pose.position.x
        y = self.prev_pose.position.y
        yaw = self.get_yaw_from_quaternion(self.prev_pose.orientation)
        vx = msg.twist.twist.linear.x
        #vy = msg.twist.twist.linear.y
        v_yaw = msg.twist.twist.angular.z

        dt = time.time()-self.prev_t
        self.prev_t = time.time()
        #print("dt : ", dt)

        new_x = x + (vx * math.cos(yaw)) * dt
        new_y = y + (vx * math.sin(yaw)) * dt
        new_yaw = yaw + v_yaw * dt

        msg.pose.pose.position.x = new_x
        msg.pose.pose.position.y = new_y
        msg.pose.pose.position.z = 0.0        
        msg.pose.pose.orientation = self.yaw_to_quaternion(new_yaw)
        self.prev_pose = msg.pose.pose

	# Publish TF transforms
        base_link_transform = TransformStamped()
        base_link_transform.header.stamp = self.get_clock().now().to_msg()
        base_link_transform.header.frame_id = 'odom'  
        base_link_transform.child_frame_id = 'base_footprint'  
        base_link_transform.transform.translation.x = new_x
        base_link_transform.transform.translation.y = new_y
        base_link_transform.transform.translation.z = 0.0
        base_link_transform.transform.rotation = self.yaw_to_quaternion(new_yaw)
        self.tf_broadcaster.sendTransform(base_link_transform)

        
        # Publish left wheel TF
        left_wheel_transform = TransformStamped()
        left_wheel_transform.header.stamp = self.get_clock().now().to_msg()
        left_wheel_transform.header.frame_id = 'base_link'
        left_wheel_transform.child_frame_id = 'wheel_left_link'
        left_wheel_transform.transform.translation.x = 0.14771
        left_wheel_transform.transform.translation.y = -0.3482
        left_wheel_transform.transform.translation.z = -0.0703
        left_wheel_transform.transform.rotation.x = 0.0
        left_wheel_transform.transform.rotation.y = 1.0
        left_wheel_transform.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(left_wheel_transform)
        
        # Publish right wheel TF
        right_wheel_transform = TransformStamped()
        right_wheel_transform.header.stamp = self.get_clock().now().to_msg()
        right_wheel_transform.header.frame_id = 'base_link'
        right_wheel_transform.child_frame_id = 'wheel_right_link'
        right_wheel_transform.transform.translation.x = 0.14771
        right_wheel_transform.transform.translation.y = 0.35684
        right_wheel_transform.transform.translation.z = -0.0703
        right_wheel_transform.transform.rotation.x = 0.0
        right_wheel_transform.transform.rotation.y = 1.0 
        right_wheel_transform.transform.rotation.z = 0.0 

        self.tf_broadcaster.sendTransform(right_wheel_transform)
        
        self.odom_pub.publish(msg)
        durations = time.time()-self.tOdom
        dProcess = time.time()-tProcess
        self.logOdom.write(str(durations) + ', \t' + str(dProcess) + '\r\n')
        self.tOdom = time.time()
        #print("tOdom : ", time.time()-tOdom)

    def yaw_to_quaternion(self, yaw):
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion

    def get_position_and_orientation(self, pose, dt):

        x = pose.pose.position.x
        y = pose.pose.position.y
        yaw = get_yaw_from_quaternion(pose.pose.orientation)
        vx = twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y
        v_yaw = odom_msg.twist.twist.angular.z
        
        dt = 0.1  # Time interval in seconds
        new_x = x + (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        new_y = y + (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        new_yaw = yaw + v_yaw * dt
        
        return (new_x, new_y, new_yaw)
    
    def get_yaw_from_quaternion(self, quaternion):
        #print(type(quaternion))
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw


def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



