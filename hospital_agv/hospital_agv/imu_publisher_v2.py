from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
from rclpy.node import Node
import serial
import rclpy
import math
import json

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200

        self.publisher_mag = self.create_publisher(MagneticField, '/mag', 10)
        self.publisher_imu = self.create_publisher(Imu, '/imu', 10)

        self.timer_mag = self.create_timer(1/100, self.publish_magnet)
        self.timer_imu = self.create_timer(1/100, self.publish_imu)
        self.create_timer(1/100, self.read_serial_data)

        self.mag = MagneticField()
        self.imu = Imu()

        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 0.0
        self.euler_roll = 0.0
        self.euler_pitch = 0.0
        self.euler_yaw = 0.0
        self.angular_vel_x = 0.0
        self.angular_vel_y = 0.0
        self.angular_vel_z = 0.0
        self.linear_accel_x = 0.0
        self.linear_accel_y = 0.0
        self.linear_accel_z = 0.0
        self.magnetometer_x = 0.0
        self.magnetometer_y = 0.0
        self.magnetometer_z = 0.0

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate)
            self.get_logger().info(f"Serial port opened: {self.serial_port}")
        except serial.SerialException:
            self.get_logger().error("Failed to open serial port. Make sure it's correct and not already in use.")
            return

    def read_serial_data(self):
        if self.ser.is_open:
            try:
                received_data = self.ser.read_until(b'\n').decode('utf-8')
                try:
                    data_dict = json.loads(received_data)
                    self.process_data(data_dict)
                except json.JSONDecodeError:
                    self.get_logger().error("Error decoding JSON data.")
            except UnicodeDecodeError:
                self.get_logger().error("Error decoding received data.")
        else:
            self.get_logger().warn("Serial port is not open.")

    def process_data(self, data_dict):
        # Process the JSON data here based on the keys (e.g., "A", "G", "L", "M")
        if "A" in data_dict:
            euler_data = data_dict["A"]
            self.euler_roll = euler_data[0]
            self.euler_pitch = euler_data[1]
            self.euler_yaw = euler_data[2]
            self.get_logger().info(f"Euler Angle: euler_roll={self.euler_roll}, euler_pitch={self.euler_pitch}, euler_yaw={self.euler_yaw}")

        if "G" in data_dict:
            angular_vel_data = data_dict["G"]
            self.angular_vel_x = angular_vel_data[0]
            self.angular_vel_y = angular_vel_data[1]
            self.angular_vel_z = angular_vel_data[2]
            self.get_logger().info(f"Angular Velocity: X={self.angular_vel_x}, Y={self.angular_vel_y}, Z={self.angular_vel_z}")

        if "L" in data_dict:
            linear_accel_data = data_dict["L"]
            self.linear_accel_x = linear_accel_data[0]
            self.linear_accel_y = linear_accel_data[1]
            self.linear_accel_z = linear_accel_data[2]
            self.get_logger().info(f"Linear Acceleration: X={self.linear_accel_x}, Y={self.linear_accel_y}, Z={self.linear_accel_z}")

        if "M" in data_dict:
            magnetometer_data = data_dict["M"]
            self.magnetometer_x = magnetometer_data[0]
            self.magnetometer_y = magnetometer_data[1]
            self.magnetometer_z = magnetometer_data[2]
            self.get_logger().info(f"Magnetometer: X={self.magnetometer_x}, Y={self.magnetometer_y}, Z={self.magnetometer_z}")

    def publish_imu(self):
        self.imu.header = Header()
        self.imu.header.frame_id = "imu_link"  # Set the correct frame ID
        self.imu.header.stamp = self.get_clock().now().to_msg()

        # Convert Euler angles to quaternion
        self.euler_to_quaternion()
        self.imu.orientation.x = self.qx
        self.imu.orientation.y = self.qy
        self.imu.orientation.z = self.qz
        self.imu.orientation.w = self.qw
        self.imu.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        # Set angular velocity data
        self.imu.angular_velocity.x = float(self.angular_vel_x)
        self.imu.angular_velocity.y = float(self.angular_vel_y)
        self.imu.angular_velocity.z = float(self.angular_vel_z)
        self.imu.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]

        # Set linear acceleration data
        self.imu.linear_acceleration.x = float(self.linear_accel_x)
        self.imu.linear_acceleration.y = float(self.linear_accel_y)
        self.imu.linear_acceleration.z = float(self.linear_accel_z)
        self.imu.linear_acceleration_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.05
        ]

        self.publisher_imu.publish(self.imu)

    def publish_magnet(self):
        self.mag.magnetic_field.x = float(self.magnetometer_x)
        self.mag.magnetic_field.y = float(self.magnetometer_y)
        self.mag.magnetic_field.z = float(self.magnetometer_z)

        self.publisher_mag.publish(self.mag)

    def euler_to_quaternion(self):
        sr = math.sin(math.radians(self.euler_roll) / 2)
        cr = math.cos(math.radians(self.euler_roll) / 2)
        sp = math.sin(math.radians(self.euler_pitch) / 2)
        cp = math.cos(math.radians(self.euler_pitch) / 2)
        sy = math.sin(math.radians(self.euler_yaw) / 2)
        cy = math.cos(math.radians(self.euler_yaw) / 2)

        self.qw = cy * cp * cr + sy * sp * sr
        self.qx = cy * cp * sr - sy * sp * cr
        self.qy = sy * cp * sr + cy * sp * cr
        self.qz = sy * cp * cr - cy * sp * sr

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

