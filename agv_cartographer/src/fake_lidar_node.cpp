#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("fake_lidar_node");

  auto scan_high_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan_high", 10);
  auto scan_low_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan_low", 10);

  rclcpp::WallRate loop_rate(10);  // Publish rate: 10 Hz

  while (rclcpp::ok()) {
    auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

    // Fill in scan_msg data here
    scan_msg->angle_min = -M_PI;
    scan_msg->angle_max = M_PI;
    scan_msg->angle_increment = 0.01;
    scan_msg->time_increment = 0.0;
    scan_msg->scan_time = 0.1;
    scan_msg->range_min = 0.1;
    scan_msg->range_max = 10.0;
    scan_msg->ranges = {1.0, 2.0, 3.0}; // Example range values

    scan_high_pub->publish(scan_msg);
    scan_low_pub->publish(scan_msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

