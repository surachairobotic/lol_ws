import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)

    def send_goal(self, pose):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose = pose

        self.publisher.publish(goal_msg)


def main():
    rclpy.init()
    goal_sender = GoalSender()

    # Define the target pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'map'
    target_pose.pose.position.x = 1.0
    target_pose.pose.position.y = 2.0
    target_pose.pose.orientation.w = 1.0

    # Send the goal
    goal_sender.send_goal(target_pose.pose)

    rclpy.spin(goal_sender)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
