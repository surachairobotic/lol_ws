import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose, Point, Quaternion, PoseStamped
import requests, math, json

def send_data(data):
    url = 'http://0.0.0.0:8000/set_robotstate'
    headers = {'Content-Type': 'application/json'}
    response = requests.post(url, json=data, headers=headers)
    #print(response)
    if response.status_code == 200:
        print('Data sent successfully.')
    else:
        print('Failed to send data.')
def get_goal():
    url = 'http://0.0.0.0:8000/goal'
    headers = {'Content-Type': 'application/json'}
    response = requests.get(url, headers=headers)
    print(response)
    if response.status_code == 200:
        #print('Goal successfully.')
        #print(type(response._content.decode()))
        #print(response._content.decode())
        #for x in response._content.decode():
        #    print(x)
        json_obj = json.loads(response._content.decode())
        print(json_obj["data"])
        return json_obj["data"]
    else:
        print('Failed to get goal.')


def pose2xyyaw(position, orientation):
    x = position.x
    y = position.y
    yaw = math.atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                     1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z))
    return x, y, yaw

def xyyaw2pose(x, y, yaw):
    pose = Pose()
    pose.position = Point(x=x, y=y, z=0.0)
    pose.orientation = Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    return pose

class GoalSender():
    def __init__(self, nh):
        self.node = nh
        self.publisher = self.node.create_publisher(PoseStamped, 'goal_pose', 10)

    def send_goal(self, pose):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose = pose

        self.publisher.publish(goal_msg)

class TransformExample(Node):

    def __init__(self):
        super().__init__('transform_example')

        self.goal = GoalSender(self)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.timer_callback)
        #self.timer_goal = self.create_timer(0.5, self.goal_callback)

        self.cnt = 0

    def goal_callback(self):
    	pose_goal = get_goal()
    	if pose_goal != None:
    	    pose = xyyaw2pose(pose_goal[0], pose_goal[1], pose_goal[2])
    	    self.goal.send_goal(pose)

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_footprint', 'map', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5)
            )
            position = transform.transform.translation
            orientation = transform.transform.rotation
            # Do something with the transform data
            #print('Transform Position:', position)
            #print('Transform Orientation:', orientation)
            p = "{:.2f}, {:.2f}, {:.2f}".format(position.x, position.y, position.z)
            o = "{:.2f}, {:.2f}, {:.2f}, {:.2f}".format(orientation.x, orientation.y, orientation.z, orientation.w)
            res = pose2xyyaw(position, orientation)
            print(p, " : ", o, " : ", res)
            send_data(res)

            # self.cnt=self.cnt+1

            # if self.cnt > 20:
            #     # Define the target pose
            #     target_pose = Pose()
            #     target_pose.position.x = 1.0
            #     target_pose.position.y = 2.0
            #     target_pose.orientation.w = 1.0

            #     self.goal.send_goal(target_pose)
            #     exit()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            #print('TF lookup failed:', e)
            send_data([0,0,0])

def main(args=None):
    rclpy.init(args=args)
    transform_example = TransformExample()
    rclpy.spin(transform_example)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
