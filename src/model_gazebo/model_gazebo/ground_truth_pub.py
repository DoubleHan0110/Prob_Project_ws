import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math

class GroundTruthPub(Node):
    def __init__(self):
        super().__init__('ground_truth_pub')
        
        self.base_link_name = 'LeKiwi_simplified_lidar::base_link'
        
        self.subscription = self.create_subscription(LinkStates,'/gazebo/link_states',self.link_states_callback,10)
        
        self.pose_pub = self.create_publisher(PoseStamped, '/ground_truth/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/ground_truth/twist', 10)
        
        self.get_logger().info(f'Ground Truth Publisher started via link: {self.base_link_name}')

    def link_states_callback(self, msg):
        if self.base_link_name in msg.name:
            idx = msg.name.index(self.base_link_name)
            
            pose_raw = msg.pose[idx]
            twist_raw = msg.twist[idx]
            
            current_time = self.get_clock().now().to_msg()
            
            # pose ground truth
            pose_msg = PoseStamped()
            pose_msg.header.stamp = current_time
            pose_msg.header.frame_id = 'odom'
            
            pose_msg.pose.position.x = pose_raw.position.x
            pose_msg.pose.position.y = pose_raw.position.y
            pose_msg.pose.position.z = 0.0
            
            
            q = [pose_raw.orientation.x, pose_raw.orientation.y, pose_raw.orientation.z, pose_raw.orientation.w]
            yaw = euler_from_quaternion(q)[2]
            
            
            r_new = quaternion_from_euler(0, 0, yaw)
            
            pose_msg.pose.orientation.x = r_new[0]
            pose_msg.pose.orientation.y = r_new[1]
            pose_msg.pose.orientation.z = r_new[2]
            pose_msg.pose.orientation.w = r_new[3]
            
            self.pose_pub.publish(pose_msg)
            
            # twist ground truth
            twist_msg = TwistStamped()
            twist_msg.header.stamp = current_time
            twist_msg.header.frame_id = 'base_footprint_gt'
            
            # trans from world - body
            v_x_world = twist_raw.linear.x
            v_y_world = twist_raw.linear.y
            
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            
            v_x_body = v_x_world * cos_yaw + v_y_world * sin_yaw
            v_y_body = -v_x_world * sin_yaw + v_y_world * cos_yaw
            
            twist_msg.twist.linear.x = v_x_body
            twist_msg.twist.linear.y = v_y_body
            twist_msg.twist.linear.z = 0.0
            
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = twist_raw.angular.z
            
            self.twist_pub.publish(twist_msg)
                


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
