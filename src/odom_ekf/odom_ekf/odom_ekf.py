import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Imu, JointState

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from rclpy.qos import QoSProfile

from message_filters import Subscriber, ApproximateTimeSynchronizer

from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomEKF(Node):
    def __init__(self):
        super().__init__('odom_ekf')
        self.get_logger().info("Odom EKF Node Started")

        self.imu_sub = Subscriber(self, Imu, '/imu')
        self.joint_state_sub = Subscriber(self, JointState, '/joint_states')
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.sync = ApproximateTimeSynchronizer([self.imu_sub, self.joint_state_sub], 10, slop = 0.04)
        self.sync.registerCallback(self.sync_callback)

        # define parameters
        self.r = 0.05
        self.L = 0.149

        # define state and covariance
        self.state = np.zeros(6, dtype=np.float64) # [x, y, theta, vx, vy, omega]
        self.state_cov = np.diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01]) 
        self.state_cov_bar = np.copy(self.state_cov)

        # define input
        self.u_vx = 0.0
        self.u_vy = 0.0
        self.u_w = 0.0
        self.u_cov = np.diag([0.01, 0.01, 0.01])

        # define time 
        self.last_time = None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_hz = 0.02
        self.tf_timer = self.create_timer(self.tf_hz, self.tf_callback)

    def cmd_vel_callback(self, msg):
        self.u_vx = msg.linear.x
        self.u_vy = msg.linear.y
        self.u_w = msg.angular.z

    def sync_callback(self, imu_msg, joint_state_msg):
        now = self.get_clock().now().to_msg()
        current_time = now.sec + now.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = current_time - self.last_time
        if dt <= 0:
            return
        # prediction part
        self.prediction(dt)

        # update part
        K = self.calculate_Kalman_gain(imu_msg)
        self.update(joint_state_msg, imu_msg, K)
        self.last_time = current_time
        
        self.publish_odometry(self.last_time)


    def prediction(self, dt):
        self.state[0] += (self.state[3] * np.cos(self.state[2]) - self.state[4] * np.sin(self.state[2])) * dt
        self.state[1] += (self.state[3] * np.sin(self.state[2]) + self.state[4] * np.cos(self.state[2])) * dt
        self.state[2] += self.state[5] * dt
        self.state[3] += (self.u_vx - self.state[3]) * dt / 0.13
        self.state[4] += (self.u_vy - self.state[4]) * dt / 0.14
        self.state[5] += (self.u_w - self.state[5]) * dt / 0.013

        Gx = np.array([
            [1, 0, (-self.state[3] * np.sin(self.state[2]) - self.state[4] * np.cos(self.state[2])) * dt, np.cos(self.state[2]) * dt, -np.sin(self.state[2]) * dt, 0],
            [0, 1, (self.state[3] * np.cos(self.state[2]) - self.state[4] * np.sin(self.state[2])) * dt, np.sin(self.state[2]) * dt, np.cos(self.state[2]) * dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1 - dt / 0.13, 0, 0],
            [0, 0, 0, 0, 1 - dt / 0.14, 0],
            [0, 0, 0, 0, 0, 1 - dt / 0.013]
        ])
        Gu = np.array([
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [dt / 0.13, 0, 0],
            [0, dt / 0.14, 0],
            [0, 0, dt / 0.013]
        ])
        self.state_cov_bar = Gx @ self.state_cov @ Gx.T + Gu @ self.u_cov @ Gu.T

    def calculate_Kalman_gain(self, imu_msg):
        self.H = np.array([
            [0.0, 0.0, 0.0, -1.0, 0.0, -self.L],
            [0.0, 0.0, 0.0, np.cos(np.pi/3), -np.cos(np.pi/6), -self.L],
            [0.0, 0.0, 0.0, np.sin(np.pi/6),  np.cos(np.pi/6), -self.L],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ], dtype=np.float64)

        self.H[0:3, :] /= self.r

        imu_cov = imu_msg.angular_velocity_covariance[8]
        self.z_cov = np.diag([0.01, 0.01, 0.01, imu_cov])

        return self.state_cov_bar @ self.H.T @ np.linalg.pinv(self.H @ self.state_cov_bar @ self.H.T + self.z_cov)

    def update(self, joint_state_msg, imu_msg, K):
        z = np.array([joint_state_msg.velocity[0], joint_state_msg.velocity[1], joint_state_msg.velocity[2], imu_msg.angular_velocity.z])
        z_hat = self.H @ self.state
        self.state = self.state + K @ (z - z_hat)
        self.state_cov = (np.eye(6) - K @ self.H) @ self.state_cov_bar

    def publish_odometry(self, timestamp):
        odom_msg = Odometry()
        odom_msg.header.stamp.sec = int(timestamp)
        odom_msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.state[0]
        odom_msg.pose.pose.position.y = self.state[1]
        odom_msg.pose.pose.position.z = 0.0
        
        quaternion = quaternion_from_euler(0, 0, self.state[2])
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        odom_msg.twist.twist.linear.x = self.state[3]
        odom_msg.twist.twist.linear.y = self.state[4]
        odom_msg.twist.twist.angular.z = self.state[5]

        pose_cov = np.zeros((6,6))
        pose_cov[0, 0] = self.state_cov[0, 0]  # x 
        pose_cov[1, 1] = self.state_cov[1, 1]  # y 
        pose_cov[2, 2] = 1e-6              
        pose_cov[3, 3] = 1e-6             
        pose_cov[4, 4] = 1e-6              
        pose_cov[5, 5] = self.state_cov[2, 2]  # yaw

        pose_cov[0, 1] = self.state_cov[0, 1]  # x-y covariance
        pose_cov[1, 0] = self.state_cov[1, 0]  # y-x covariance
        pose_cov[0, 5] = self.state_cov[0, 2]
        pose_cov[1, 5] = self.state_cov[1, 2]
        pose_cov[5, 0] = self.state_cov[2, 0]
        pose_cov[5, 1] = self.state_cov[2, 1]

        odom_msg.pose.covariance = pose_cov.flatten().tolist()

        twist_cov = np.zeros((6,6))
        twist_cov[0, 0] = self.state_cov[3, 3]  # vx
        twist_cov[1, 1] = self.state_cov[4, 4]  # vy
        twist_cov[2, 2] = 1e-6                  # vz
        twist_cov[3, 3] = 1e-6                  # wx
        twist_cov[4, 4] = 1e-6                  # wy
        twist_cov[5, 5] = self.state_cov[5, 5]  # wz  ← 关键修正：原来写到了 [2,2]

        twist_cov[0, 1] = self.state_cov[3, 4]  # vx-vy covariance
        twist_cov[1, 0] = self.state_cov[4, 3]  # vy-vx covariance
        twist_cov[0, 5] = self.state_cov[3, 5]  # vx-omega covariance
        twist_cov[5, 0] = self.state_cov[5, 3]  # omega-vx covariance
        twist_cov[1, 5] = self.state_cov[4, 5]  # vy-omega covariance
        twist_cov[5, 1] = self.state_cov[5, 4]  # omega-vy covariance

        odom_msg.twist.covariance = twist_cov.flatten().tolist()

        self.odom_pub.publish(odom_msg)

    def tf_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.state[0]
        t.transform.translation.y = self.state[1]
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = np.sin(self.state[2] / 2.0)
        t.transform.rotation.w = np.cos(self.state[2] / 2.0)

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomEKF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()