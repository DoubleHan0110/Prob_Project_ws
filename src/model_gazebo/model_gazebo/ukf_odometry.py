import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np

class UkfOdometry(Node):
    def __init__(self):
        super().__init__('ukf_odometry')
        self.get_logger().info("UKF Node Started")
        self.state = np.zeros(6)  # [x, y, theta, vx, vy, omega]
        self.state_cov = np.eye(6) * 0.00001  # confident initial state

        self.last_time = None
        self.last_u = None  # [ax, ay, w_gyro]
        self.last_Su = None  # Process noise covariance

        # UKF parameters for n=6 state dimensions
        self.alpha = 1e-3
        self.beta = 2.0
        self.kappa = 0.0

        r = 0.05  # wheel radius
        L0 = 0.132  # wheelbase length parameter

        self.C_matrix = (1/r) * np.array([
            [0, 0, 0, -1, 0, L0],
            [0, 0, 0, np.sin(np.pi/6), -np.cos(np.pi/6), L0],
            [0, 0, 0, np.sin(np.pi/6), np.cos(np.pi/6), L0]
        ])

        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.ukf_pub = self.create_publisher(Odometry, '/ukf_odometry', 10)
        
    def imu_callback(self, msg):
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        u = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.angular_velocity.z])
        S_u = np.diag([msg.linear_acceleration_covariance[0],
                       msg.linear_acceleration_covariance[4],
                       msg.angular_velocity_covariance[8]])
        
        if self.last_time is not None:
            dt = msg_time - self.last_time
            self.predict(dt, u, S_u)
        
        self.last_time = msg_time
        self.last_u = u
        self.last_Su = S_u
        self.publish_odometry(msg.header.stamp)

    # here we corrupt the measurement with noise as soon as it comes in
    # we apply noise sampled from a zero-mean gaussian with 1 percent current velocity as stddev
    def joint_state_callback(self, msg):
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None: #throw out measurement since we have no prior yet
            return
        
        # First run prediction on remaining time since last update
        dt = msg_time - self.last_time
        self.predict(dt, self.last_u, self.last_Su)

        # Process Joint State data here (measurement model)
        w1, w2, w3 = msg.velocity
        S_z = 0.02**2 * np.diag([w1**2, w2**2, w3**2])  # Measurement noise covariance
        z = np.array(msg.velocity) + np.random.multivariate_normal(np.zeros(3), S_z)

        K = self.state_cov @ self.C_matrix.T @ np.linalg.pinv(self.C_matrix @ self.state_cov @ self.C_matrix.T + S_z)
        self.state = self.state + K @ (z - self.C_matrix @ self.state)
        self.state_cov = (np.eye(6) - K @ self.C_matrix) @ self.state_cov
        self.last_time = msg_time
        self.publish_odometry(msg.header.stamp)
    
    def predict(self, dt, u, S_u):
        """UKF prediction step with state-input augmentation"""
        n_x = 6  # state dimension
        n_u = 3  # input dimension
        n_aug = n_x + n_u  # augmented dimension
        
        # Augmented state and covariance
        x_aug = np.concatenate([self.state, u])
        P_aug = np.block([
            [self.state_cov, np.zeros((n_x, n_u))],
            [np.zeros((n_u, n_x)), S_u]
        ])
        
        # UKF weights
        lambda_aug = self.alpha**2 * (n_aug + self.kappa) - n_aug
        W_m = np.zeros(2 * n_aug + 1)
        W_c = np.zeros(2 * n_aug + 1)
        W_m[0] = lambda_aug / (n_aug + lambda_aug)
        W_c[0] = W_m[0] + (1 - self.alpha**2 + self.beta)
        for i in range(1, 2 * n_aug + 1):
            W_m[i] = 1 / (2 * (n_aug + lambda_aug))
            W_c[i] = W_m[i]
        
        # Generate augmented sigma points
        sqrt_P = np.linalg.cholesky((n_aug + lambda_aug) * P_aug)
        sigma_points_aug = np.zeros((2 * n_aug + 1, n_aug))
        sigma_points_aug[0] = x_aug
        for i in range(n_aug):
            sigma_points_aug[1 + i] = x_aug + sqrt_P[:, i]
            sigma_points_aug[1 + n_aug + i] = x_aug - sqrt_P[:, i]
        
        # Propagate sigma points through dynamics
        sigma_points_pred = np.zeros((2 * n_aug + 1, n_x))
        for i in range(2 * n_aug + 1):
            state_i = sigma_points_aug[i, :n_x]
            u_i = sigma_points_aug[i, n_x:]
            sigma_points_pred[i] = self.g(state_i, u_i, dt)
        
        x_bar = np.sum(W_m[:, np.newaxis] * sigma_points_pred, axis=0)
        P_bar = np.zeros((n_x, n_x))
        for i in range(2 * n_aug + 1):
            diff = sigma_points_pred[i] - x_bar
            P_bar += W_c[i] * np.outer(diff, diff)
        
        self.state = x_bar
        self.state_cov = P_bar
    
    ### nonlinear dynamics model - see README.md 
    def g(self, state, u, dt):
        x, y, theta, vx, vy, omega = state
        ax, ay, w_gyro = u

        x_next = x + (vx * np.cos(theta) - vy * np.sin(theta)) * dt
        y_next = y + (vx * np.sin(theta) + vy * np.cos(theta)) * dt
        theta_next = theta + ((omega + w_gyro) / 2) * dt # trapezoidal integration
        vx_next = vx + ax * dt
        vy_next = vy + ay * dt
        omega_next = w_gyro

        return np.array([x_next, y_next, theta_next, vx_next, vy_next, omega_next])
    
    def publish_odometry(self, timestamp):
        """Publish odometry message with current state estimate"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        x, y, theta, vx, vy, omega = self.state
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        
        # Convert theta to quaternion (rotation around z-axis)
        odom_msg.pose.pose.orientation.z = np.sin(theta / 2.0)
        odom_msg.pose.pose.orientation.w = np.cos(theta / 2.0)
        
        # Pose covariance (6x6, row-major: x, y, z, roll, pitch, yaw)
        pose_cov = np.zeros(36)
        pose_cov[0] = self.state_cov[0, 0]    # x variance
        pose_cov[7] = self.state_cov[1, 1]    # y variance
        pose_cov[35] = self.state_cov[2, 2]   # theta variance
        pose_cov[1] = self.state_cov[0, 1]    # x-y covariance
        pose_cov[6] = self.state_cov[1, 0]    # y-x covariance
        odom_msg.pose.covariance = pose_cov.tolist()
        
        # Twist: [vx, vy, omega] from state (in body frame)
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega
        
        # Twist covariance (6x6, row-major: vx, vy, vz, wx, wy, wz)
        twist_cov = np.zeros(36)
        twist_cov[0] = self.state_cov[3, 3]    # vx variance
        twist_cov[7] = self.state_cov[4, 4]    # vy variance
        twist_cov[35] = self.state_cov[5, 5]   # omega variance
        twist_cov[1] = self.state_cov[3, 4]    # vx-vy covariance
        twist_cov[6] = self.state_cov[4, 3]    # vy-vx covariance
        odom_msg.twist.covariance = twist_cov.tolist()
        
        self.ukf_pub.publish(odom_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = UkfOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()