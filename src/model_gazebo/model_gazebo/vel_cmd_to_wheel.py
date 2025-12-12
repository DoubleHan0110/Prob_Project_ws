import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class VelCmdToWheel(Node):
    def __init__(self):
        super().__init__('vel_cmd_to_wheel')

        # wheel radius
        self.declare_parameter('wheel_radius', 0.1) 
        # distance between wheel center and robot base center
        self.declare_parameter('wheel_separation', 0.132) 

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.Length = self.get_parameter('wheel_separation').value

        self.publisher_ = self.create_publisher(Float64MultiArray, '/omni_wheel_controller/commands', 10)

        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)

    def cmd_vel_callback(self, msg):

        vx = msg.linear.x  
        vy = msg.linear.y  
        w  = msg.angular.z 
        # self.get_logger().info(f'vx: {vx}, vy: {vy}, w: {w}')


        r = self.wheel_radius
        L = self.Length

        # w1: back
        # w2: front right
        # w3: front left
        
        w1 = (-vx - w * L) / r
        w2 = (math.cos(math.radians(60)) * vx - math.cos(math.radians(30)) * vy - w * L) / r
        w3 = (math.cos(math.radians(60)) * vx + math.cos(math.radians(30)) * vy - w * L) / r

        self.get_logger().info(f'w1: {w1}, w2: {w2}, w3: {w3}')

        cmd_msg = Float64MultiArray()
        cmd_msg.data = [w1, w2, w3] 

        self.publisher_.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelCmdToWheel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

