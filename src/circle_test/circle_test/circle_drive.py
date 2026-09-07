import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class circle_drive(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('circle_driver_node')
        
        # Create a publisher on the '/cmd_vel' topic with a queue size of 10
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create a timer to publish at 10 Hz (0.1 seconds)
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        
        # Set linear and angular velocities
        msg.linear.x = 0.2
        msg.angular.z = 0.5
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: v_x={msg.linear.x}, w_z={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = circle_drive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
