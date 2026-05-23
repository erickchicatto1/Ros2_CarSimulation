import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist

class Controller(Node):

    def __init__(self):
        super().__init__('move_robot')
        self.pub = self.create_publisher(Twist,'cmd_vel',10)
        
        #Move robot fwd
        msg = Twist()
        msg.linear.x = 0.5
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
        
        timer_period = 10.0
        self.tmr = self.create_timer(timer_period,self.timer_callback)
        
    def timer_callback(self):
       #stop the robot
        msg = Twist()
        msg.linear.x = 0.5
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        
        
        
if __name__ == '__main__':
    main()        
        
        
        
       
        
