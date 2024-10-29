import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
import math

class Pose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

class DriverNode(Node):

    def __init__(self):
        # Initialize the node
        super().__init__('GoToGoal')
        # Initialize the publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.go_to_goal)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pose = Pose()

    def odom_callback(self, data):
        pose = data.pose.pose
        siny_cosp = 2 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y)
        cosy_cosp = 1 - 2 * (pose.orientation.y**2 + pose.orientation.z**2)
        
        # Remove trailing commas here to assign floats instead of tuples
        self.pose.x = pose.position.x
        self.pose.y = pose.position.y
        self.pose.theta = math.atan2(siny_cosp, cosy_cosp)
    
    def angle_norm (self,angle):
        
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        
        return angle

    def go_to_goal(self):
        goal = Pose()
        
        # Read goal positions from command-line arguments
        goal.x = float(sys.argv[1])
        goal.y = float(sys.argv[2])
        goal.theta = float(sys.argv[3])

        new_vel = Twist()
        
        # Print the difference to the goal as a debug statement
        distance_error = math.sqrt((goal.x - self.pose.x)**2 + (goal.y - self.pose.y)**2)

        angle_to_goal = math.atan2(goal.y - self.pose.y, goal.x - self.pose.x)

        angle_error = angle_to_goal - self.pose.theta
        angle_error = self.angle_norm(angle_error)

        pose_angle_err = goal.theta - self.pose.theta
        pose_angle_err = self.angle_norm(pose_angle_err)

        self.get_logger().info(f'{angle_to_goal} - {self.pose.theta} ')
        self.get_logger().info(f'e: {distance_error}')
        
        self.get_logger().info(f'e: {angle_error}')

        tolerance_dist = 0.1

        max_angular_vel = 3.0
        kp = max_angular_vel/(2*math.pi)
        max_val_x_vel = 2.0
        
        if distance_error >= tolerance_dist:
            new_vel.angular.z = kp* angle_error
            new_vel.linear.x = kp * distance_error
            
        elif abs(pose_angle_err) >= tolerance_dist:
            new_vel.linear.x = 0.0
            new_vel.angular.z = kp*pose_angle_err
            
        else:
            new_vel.angular.z = 0.0
            new_vel.linear.x = 0.0
            self.cmd_vel_pub.publish(new_vel)
            self.get_logger().info(f'Goal Reached')
            quit()

        if  new_vel.linear.x > max_val_x_vel:
            new_vel.linear.x = max_val_x_vel
        #new_vel.linear.x = 0.0
        self.cmd_vel_pub.publish(new_vel)
        
        
    def stop_turtlebot(self):
        # define what happens when program is interrupted
        self.get_logger().info('stopping turtlebot')
        self.cmd_vel_pub.publish(Twist())  # Stop movement

def main(args=None):
    rclpy.init(args=args)
    
    try:
        driver = DriverNode()
        rclpy.spin(driver)
    	
    except KeyboardInterrupt:
        driver.stop_turtlebot()
        driver.destroy_node()
        rclpy.shutdown()
    	
if __name__ == '__main__':
    main()
