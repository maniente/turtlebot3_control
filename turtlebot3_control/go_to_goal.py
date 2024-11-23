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
        self.timer_period = 0.1  # seconds
        self.first = True
        self.timer = self.create_timer(self.timer_period, self.follow_path)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pose = Pose()
        self.index  = 0
        self.path = []
        self.intregral_av = 0
        self.intregral_lv = 0
        

    def odom_callback(self, data):
        pose = data.pose.pose
        siny_cosp = 2 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y)
        cosy_cosp = 1 - 2 * (pose.orientation.y**2 + pose.orientation.z**2)
        
        # Remove trailing commas here to assign floats instead of tuples
        self.pose.x = pose.position.x
        self.pose.y = pose.position.y
        self.pose.theta = math.atan2(siny_cosp, cosy_cosp)
    
    def go_to_goal(self,point):
        goal = point
        
        new_vel = Twist()
        
        # Print the difference to the goal as a debug statement
        distance_error = math.sqrt((goal.x - self.pose.x)**2 + (goal.y - self.pose.y)**2)

        angle_to_goal = math.atan2(goal.y - self.pose.y, goal.x - self.pose.x)

        angle_error = angle_to_goal - self.pose.theta
        angle_error = angle_norm(angle_error)

        pose_angle_err = goal.theta - self.pose.theta
        pose_angle_err = angle_norm(pose_angle_err)

        self.get_logger().info(f'e d: {distance_error}')
        
        self.get_logger().info(f'e a: {angle_error}')

        tolerance_dist = 0.05
        tolerance_angle = 0.05

        max_angular_vel = 10.0
        kp_av = max_angular_vel/(2*math.pi)
        
        max_val_x_vel = 2.0
        kp_lv = 0.5

        sample_time = self.timer_period
        
        ki_lv = 1.5
        ki_av = 0.5

        if distance_error >= tolerance_dist:

            self.integral_av =+ ki_av*angle_error*sample_time
            self.integral_lv =+ ki_lv*distance_error*sample_time
            
            prop_av = kp_av* angle_error
            prop_lv = kp_lv* distance_error
            
            u_av = self.integral_av +prop_av
            u_lv = self.integral_lv +prop_lv

            if abs(u_av) > max_angular_vel:
                u_av = math.copysign(max_angular_vel, u_av)
            
            if abs(u_lv) > max_val_x_vel:
                u_lv = math.copysign(max_val_x_vel, u_lv)

            self.get_logger().info(f'u av: {u_av}')
            self.get_logger().info(f'u lv: {u_lv}')

            new_vel.angular.z = u_av
            new_vel.linear.x = u_lv
            
        elif abs(pose_angle_err) >= tolerance_angle:
            new_vel.linear.x = 0.0
            new_vel.angular.z = kp_av*pose_angle_err
            
        else:
            new_vel.angular.z = 0.0
            new_vel.linear.x = 0.0
            self.cmd_vel_pub.publish(new_vel)
            self.get_logger().info(f'Goal Reached')
            self.integral_av = 0
            self.integral_lv = 0

            return True

        if  new_vel.linear.x > max_val_x_vel:
            new_vel.linear.x = max_val_x_vel
        #new_vel.linear.x = 0.0
        self.cmd_vel_pub.publish(new_vel)
        return False
        
    def follow_path(self):
        if self.first:
            self.first = False
            if sys.argv[1] != '':
                command = sys.argv[1]
            # square side-lenght start from that point and return at the same starting point
            if command == 'square':
                lside = float(sys.argv[2])
                self.path = square_path(lside,self.pose.x,self.pose.y,self.pose.theta)
            else:
                point = Pose()
                point.x = float(sys.argv[1])
                point.y = float(sys.argv[2])
                point.theta = float(sys.argv[3])
                self.path.append(point)
        else:
            point_goal = self.path[self.index]
            if self.go_to_goal(point_goal):
                self.index = self.index + 1
                if self.index == len(self.path):
                    self.get_logger().info(f'Path final point reached')
                    quit()
    
    

        
    def stop_turtlebot(self):
        # define what happens when program is interrupted
        self.get_logger().info('stopping turtlebot')
        self.cmd_vel_pub.publish(Twist())  # Stop movement

def angle_norm (angle):
        
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
        
    return angle

def square_path(l, x, y, theta):
    path = []
    dist_ptp = 0.5  # Distance between points along the path
    
    prev_point = Pose()
    prev_point.x = x
    prev_point.y = y
    prev_point.theta = theta
    
    for n in range(4):
        dist = 0
        while dist < l:
            # Create a new point along the current side
            point = Pose()
            point.x = prev_point.x + dist_ptp * math.cos(prev_point.theta)
            point.y = prev_point.y + dist_ptp * math.sin(prev_point.theta)
            point.theta = prev_point.theta
            path.append(point)
            
            # Update distance and prev_point to the new point
            dist += dist_ptp
            prev_point = point  # Move to the new point
        # Rotate by 90 degrees for the next side
        prev_point.theta += math.pi / 2
    return path

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
