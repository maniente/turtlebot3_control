import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped,PolygonStamped,Point32
from nav_msgs.msg import Odometry
import sys
import math
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan,PointCloud
import numpy as np
from scipy.ndimage import label, center_of_mass
from scipy.spatial import ConvexHull,Delaunay

class Pose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

class ExploreNode(Node):

    def __init__(self):
        # Initialize the node
        super().__init__('Explore_Node')
        # Initialize the publisher

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.first = True
        self.timer = self.create_timer(self.timer_period, self.go_to_goal)

        self.goal_pub = self.create_publisher(PointStamped, '/new_goal', 10)
        self.point_cloud_pub = self.create_publisher(PointCloud, '/point_borders', 10)

        self.pose_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pose = Pose()
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.index  = 0
        self.counter = 0

        self.goal = Pose()
        self.old_goal = self.goal

        self.turn_direction = 1
        
        self.intregral_av = 0
        self.intregral_lv = 0
        self.max_val_x_vel = 0.22
        self.max_angular_vel = 4
        
        self.goal_reached = False
        self.obstacle_detected = False
        self.first = True
        self.block = False
        
    def lidar_callback(self, data):
        # Parameters
        field_range = 60  # Field of view in degrees
        initial_angle = 330  # Start angle in degrees
        detection_threshold = 0.75  # Distance to detect an obstacle (meters)

        # Reset obstacle detection
        self.obstacle_detected = False

        # Process lidar ranges
        message_range = np.array(data.ranges)  # Convert to NumPy array for easier processing

        # Define angles of interest
        angles_of_interest = [(initial_angle + i) % 360 for i in range(field_range)]

        # Obstacle detection and direction determination
        left_count = 0
        right_count = 0

        for angle in angles_of_interest:
            distance = message_range[angle]
            if 0 < distance < detection_threshold:  # Valid and within threshold
                self.obstacle_detected = True
                if angle < 360 and angle >= 330:  # Right side (e.g., angles 330-360)
                    right_count += 1
                elif angle < 30 and angle >= 0:  # Left side (e.g., angles 0-30)
                    left_count += 1

        if self.obstacle_detected:
            if left_count > right_count or (left_count-right_count)<10:
                self.turn_direction = -1  # Turn right
            else:
                self.turn_direction = 1   # Turn left
        

    def map_callback (self, map_data):
        #self.get_logger().info(f'test')
        self.counter+=1
        if not self.goal_reached and self.counter < 55:
            return

        width = map_data.info.width
        height = map_data.info.height
        
        if self.first :
            #self.origin = map_data.info.origin
            self.res = map_data.info.resolution
            self.first = False
        
        self.origin = map_data.info.origin

        siny_cosp = 2 * (self.origin.orientation.w * self.origin.orientation.z + self.origin.orientation.x * self.origin.orientation.y)
        cosy_cosp = 1 - 2 * (self.origin.orientation.y**2 + self.origin.orientation.z**2)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        new_goal = Pose()

        data = np.array(map_data.data).reshape((height, width))
        
        border_points = np.argwhere(data >= 70)

        binary_map = np.zeros_like(data)
        if len(border_points) >= 3:
            #hull = ConvexHull(border_points)
            delaunay = Delaunay(border_points)
        
            # Step 5: For each point in the map, check if it is inside the convex hull and its value is -1
            for i in range(height):
                for j in range(width):
                    if data[i, j] == -1:  # Check if the point is -1 in the map
                        point = np.array([i, j])
                        # Check if the point is inside the convex hull using Delaunay's find_simplex
                        if delaunay.find_simplex(point) >= 0:  # The point is inside the convex hull
                            binary_map[i, j] = 1  # Mark this point in the binary map

            #else 
                #vai a caso
            #self.get_logger().info(f'len: {binary_map}')
            s = [[1,1,1],[1,1,1],[1,1,1]]
            labeled_array, num_features = label(binary_map, structure=s)

            region_sizes = [(labeled_array == i).sum() for i in range(1, num_features + 1)]
            self.get_logger().info(f'region sizes {region_sizes}')
            largest_region_index = np.argmax(region_sizes) + 1

            centroids = []
            for region_index in range(1, num_features + 1):

                # Calculate the center of mass for each region
                centroid_cell = center_of_mass(binary_map, labeled_array, region_index)
                centroid_x = centroid_cell[1] * self.res
                centroid_y = centroid_cell[0] * self.res
                
                # Transform the centroid to world coordinates
                world_x = (
                    centroid_x * math.cos(theta) - centroid_y * math.sin(theta) + self.origin.position.x
                )
                world_y = (
                    centroid_x * math.sin(theta) + centroid_y * math.cos(theta) + self.origin.position.y
                )
                
                # Add to the list of centroids
                centroids.append((world_x, world_y))
                
                # Log each centroid
                self.get_logger().info(f"Centroid {region_index}: ({world_x}, {world_y})")
            
            if self.counter >= 55 or self.block:
                random_index = np.random.choice(len(centroids))  # Select an index randomly
                centroid_cell = centroids[random_index]  # Get the centroid tuple
            else:
                centroid_cell = centroids[largest_region_index]
                self.get_logger().info(f"Centroid {largest_region_index}: ({centroids[largest_region_index]})")
                self.get_logger().info(f"counter {self.counter}")
            self.counter = 0
            
            new_goal.x = centroid_cell[0]
            new_goal.y = centroid_cell[1]
        else:
            new_goal.x = self.pose.x+1
            new_goal.y = self.pose.y+1

        self.goal = new_goal

        point_msg = PointStamped()
        point_cloud_msg = PointCloud()

        point_cloud_msg.header.frame_id = 'map' 
        point_cloud_msg.header.stamp = self.get_clock().now().to_msg()

        point_msg.header.frame_id = 'map'
        point_msg.header.stamp = self.get_clock().now().to_msg()

        point_msg.point.x = new_goal.x 
        point_msg.point.y = new_goal.y
        point_msg.point.z = 0.0

        self.goal_pub.publish(point_msg)

        try:
            converted_points = []

            hull_points = np.argwhere(binary_map == 1)
            #self.get_logger().info(f'bp: {hull_points}')
            for point in hull_points:
                j, i = point  # Cell indices
                #self.get_logger().info(f'bp: {i}*{self.res} cos{theta}')
                px = i*self.res
                py = j*self.res
                new_x = px * math.cos(theta) - py*math.sin(theta) + self.origin.position.x
                new_y = px * math.sin(theta) + py*math.cos(theta) + self.origin.position.y
                #self.get_logger().info(f'bp: {new_x}, {new_y}')
                
                converted_points.append(Point32(x=new_x, y=new_y, z=0.0))

            point_cloud_msg.points = converted_points
            self.point_cloud_pub.publish(point_cloud_msg)
        except:
             self.get_logger().info(f'no hull')
    

    def odom_callback(self, data):
        pose = data.pose.pose
        siny_cosp = 2 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y)
        cosy_cosp = 1 - 2 * (pose.orientation.y**2 + pose.orientation.z**2)
        
        # Remove trailing commas here to assign floats instead of tuples
        self.pose.x = pose.position.x
        self.pose.y = pose.position.y
        self.pose.theta = math.atan2(siny_cosp, cosy_cosp)
    
    def go_to_goal(self):
        
        if self.goal != self.old_goal:
            self.intregral_av = 0
            self.intregral_lv = 0
            self.old_goal = self.goal
            self.get_logger().info(f'nuovo')
        elif self.goal_reached:
            self.block = True
        else:
            self.block = False
        goal = self.goal
        
        new_vel = Twist()
        
        # Print the difference to the goal as a debug statement
        distance_error = math.sqrt((goal.x - self.pose.x)**2 + (goal.y - self.pose.y)**2)

        angle_to_goal = math.atan2(goal.y - self.pose.y, goal.x - self.pose.x)

        angle_error = angle_to_goal - self.pose.theta
        angle_error = angle_norm(angle_error)

        pose_angle_err = goal.theta - self.pose.theta
        pose_angle_err = angle_norm(pose_angle_err)

        #self.get_logger().info(f'e d: {distance_error}')
        #self.get_logger().info(f'e a: {angle_error}')

        tolerance_dist = 1
        tolerance_angle = 1

        kp_av = self.max_angular_vel/(2*math.pi)
        
        kp_lv = 0.5

        sample_time = self.timer_period
        
        ki_lv = 1.5
        ki_av = 0.5

        if (self.obstacle_detected):
            new_vel.angular.z = 0.5*self.turn_direction
            new_vel.linear.x = 0.0
            self.get_logger().info(f'obstacle')

        elif  distance_error >= tolerance_dist:
            self.goal_reached = False
            self.get_logger().info(f'Goal ->')
            self.integral_av =+ ki_av*angle_error*sample_time
            self.integral_lv =+ ki_lv*distance_error*sample_time
            
            prop_av = kp_av* angle_error
            prop_lv = kp_lv* distance_error
            
            u_av = self.integral_av +prop_av
            u_lv = self.integral_lv +prop_lv

            if abs(u_av) > self.max_angular_vel:
                u_av = math.copysign(self.max_angular_vel, u_av)
            
            if abs(u_lv) > self.max_val_x_vel:
                u_lv = math.copysign(self.max_val_x_vel, u_lv)

            #self.get_logger().info(f'u av: {u_av}')
            #self.get_logger().info(f'u lv: {u_lv}')

            new_vel.angular.z = u_av
            new_vel.linear.x = u_lv
            
        # elif abs(pose_angle_err) >= tolerance_angle:
        #     new_vel.linear.x = 0.0
        #     new_vel.angular.z = kp_av*pose_angle_err
            
        else:
            new_vel.angular.z = 0.0
            new_vel.linear.x = 0.0
            self.cmd_vel_pub.publish(new_vel)
            self.get_logger().info(f'Goal Reached')
            self.goal_reached = True
            self.integral_av = 0
            self.integral_lv = 0

            return True

        if  new_vel.linear.x > self.max_val_x_vel:
            new_vel.linear.x = self.max_val_x_vel
        
        #new_vel.linear.x = 0.0
        self.cmd_vel_pub.publish(new_vel)
        return False
        
    def follow_path(self):
        if self.first:
            self.first = False
            if sys.argv[1] != '':
                command = sys.argv[1]
                arg_1 = float(sys.argv[1])
                arg_2 = float(sys.argv[2])
                if sys.argv[3] != '':
                    arg_3 = float(sys.argv[3])
            else:
                arg_1 = 0
                arg_2 = 0
                arg_3 = 0
            # square side-lenght start from that point and return at the same starting point
            if command == 'square':
                lside = float(sys.argv[2])
                self.path = square_path(lside,self.pose.x,self.pose.y,self.pose.theta)
            else:
                point = Pose()
                point.x = arg_1
                point.y = arg_2
                point.theta = arg_3
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
        driver = ExploreNode()
        rclpy.spin(driver)
    	
    except KeyboardInterrupt:
        driver.stop_turtlebot()
        driver.destroy_node()
        rclpy.shutdown()
    	
if __name__ == '__main__':
    main()
