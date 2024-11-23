import math

class Pose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def __repr__(self):  # Add this for better print formatting
        return f"Pose(x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f})"

def square_path(l, x, y, theta):
    path = []
    dist_ptp = 0.1  # Distance between points along the path
    
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
    path = square_path(1, 0, 0, math.pi/2)
    for i, pose in enumerate(path):
        print(f"Point {i}: {pose}")

if __name__ == '__main__':
    main()
