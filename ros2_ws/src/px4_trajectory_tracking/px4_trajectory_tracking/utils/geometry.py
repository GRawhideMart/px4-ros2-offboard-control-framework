import math
from geometry_msgs.msg import PoseStamped

def calculate_circular_trajectory(radius, omega, time_sec):
    """
    Calculates the position and velocity for a circular trajectory.

    Parameters:
    radius (float): The radius of the circle in meters.
    omega (float): Angular velocity in rad/s.
    time_sec (float): Current time in seconds.

    Returns:
    tuple: A tuple containing (x, y, vx, vy).
    """
    # Position
    target_x = radius * math.cos(omega * time_sec)
    target_y = radius * math.sin(omega * time_sec)
    
    # Velocity (Feed-Forward)
    target_vx = -radius * omega * math.sin(omega * time_sec)
    target_vy =  radius * omega * math.cos(omega * time_sec)

    return target_x, target_y, target_vx, target_vy

def ned_to_enu_pose(position, timestamp):
    """
    Converts a PX4 NED position array into a ROS PoseStamped (ENU) message.

    Parameters:
    position (list): A list or array [x, y, z] in NED frame.
    timestamp (Time): The current ROS timestamp.

    Returns:
    PoseStamped: The pose message ready for visualization.
    """
    pose = PoseStamped()
    pose.header.stamp = timestamp
    pose.header.frame_id = 'map'
    
    pose.pose.position.x = float(position[0])
    pose.pose.position.y = float(position[1])
    pose.pose.position.z = -float(position[2]) # Invert Z for ENU

    return pose