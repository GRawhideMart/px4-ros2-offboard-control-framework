import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry
from nav_msgs.msg import Path

from px4_trajectory_tracking.utils.settings import *
from px4_trajectory_tracking.utils.geometry import calculate_circular_trajectory, ned_to_enu_pose

class OffboardAgent(Node):
    """
    An agent that interacts with the PX4 Autopilot to perform offboard control operations.
    It handles state estimation, trajectory generation, and command publishing.
    """

    def __init__(self):
        """
        Initializes the OffboardAgent, ROS 2 publishers, subscribers, and timers.
        """
        super().__init__('offboard_agent')

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=QOS_DEPTH
        )

        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', self.qos_profile)
        self.traj_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.qos_profile)
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', self.qos_profile)
        self.viz_path_pub = self.create_publisher(Path, '/visual_path', 10)
        
        # Subscribers
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self._status_cb, self.qos_profile)
        self.vehicle_odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self._odometry_cb, self.qos_profile)

        # State initialization
        self.vehicle_status = VehicleStatus()
        self.tick_counter = 0
        self.path_history = Path()
        self.path_history.header.frame_id = 'map'

        self.timer = self.create_timer(TIMER_PERIOD, self.step)

    def step(self):
        """
        Performs a single control step. Publishes heartbeat and trajectory setpoints.
        """
        self._publish_heartbeat()
        self._check_arming_condition()
        self._publish_trajectory()
        self.tick_counter += 1

    def send_command(self, command, param1=0.0, param2=0.0):
        """
        Sends a VehicleCommand to the flight controller.

        Parameters:
        command (int): The MAVLink command ID.
        param1 (float): First parameter of the command.
        param2 (float): Second parameter of the command.
        """
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)

    def _publish_heartbeat(self):
        """Publishes the OffboardControlMode heartbeat to keep the connection alive."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        self.offboard_mode_pub.publish(msg)

    def _check_arming_condition(self):
        """Checks if the agent should arm the vehicle based on the elapsed time."""
        if self.tick_counter == int(ARMING_THRESHOLD / TIMER_PERIOD):
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def _publish_trajectory(self):
        """Computes and publishes the next trajectory setpoint."""
        time_sec = self.tick_counter * TIMER_PERIOD
        x, y, vx, vy = calculate_circular_trajectory(TRAJECTORY_RADIUS, ANGULAR_VELOCITY, time_sec)

        msg = TrajectorySetpoint()
        msg.position = [x, y, FLIGHT_ALTITUDE]
        msg.velocity = [vx, vy, 0.0]
        msg.yaw = math.atan2(y, x) + 1.57

        self.traj_setpoint_pub.publish(msg)

    def _status_cb(self, msg):
        """Callback for vehicle status updates."""
        self.vehicle_status = msg

    def _odometry_cb(self, msg):
        """
        Callback for vehicle odometry. Handles visualization path updates.
        
        Parameters:
        msg (VehicleOdometry): The odometry message from PX4.
        """
        pose = ned_to_enu_pose(msg.position, self.get_clock().now().to_msg())
        self.path_history.poses.append(pose)
        
        if len(self.path_history.poses) > VISUALIZATION_HISTORY:
            self.path_history.poses.pop(0)

        self.viz_path_pub.publish(self.path_history)