import rclpy
from px4_trajectory_tracking.agent import OffboardAgent

def launch(args=None):
    rclpy.init(args=args)
    agent = OffboardAgent()
    
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    launch()