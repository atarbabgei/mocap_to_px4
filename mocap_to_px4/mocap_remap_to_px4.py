# Standard library imports

# Third-party imports
import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleOdometry
from mocap_msgs.msg import Position

from numpy import NaN

# Pubsub node class definition
class MoCapPubSub(Node):

    # Pubsub constructor
    def __init__(self):
        super().__init__('mocap_remap_to_px4')  # Initialize node

        # Declare parameters with type
        self.declare_parameter('uav_topic', rclpy.Parameter.Type.STRING)  # UAV topic (expected from launch file)
        self.declare_parameter('px4_visual_odometry_topic', rclpy.Parameter.Type.STRING)  # PX4 topic (expected from launch file)

        # Get parameters
        uav_topic = self.get_parameter('uav_topic').get_parameter_value().string_value
        px4_visual_odometry_topic = self.get_parameter('px4_visual_odometry_topic').get_parameter_value().string_value

        if not uav_topic or not px4_visual_odometry_topic:
            self.get_logger().fatal("Parameters 'uav_topic' and 'px4_visual_odometry_topic' must be provided.")
            rclpy.shutdown()
            return

        # Initialize subscriber to UAV topic
        self.uav_subscription = self.create_subscription(Position, 
            uav_topic, self.mocap_callback, 10)

        # Initialize publisher to PX4 vehicle_visual_odometry topic
        self.px4_publisher = self.create_publisher(VehicleOdometry, 
            px4_visual_odometry_topic, 10)

        # Initialize subscriber count
        self.prev_subscription_count = 0

        # Create a timer to check for subscription changes
        self.subscription_check_timer = self.create_timer(1.0, self.check_subscriptions)

        self.get_logger().info(f"PX4 mocap pub-sub node initialized with:\n Subscriber Topic: {uav_topic}\n Publisher Topic: {px4_visual_odometry_topic}")

    # Callback for when new mocap message received to publish to PX4
    def mocap_callback(self, msg):
        msg_px4 = VehicleOdometry()  # Message to be sent to PX4

        # Transfer data from mocap message to PX4 message
        # VICON Front, Left, Up to PX4 Front, Right, Down
        # Position/orientation components
        msg_px4.pose_frame = 2  # FRD from px4 message
        msg_px4.position = [msg.x_trans / 1000.0, -msg.y_trans / 1000.0, -msg.z_trans / 1000.0]
        msg_px4.q = [msg.w, msg.x_rot, -msg.y_rot, -msg.z_rot]
        
        # Velocity components (unknown)
        msg_px4.velocity_frame = 2  # FRD from px4 message
        msg_px4.velocity = [NaN, NaN, NaN]
        msg_px4.angular_velocity = [NaN, NaN, NaN]

        # Variances
        msg_px4.position_variance = [0.0, 0.0, 0.0]
        msg_px4.orientation_variance = [0.0, 0.0, 0.0]
        msg_px4.velocity_variance = [0.0, 0.0, 0.0]

        self.px4_publisher.publish(msg_px4)  # Publish to PX4

    # Check for subscription changes
    def check_subscriptions(self):
        current_subscription_count = self.px4_publisher.get_subscription_count()
        if current_subscription_count != self.prev_subscription_count:
            if current_subscription_count > 0:
                self.get_logger().info("/fmu/in/vehicle_visual_odometry is now subscribed by a PX4 vehicle.")
            else:
                self.get_logger().warn("No subscribers on /fmu/in/vehicle_visual_odometry topic. Please check PX4 connection on ROS2 network.")
            self.prev_subscription_count = current_subscription_count

def main(args=None):
    rclpy.init(args=args)

    mocap_remap_to_px4 = MoCapPubSub()

    try:
        rclpy.spin(mocap_remap_to_px4)
    except KeyboardInterrupt:
        pass
    finally:
        mocap_remap_to_px4.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
