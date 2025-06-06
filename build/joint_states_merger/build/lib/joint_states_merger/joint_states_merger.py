import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from threading import Lock

class JointStatesMerger(Node):
    def __init__(self):
        super().__init__('joint_states_merger')

        self.declare_parameter('input_topics', ['/joint_states', '/joint_states_static'])
        self.declare_parameter('output_topic', '/joint_states_merged')

        self.input_topics = self.get_parameter('input_topics').get_parameter_value().string_array_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.lock = Lock()
        self.latest_joints = {}

        self.publisher = self.create_publisher(JointState, self.output_topic, 10)

        # Define wheel joints to exclude from static joint states
        self.wheel_joints = ['left_wheel_joint', 'right_wheel_joint']

        self.subscribers = []
        for topic in self.input_topics:
            # Use lambda default arg to pass topic info to callback
            sub = self.create_subscription(
                JointState,
                topic,
                lambda msg, t=topic: self.joint_state_callback(msg, t),
                10)
            self.subscribers.append(sub)
            self.get_logger().info(f'Subscribed to {topic}')

    def joint_state_callback(self, msg: JointState, topic: str):
        with self.lock:
            # If message is from static joint states topic, skip wheel joints
            if topic == '/joint_states_static':
                for i, name in enumerate(msg.name):
                    if name in self.wheel_joints:
                        continue  # Skip wheels from static topic
                    pos = msg.position[i] if i < len(msg.position) else 0.0
                    vel = msg.velocity[i] if (msg.velocity and i < len(msg.velocity)) else 0.0
                    eff = msg.effort[i] if (msg.effort and i < len(msg.effort)) else 0.0
                    self.latest_joints[name] = {'position': pos, 'velocity': vel, 'effort': eff}
            else:
                # For other topics (controller), accept all joints (including wheels)
                for i, name in enumerate(msg.name):
                    pos = msg.position[i] if i < len(msg.position) else 0.0
                    vel = msg.velocity[i] if (msg.velocity and i < len(msg.velocity)) else 0.0
                    eff = msg.effort[i] if (msg.effort and i < len(msg.effort)) else 0.0
                    self.latest_joints[name] = {'position': pos, 'velocity': vel, 'effort': eff}

            merged_msg = JointState()
            merged_msg.header.stamp = self.get_clock().now().to_msg()

            joint_names = sorted(self.latest_joints.keys())
            merged_msg.name = joint_names
            merged_msg.position = [self.latest_joints[n]['position'] for n in joint_names]
            merged_msg.velocity = [self.latest_joints[n]['velocity'] for n in joint_names]
            merged_msg.effort = [self.latest_joints[n]['effort'] for n in joint_names]

            self.publisher.publish(merged_msg)

def main(args=None):
    rclpy.init(args=args)
    merger = JointStatesMerger()
    rclpy.spin(merger)
    merger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
