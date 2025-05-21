import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np
from rcl_interfaces.msg import SetParametersResult

class JointSimulator:
    def __init__(self, K, T, noise):
        self.K = K 
        self.T = T 
        self.noise = noise
        
        self.angle = 0.0
        self.angular_velocity = 0.0
        self.voltage = 0.0
        
        self.last_time = None

    def update(self, dt):

        if dt > 0:
            self.angular_velocity = (-1.0/self.T) * self.angle + (self.K/self.T) * self.voltage
            
            self.angle += self.angular_velocity * dt
            
            if self.noise > 0:
                self.angle += np.random.normal(0, self.noise)

class JointSimulatorNode(Node):
    def __init__(self):
        super().__init__('joint_simulator_node')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('K', 230.0), 
                ('T', 0.15), 
                ('noise', 0.0)
            ])
        
        self.simulator = JointSimulator(
            self.get_parameter('K').value,
            self.get_parameter('T').value,
            self.get_parameter('noise').value
        )
        
        self.publisher_ = self.create_publisher(Float64, 'measured_angle', 10)
        
        self.subscription = self.create_subscription(
            Float64,
            'voltage',
            self.voltage_listener,
            10
        )
        
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.status_timer = self.create_timer(0.5, self.status)
        
        self.get_logger().info("Joint simulator node initialized")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'K' and param.value > 0:
                self.simulator.K = param.value
                self.get_logger().info(f"Updated K to {param.value}")
            elif param.name == 'T' and param.value > 0:
                self.simulator.T = param.value
                self.get_logger().info(f"Updated T to {param.value}")
            elif param.name == 'noise' and param.value >= 0:
                self.simulator.noise = param.value
                self.get_logger().info(f"Updated noise to {param.value}")
        
        return SetParametersResult(successful=True)

    def voltage_listener(self, msg):
        self.simulator.voltage = msg.data

    def timer_callback(self):
        current_time = self.get_clock().now()
        
        if self.simulator.last_time is None:
            dt = 0.0
            self.simulator.last_time = current_time
        else:
            dt = (current_time - self.simulator.last_time).nanoseconds / 1e9
            self.simulator.last_time = current_time
        
        self.simulator.update(dt)
        
        angle_msg = Float64()
        angle_msg.data = self.simulator.angle
        self.publisher_.publish(angle_msg)

    def status(self):
        self.get_logger().info(
            f"[STATUS] Angle: {self.simulator.angle:.3f} rad, "
            f"Velocity: {self.simulator.angular_velocity:.3f} rad/s, "
            f"Voltage: {self.simulator.voltage:.3f} V, "
        )

def main(args=None):
    rclpy.init(args=args)
    node = JointSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()