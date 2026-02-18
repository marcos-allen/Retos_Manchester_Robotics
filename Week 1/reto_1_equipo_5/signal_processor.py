#!/usr/bin/env python3

#imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np



#createclass
class SignalProcessor(Node):

    def __init__(self):
        super().__init__('signal_processor_node')

        self.timer = self.create_timer(0.1, self.process_callback)

        self.current_signal = 0.0
        self.current_time = 0.0

        # Crear subscriber
        self.subscription = self.create_subscription(
            Float32,
            '/signal',
            self.signal_callback,
            10
        )

        self.time_subscription = self.create_subscription(
            Float32,
            '/time',
            self.time_callback,
            10
        )

        # Crear publisher para señal procesada
        self.publisher_ = self.create_publisher(
            Float32,
            '/proc_signal',
            10
        )


    #callbackfunction
    def signal_callback(self, msg):
        self.current_signal = msg.data

    def time_callback(self, msg):
        self.current_time = msg.data

    def process_callback(self):
        phase_shift = 1.0  # hardcoded

        # recalcular señal con phase shift
        new_signal = np.sin(self.current_time + phase_shift)

        # offset para que sea positiva
        new_signal = new_signal + 1.0

        # reducir amplitud a la mitad
        new_signal = 0.5 * new_signal

        msg = Float32()
        msg.data = float(new_signal)

        self.publisher_.publish(msg)

        self.get_logger().info(f'Processed signal: {msg.data}')





# mains
def main(args=None):

    rclpy.init(args=args)

    node = SignalProcessor()

    # Ejecutar nodo
    rclpy.spin(node)

    # Destroy node
    node.destroy_node()

    # User stop
    rclpy.shutdown()


#execute node
if __name__ == '__main__':
    main()
