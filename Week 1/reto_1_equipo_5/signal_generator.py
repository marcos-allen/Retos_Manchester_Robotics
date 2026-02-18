#!/usr/bin/env python3

#imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np


#create class
class SignalGenerator(Node):

    def __init__(self):
        super().__init__('signal_generator_node')

        # Crear publisher
        self.signal_publisher_ = self.create_publisher(
            Float32,
            '/signal',
            10
        )

        self.time_publisher_ = self.create_publisher(
            Float32,
            '/time',
            10
        )


        # Crear timer (cada 10 hz)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Variable tiempo
        self.t = 0.0


    #callbackfuntion
    def timer_callback(self):

        # Generar señal seno
        signal = np.sin(self.t)

        # Crear mensaje
        msg = Float32()
        msg.data = float(signal)

        # mensaje tiempo
        time_msg = Float32()
        time_msg.data = float(self.t)

        # Publicar
        self.signal_publisher_.publish(msg)

        self.time_publisher_.publish(time_msg)

        # Mostrar en terminal
        self.get_logger().info(f'Signal: {msg.data}  Time: {time_msg.data}')

        # Incrementar tiempo
        self.t += 0.1


#main
def main(args=None):

    rclpy.init(args=args)

    node = SignalGenerator()

    # Ejecutar nodo
    rclpy.spin(node)

    # Destroy node
    node.destroy_node()

    # User stop
    rclpy.shutdown()


#executenode
if __name__ == '__main__':
    main()
