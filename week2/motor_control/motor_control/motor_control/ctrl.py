# imports
import rclpy  # ros2
from rclpy.node import Node  # node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32  # msg float


# Crear class controller
class Controller(Node):

    def __init__(self):
        super().__init__('ctrl_node')

        # PARAMETRO DEL CONTROLADOR
        # self.kp =2.0
        self.declare_parameter('kp', 1.0)
        self.kp = self.get_parameter('kp').value
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        # Variables del sistema
        self.setpoint = 0.0  # VALOR DESEADO
        self.output = 0.0    # VALOR ACTUAL
        self.integral = 0.0
        self.prev_error = 0.0
        self.dt = 0.02  # mismo que el timer

        # publisher del control
        self.control_pub = self.create_publisher(Float32, 'motor_input_u', 10)

        # subscribers
        self.create_subscription(Float32, 'set_point', self.setpoint_cb, 10)  # recibe setpoint
        self.create_subscription(Float32, 'motor_speed_y', self.output_cb, 10)  # recibe salida del motor

        # ejecutar control periodicamente
        self.timer = self.create_timer(0.02, self.control_loop)

        # callback para cambios de parámetros en runtime
        self.add_on_set_parameters_callback(self.param_callback)

    # Callbacks

    def setpoint_cb(self, msg):
        self.setpoint = msg.data  # guarda el valor

    # guarda la velocidad real
    def output_cb(self, msg):
        self.output = msg.data

    def control_loop(self):
        error = self.setpoint - self.output

        # Parte integral
        self.integral += error * self.dt

        # Parte derivativa
        derivative = (error - self.prev_error) / self.dt

        # PID completo
        control_signal = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        # guardar error anterior
        self.prev_error = error

        msg = Float32()
        msg.data = control_signal

        self.control_pub.publish(msg)

    def param_callback(self, params):
        for param in params:
            if param.name == 'kp':
                self.kp = param.value
            elif param.name == 'ki':
                self.ki = param.value
            elif param.name == 'kd':
                self.kd = param.value

        self.get_logger().info(
            f"PID actualizado -> Kp:{self.kp} Ki:{self.ki} Kd:{self.kd}"
        )

        return SetParametersResult(successful=True)


# main
def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

