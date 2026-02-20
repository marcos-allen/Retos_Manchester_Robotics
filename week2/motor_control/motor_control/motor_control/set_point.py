# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.srv import SetProcessBool

#Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')

        # Retrieve sine wave parameters
        self.amplitude = 2.0
        self.omega  = 1.0
        self.system_running = False
        
        # parametros para crear señales difernetes 
        self.declare_parameter("signal_type", "sine")
        self.signal_type = self.get_parameter("signal_type").value
        #change the parameter 
        self.add_on_set_parameters_callback(self.parameter_callback)

        #Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)
        timer_period = 0.02 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)

        #Create a service client for /EnableProcess
        self.cli = self.create_client(SetProcessBool, 'EnableProcess')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        #Create a messages and variables to be used
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        self.get_logger().info("SetPoint Node Started \U0001F680")

        self.send_request(True)

    # Timer Callback: Generate and Publish Sine Wave Signal
    def timer_cb(self):

        if not self.system_running:
            return  # Stop processing if simulation is not running
        
        #Calculate elapsed time
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds/1e9
        # Generate sine wave signal
        #self.signal_msg.data = self.amplitude * np.sin(self.omega * elapsed_time)
        if self.signal_type == "sine":
            value = self.amplitude * np.sin(self.omega * elapsed_time)

        elif self.signal_type == "step":
            value = self.amplitude

        elif self.signal_type == "square":
            value = self.amplitude * np.sign(np.sin(self.omega * elapsed_time))

        elif self.signal_type == "ramp":
            value = self.amplitude * (elapsed_time % 10) / 10

        else:
            value = 0.0

        self.signal_msg.data = float(value)
        # Publish the signal
        self.signal_publisher.publish(self.signal_msg)
    
    def send_request(self, enable: bool):
        """Send a request to start or stop the simulation."""
        request = SetProcessBool.Request()
        request.enable = enable

        future = self.cli.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        """Process the service response."""
        try:
            response = future.result()
            if response.success:
                self.system_running = True
                self.get_logger().info(f'Success: {response.message}')
            else:
                self.simulation_running = False
                self.get_logger().warn(f'Failure: {response.message}')
        except Exception as e:
            self.simulation_running = False
            self.get_logger().error(f'Service call failed: {e}')
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == "signal_type":
                self.signal_type = param.value
                self.get_logger().info(f"Signal type changed to: {self.signal_type}")
        return SetParametersResult(successful=True)

#Main
def main(args=None):
    rclpy.init(args=args)

    set_point = SetPointPublisher()

    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()
