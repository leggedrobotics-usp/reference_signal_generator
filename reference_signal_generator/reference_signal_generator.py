import rclpy
from rclpy.node import Node

from ref_signal_srvs.srv import Step
from ref_signal_srvs.srv import Ramp
from ref_signal_srvs.srv import Wave
from ref_signal_srvs.srv import Chirp

from std_srvs.srv import Trigger
from std_msgs.msg import Float64

from scipy import signal
import math

class RefSignalServer(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('reference_signal_server')
        # Declare the topic_name parameter
        self.declare_parameter('topic_name', 'reference_signal')
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        # Create the services for each signal type
        self.step_service = self.create_service(Step, topic_name + '/step', self.step_callback)
        self.ramp_service = self.create_service(Ramp, topic_name + '/ramp', self.ramp_callback)
        self.square_service = self.create_service(Wave, topic_name +'/square', self.square_callback)
        self.sine_service = self.create_service(Wave, topic_name +'/sine', self.sine_callback)
        self.triangle_service = self.create_service(Wave, topic_name +'/triangle', self.triangle_callback)
        self.sawtooth_service = self.create_service(Wave, topic_name +'/sawtooth', self.sawtooth_callback)
        self.chirp_service = self.create_service(Chirp, topic_name +'/chirp', self.chirp_callback)
        # Create an additional service to stop the reference signal
        self.stop_service = self.create_service(Trigger, topic_name +'/stop', self.stop_callback)
        # Create a publisher with the parameterized topic name
        self.publisher = self.create_publisher(Float64, topic_name, 10)
        # Initialize variables
        self.msg = Float64()
        self.msg.data = 0.0
        self.publish_period = 0.0
        self.timer = self.create_timer(self.publish_period, self.publish_reference_signal)
        self.elapsed_time = 0.0
        self.total_time = 0.0
        self.signal_type = 'none'

    def step_callback(self, request, response):
        # Define signal type
        self.signal_type = 'step'
        # Read parameters from service call
        self.initial_value = request.initial_value
        self.final_value = request.final_value
        self.step_time = request.step_time
        self.publish_rate = request.publish_rate
        self.publish_period = 1/self.publish_rate
        self.total_time = request.total_time
        # Destroy the previous timer
        self.timer.destroy()
        # Start the timer
        self.elapsed_time = 0.0
        self.timer = self.create_timer(self.publish_period, self.publish_reference_signal)
        # Respond the service call
        response.success = True
        response.message = 'Step signal successfully started'
        return response
    
    def ramp_callback(self, request, response):
        # Define signal type
        self.signal_type = 'ramp'
        # Read parameters from service call
        self.initial_value = request.initial_value
        self.slope = request.slope
        self.start_time = request.start_time
        self.publish_rate = request.publish_rate
        self.publish_period = 1/self.publish_rate
        self.total_time = request.total_time
        # Destroy the previous timer
        self.timer.destroy()
        # Start the timer
        self.elapsed_time = 0.0
        self.timer = self.create_timer(self.publish_period, self.publish_reference_signal)
        # Respond the service call
        response.success = True
        response.message = 'Ramp signal successfully started'
        return response

    def square_callback(self, request, response):
        # Define signal type
        self.signal_type = 'square'
        # Call the waveform callback
        self.wave_callback(request)
        # Respond the service call
        response.success = True
        response.message = 'Square signal successfully started'
        return response
    
    def sine_callback(self, request, response):
        # Define signal type
        self.signal_type = 'sine'
        # Call the waveform callback
        self.wave_callback(request)
        # Respond the service call
        response.success = True
        response.message = 'Sine signal successfully started'
        return response

    def triangle_callback(self, request, response):
        # Define signal type
        self.signal_type = 'triangle'
        # Call the waveform callback
        self.wave_callback(request)
        # Respond the service call
        response.success = True
        response.message = 'Triangle signal successfully started'
        return response

    def sawtooth_callback(self, request, response):
        # Define signal type
        self.signal_type = 'sawtooth'
        # Call the waveform callback
        self.wave_callback(request)
        # Respond the service call
        response.success = True
        response.message = 'Sawtooth signal successfully started'
        return response
    
    # Generic wave callback (used by square, sine, triangle and sawtooth)
    # All four use the same Wave.srv service interface
    def wave_callback(self, request):
        # Read parameters from service call
        self.offset = request.offset
        self.amplitude = request.amplitude
        self.frequency = request.frequency
        self.publish_rate = request.publish_rate
        self.publish_period = 1/self.publish_rate
        self.total_time = request.total_time
        # Destroy the previous timer
        self.timer.destroy()
        # Start the timer
        self.elapsed_time = 0.0
        self.timer = self.create_timer(self.publish_period, self.publish_reference_signal)

    def chirp_callback(self, request, response):
        # Define signal type
        self.signal_type = 'chirp'
        # Read parameters from service call
        self.offset = request.offset
        self.amplitude = request.amplitude
        self.initial_frequency = request.initial_frequency
        self.target_frequency = request.target_frequency
        self.target_time = request.target_time
        self.publish_rate = request.publish_rate
        self.publish_period = 1/self.publish_rate
        self.total_time = request.total_time
        # Destroy the previous timer
        self.timer.destroy()
        # Start the timer
        self.elapsed_time = 0.0
        self.timer = self.create_timer(self.publish_period, self.publish_reference_signal)
        # Respond the service call
        response.success = True
        response.message = 'Chirp signal successfully started'
        return response

    def stop_callback(self, request, response):
        # Stop publishing
        self.timer.destroy()
        self.elapsed_time = float('inf')
        # Respond the service call
        response.success = True
        response.message = self.signal_type.capitalize() + ' signal successfully stopped'
        # Set signal_type to 'none'
        self.signal_type = 'none'
        return response

    def publish_reference_signal(self):
        if self.elapsed_time > self.total_time:
            # Stop publishing
            self.timer.destroy()
            self.elapsed_time = float('inf')
            self.signal_type = 'none'
        else:
            # Recognize the signal type
            match self.signal_type:
                # Generate the reference signal at current timestamp
                case'step':
                    if self.elapsed_time >= self.step_time:
                        self.msg.data = self.final_value
                    else:
                        self.msg.data = self.initial_value
                case 'ramp':
                    if self.elapsed_time >= self.start_time:
                        self.msg.data = self.initial_value + (self.elapsed_time - self.start_time)*self.slope
                    else:
                        self.msg.data = self.initial_value
                case 'square':
                    self.msg.data = self.offset + self.amplitude/2 * math.copysign(1, math.sin(2 * math.pi * self.frequency * self.elapsed_time))
                case 'sine':
                    self.msg.data = self.offset + self.amplitude/2 * math.sin(2 * math.pi * self.frequency * self.elapsed_time)
                case 'triangle':
                    self.msg.data = self.offset + self.amplitude/2 * signal.sawtooth(2 * math.pi * self.frequency * self.elapsed_time, width=0.5)
                case 'sawtooth':
                    self.msg.data = self.offset + self.amplitude/2 * signal.sawtooth(2 * math.pi * self.frequency * self.elapsed_time, width=1)
                case 'chirp':
                    self.msg.data = self.offset + self.amplitude/2 * signal.chirp(self.elapsed_time, self.initial_frequency, self.target_time, self.target_frequency)
                case _:
                    pass
            # Publish the message
            self.publisher.publish(self.msg)
            # Increment the elapsed time
            self.elapsed_time += self.publish_period

def main():
    rclpy.init()
    reference_signal_server = RefSignalServer()
    rclpy.spin(reference_signal_server)
    reference_signal_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()