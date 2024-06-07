import rclpy
from rclpy.node import Node

from reference_signal_srvs.srv import Setup

from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray

from scipy import signal
import math
import array

def check_array_length_consistency(arrays):
    if not arrays:
        return True
    else:
        length = len(arrays[0])
        for arr in arrays:
            if len(arr)!= length:
                return False
        return True
    
class RefSignalServer(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('reference_signal_server')
        # Declare the topic_name parameter
        self.declare_parameter('topic_name', 'reference_signal')
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        # Create services for starting and stopping the reference signal
        self.start_service = self.create_service(Setup, topic_name + '/start', self.start_callback)
        self.stop_service = self.create_service(Trigger, topic_name +'/stop', self.stop_callback)
        # Create a publisher with the parameterized topic name
        self.publisher = self.create_publisher(Float64MultiArray, topic_name, 10)
        # Initialize variables
        self.msg = Float64MultiArray()
        self.publish_period = 0.0
        self.timer = self.create_timer(self.publish_period, self.publish_reference_signal)
        self.elapsed_time = float('inf')
        self.total_time = 0.0
        self.signal_type = []

    def start_callback(self, request, response):
        # Check if provided signal types are valid
        if any(s not in set(['step', 'ramp', 'sine', 'square', 'triangle', 'sawtooth', 'chirp']) for s in request.signal_type):
            response.success = False
            response.message = 'Invalid signal type detected'
            return response
        # Check if all parameter arrays have the same length
        elif not check_array_length_consistency([request.signal_type,
                                                 request.initial_value,
                                                 request.final_value,
                                                 request.start_time,
                                                 request.end_time,
                                                 request.slope,
                                                 request.offset,
                                                 request.amplitude,
                                                 request.frequency,
                                                 request.phase,
                                                 request.initial_frequency,
                                                 request.target_frequency,
                                                 request.target_time]):
            response.success = False
            response.message = 'Parameter array length inconsistency detected'
            return response
        else:
            # Define signal type
            self.signal_type = request.signal_type
            # Check if parameter arrays match the length of signal_type array
            # Read signal parameters from service call
            self.initial_value = request.initial_value
            self.final_value = request.final_value
            self.start_time = request.start_time
            self.end_time = request.end_time
            self.slope = request.slope
            self.offset = request.offset
            self.amplitude = request.amplitude
            self.frequency = request.frequency
            self.phase = request.phase
            self.initial_frequency = request.initial_frequency
            self.target_frequency = request.target_frequency
            self.target_time = request.target_time
            self.publish_rate = request.publish_rate
            self.publish_period = 1/self.publish_rate
            self.total_time = float(request.total_time)
            # Initialize msg.data
            self.msg.data = array.array('d', [0.0] * len(self.signal_type))
            # Destroy the previous timer
            self.timer.destroy()
            # Start the timer
            self.elapsed_time = 0.0
            self.timer = self.create_timer(self.publish_period, self.publish_reference_signal)
            # Respond the service call
            # Respond the service call
            response.success = True
            response.message = 'Reference signal successfully started'
            return response
    
    def stop_callback(self, request, response):
        # Stop publishing
        self.timer.destroy()
        self.elapsed_time = float('inf')
        # Respond the service call
        response.success = True
        response.message = 'Reference signal successfully stopped'
        return response

    def publish_reference_signal(self):
        if self.elapsed_time > self.total_time:
            # Stop publishing
            self.timer.destroy()
            self.elapsed_time = float('inf')
        else:
            # Check each reference input
            for i in range(len(self.signal_type)):
                # Recognize the signal type
                match self.signal_type[i]:
                    case 'step':
                        if self.elapsed_time >= self.start_time[i] and self.elapsed_time < self.end_time[i]:
                                self.msg.data[i] = self.final_value[i]
                        else:
                            self.msg.data[i] = self.initial_value[i]
                    case 'ramp':
                        if self.elapsed_time < self.start_time[i]:
                            self.msg.data[i] = self.initial_value[i]
                        elif self.elapsed_time < self.end_time[i]:
                            self.msg.data[i] = self.initial_value[i] + (self.elapsed_time - self.start_time[i])*self.slope[i]
                        else:
                            self.msg.data[i] = self.initial_value[i] + (self.end_time[i] - self.start_time[i])*self.slope[i]
                    case 'sine':
                        if self.elapsed_time >= self.start_time[i] and self.elapsed_time < self.end_time[i]:
                            self.msg.data[i] = self.offset[i] + self.amplitude[i]/2 * math.sin(2 * math.pi * self.frequency[i] * (self.elapsed_time - self.start_time[i]) + math.pi / 180 * self.phase[i])
                        else:
                            self.msg.data[i] = self.offset[i]
                    case 'square':
                        if self.elapsed_time >= self.start_time[i] and self.elapsed_time < self.end_time[i]:
                            self.msg.data[i] = self.offset[i] + self.amplitude[i]/2 * math.copysign(1, math.sin(2 * math.pi * self.frequency[i] * (self.elapsed_time - self.start_time[i]) + math.pi / 180 * self.phase[i]))
                        else:
                            self.msg.data[i] = self.offset[i]
                    case 'triangle':
                        if self.elapsed_time >= self.start_time[i] and self.elapsed_time < self.end_time[i]:
                            self.msg.data[i] = self.offset[i] + self.amplitude/2 * signal.sawtooth(2 * math.pi * self.frequency[i] * (self.elapsed_time - self.start_time[i]) + math.pi / 180 * self.phase[i], width=0.5)
                        else:
                            self.msg.data[i] = self.offset[i]
                    case 'sawtooth':
                        if self.elapsed_time >= self.start_time[i] and self.elapsed_time < self.end_time[i]:
                            self.msg.data[i] = self.offset[i] + self.amplitude/2 * signal.sawtooth(2 * math.pi * self.frequency[i] * (self.elapsed_time - self.start_time[i]) + math.pi / 180 * self.phase[i], width=1)
                        else:
                            self.msg.data[i] = self.offset[i]
                    case 'chirp':
                        if self.elapsed_time >= self.start_time[i] and self.elapsed_time < self.end_time[i]:
                            self.msg.data[i] = self.offset[i] + self.amplitude[i]/2 * signal.chirp(self.elapsed_time - self.start_time[i], self.initial_frequency[i], self.target_time[i], self.target_frequency[i], phi=self.phase[i])
                        else:
                            self.msg.data[i] = self.offset[i]
                    case _:
                        pass
            # Publish the reference signal
            self.publisher.publish(self.msg)
            # Update the elapsed time
            self.elapsed_time += self.publish_period

def main():
    rclpy.init()
    reference_signal_server = RefSignalServer()
    rclpy.spin(reference_signal_server)
    reference_signal_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()