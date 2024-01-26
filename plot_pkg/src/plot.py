#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np
import time

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')
        # self.subscription1 = self.create_subscription(Float64, '/sinus', self.listener_callback1, 10)
        self.subscription1 = self.create_subscription(JointState, '/joint_states', self.listener_callback1, 10)
        self.subscription2 = self.create_subscription(Float64MultiArray, '/velocity_controller/commands', self.listener_callback2, 10)
        plt.ion() # Turn on interactive mode
        fig, self.ax = plt.subplots()
        self.ax.legend(['Sinus', 'Cosinus'])
        self.line1, = self.ax.plot([], [], 'r-') # Initialize an empty line
        self.line2, = self.ax.plot([], [], 'b-') # Initialize another empty line
        self.ax.set_xlim(0, 10) # Set initial x-axis limits
        self.ax.set_ylim(-2, 2) # Set y-axis limits to expect float values between 0 and 1
        self.second_data = 0


    def listener_callback1(self, msg):
        # print(f"Received data: {len(msg.position)}")
        # position = msg.position
        # velocity = msg.velocity
        if len(msg.position) > 1:
            self.plot_data(msg.position[2] , self.line1) # use the middle joint instead of the first
        else:
            self.plot_data(msg.position[0] , self.line1)

        self.plot_data(self.second_data, self.line2)

        # Rescale x-axis if needed
        if plt.xlim()[1] - self.line1.get_xdata()[-1] < 1:
            self.ax.set_xlim(plt.xlim()[0], plt.xlim()[1] + 10)

        if plt.xlim()[1] - self.line2.get_xdata()[-1] < 1:
            self.ax.set_xlim(plt.xlim()[0], plt.xlim()[1] + 10)

    def listener_callback2(self, msg):
        # print(f"Received data: {msg.data}")

        self.second_data = msg.data[0]


    # Function to update the plot
    def update_line(self, line, new_data_x, new_data_y):
        line.set_xdata(np.append(line.get_xdata(), new_data_x))
        line.set_ydata(np.append(line.get_ydata(), new_data_y))
        plt.draw()

    # Function to plot data
    def plot_data(self, msg, line):
        # Assuming 'msg' is a new data point and 'line' is the line object
        current_time = plt.xlim()[1]
        self.update_line(line, current_time, msg)
        plt.pause(0.001)  # Pause to update the plot



def main(args=None):
    rclpy.init(args=args)
    data_subscriber = DataSubscriber()
    rclpy.spin(data_subscriber)
    data_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
