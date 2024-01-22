#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class SinusPublisher(Node):
   def __init__(self):
      super().__init__('sinus_publisher')
      self.publisher_ = self.create_publisher(Float64, '/sinus', 10)
      timer_period = 0.05 # seconds
      self.timer = self.create_timer(timer_period, self.timer_callback)
      self.x = 0

   def timer_callback(self):
      # Increase the value by 0.1 rad
      self.x += 0.1
      # Multiply the sinus value by 10
      # so it's nicer for the plot example
      message = Float64()
      message.data = math.sin(self.x) * 10
      self.publisher_.publish(message)


def main(args=None):
   rclpy.init(args=args)

   sinus_publisher = SinusPublisher()

   rclpy.spin(sinus_publisher)

   sinus_publisher.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()