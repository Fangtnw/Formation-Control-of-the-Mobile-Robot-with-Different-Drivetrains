import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
import math
import sys
import os
import time
from pynput import keyboard
import ament_index_python
import subprocess
import threading

message = """
Ackermann              DiffDrive
---------------------------
Moving around:
          ^                    ^ 
          w                    i 
   < a    s    d >      < a    s    d > 
---------------------------
w : increase speed
s : decrease speed
a : turn left
d : turn right

"""


class CommandPublisher(Node):
    def __init__(self):
        super().__init__('coop_keyboard_teleop')
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/car/cmd_ackermann', 10)
        self.diffdrive_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ackermann_speed = 0.0
        self.ackermann_steering_angle = 0.0
        self.diffdrive_linear = 0.0
        self.diffdrive_angular = 0.0
        self.lines = []
        self.key_press_times = {}  # Store the start time of each key press
        self.active_keys = set()
        self.lock = threading.Lock()
        print(message)

    def on_press(self, key):
        try:
            received_key = key.char
            self.lock.acquire()
            self.active_keys.add(received_key.lower())
            # Set the initial time when the key is pressed
            if received_key.lower() not in self.key_press_times:
                self.key_press_times[received_key.lower()] = time.time()
            self.update_velocities()
            self.lock.release()
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            received_key = key.char
            self.lock.acquire()
            self.active_keys.remove(received_key.lower())
            self.key_press_times[received_key.lower()] = None
            self.update_velocities()
            self.lock.release()
        except AttributeError:
            pass

    def update_velocities(self):
        current_time = time.time()
        if not self.active_keys:
            self.ackermann_speed = 0.0
            self.diffdrive_linear = 0.0
            self.diffdrive_angular = 0.0
            self.ackermann_steering_angle = 0.0
            return

        # Set initial velocities to zero
        self.ackermann_speed = 0.0
        self.diffdrive_linear = 0.0
        self.diffdrive_angular = 0.0
        # Update velocities based on the pressed keys
        for key in self.active_keys:
            if key == 'w':
                self.ackermann_speed = self.calculate_ramped_velocity(-0.3, current_time, key)
            elif key == 'd':
                self.ackermann_steering_angle = self.calculate_ramped_velocity(0.6, current_time, key)
            elif key == 's':
                self.ackermann_speed = self.calculate_ramped_velocity(0.3, current_time, key)
            elif key == 'a':
                self.ackermann_steering_angle = self.calculate_ramped_velocity(-0.6, current_time, key)
            elif key == 'i':  # Up arrow key
                self.diffdrive_linear = self.calculate_ramped_velocity(0.3, current_time, key)
            elif key == 'k':  # Down arrow key
                self.diffdrive_linear = self.calculate_ramped_velocity(-0.3, current_time, key)
            elif key == 'j':  # Left arrow key
                self.diffdrive_angular = self.calculate_ramped_velocity(0.5, current_time, key)
            elif key == 'l':  # Right arrow key
                self.diffdrive_angular = self.calculate_ramped_velocity(-0.5, current_time, key)
            elif key == 'r':
                self.ackermann_steering_angle = 0.0
            elif key == 'o':
                self.diffdrive_angular = 0.0
                self.diffdrive_linear = 0.0
            elif key == 'q':
                self.ackermann_speed = 0.0
                self.ackermann_steering_angle = 0.0

        
    def calculate_ramped_velocity(self, target_velocity, current_time, key):
        ramp_duration = 0.1  # Duration of the ramp in seconds (from 0 to target vel)
        initial_velocity = 0.0

        if key not in self.key_press_times or self.key_press_times[key] is None:
            # Set the initial time when the key is pressed
            self.key_press_times[key] = current_time

        acceleration = (target_velocity - initial_velocity) / ramp_duration
        elapsed_time = current_time - self.key_press_times[key]

        if elapsed_time <= ramp_duration:
            return initial_velocity + (acceleration * elapsed_time)
        else:
            return target_velocity
        

    def timer_callback(self):
        self.publish_commands()

    def publish_commands(self):
        self.lock.acquire()
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.drive.steering_angle = self.ackermann_steering_angle
        ackermann_msg.drive.speed = self.ackermann_speed

        diffdrive_msg = Twist()
        diffdrive_msg.linear.x = self.diffdrive_linear
        diffdrive_msg.angular.z = self.diffdrive_angular

        ackermann_messages = "Ackermann - speed: " + str(self.ackermann_speed) + " steering_angle: " + str(
            self.ackermann_steering_angle)
        diffdrive_messages = "DiffDrive - linear: " + str(self.diffdrive_linear) + " angular: " + str(
            self.diffdrive_angular)
        print(ackermann_messages)
        print(diffdrive_messages)
        self.lines.append(ackermann_messages)
        self.ackermann_publisher.publish(ackermann_msg)
        self.diffdrive_publisher.publish(diffdrive_msg)
        if len(self.lines) > 1:
            self.lines.clear()
            os.system('cls' if os.name == 'nt' else 'clear')
            print(message)
            print(ackermann_messages)
            print(diffdrive_messages)
        self.lock.release()


def main(args=None):
    rclpy.init(args=args)
    command_publisher = CommandPublisher()

    def set_non_blocking_input():
        print("Starting keyboard listener...")
        with keyboard.Listener(on_press=command_publisher.on_press, on_release=command_publisher.on_release) as listener:
            listener.join()

    thread = threading.Thread(target=set_non_blocking_input)
    thread.daemon = True
    thread.start()

    rclpy.spin(command_publisher)
    command_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
