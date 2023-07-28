#!/usr/bin/env python

import rospy
import csv
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates

class camera_state:

    def __init__(self):
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        self.model_state = ModelState()
        self.model_state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_state_callback)
        
        # Load the CSV file
        with open('trajectory.csv', 'r') as file:
            reader = csv.reader(file)
            self.trajectory = list(reader)

        # Initialize the current row index
        self.current_row = 0

        # Initialize elapsed time for the transition
        self.elapsed_time = 0

        # Set the desired duration for the transitions (in seconds)
        self.transition_duration = 3  # Change this value according to your requirement



    def reset_to_home(self):
        # Set the model's position and velocity to the home position
        self.model_state.model_name = 'multirotor'
        self.model_state.pose.position.x = 0
        self.model_state.pose.position.y = 0
        self.model_state.pose.position.z = 0
        self.model_state.twist.linear.x = 0
        self.model_state.twist.linear.y = 0
        self.model_state.twist.linear.z = 0
        self.model_state.reference_frame = 'camera_link'

        # Publish the model state
        self.pub.publish(self.model_state)


    # def spin(self):
    #     # if self.current_row >= len(self.trajectory):
    #     #     self.reset_to_home()
    #     # Get the current row from the trajectory
    #     row = self.trajectory[self.current_row]

    #     # Update the positions and velocities based on the current row
    #     self.model_state.model_name = 'multirotor'
    #     self.model_state.pose.position.x = float(row[0])
    #     self.model_state.pose.position.y = float(row[1])
    #     self.model_state.pose.position.z = float(row[2])
    #     self.model_state.twist.linear.x = float(row[3])
    #     self.model_state.twist.linear.y = float(row[4])
    #     self.model_state.twist.linear.z = float(row[5])
    #     self.model_state.reference_frame = 'camera_link'

    #     # Publish the model state
    #     self.pub.publish(self.model_state)

    #     # Increment the current row
    #     self.current_row += 1
    #     # If we've reached the end of the trajectory, loop back to the start
    #     if self.current_row >= len(self.trajectory):
    #         self.current_row = 0

    def spin(self):
        # If we've reached the end of the trajectory, reset to home
        if self.current_row >= len(self.trajectory):
            self.reset_to_home()
            return

        # Get the current row and next row from the trajectory
        current_row = self.trajectory[self.current_row]
        next_row = self.trajectory[(self.current_row + 1) % len(self.trajectory)]

        # Calculate the progress based on the elapsed time and the desired duration for the transition
        progress = min(self.elapsed_time / self.transition_duration, 1)

        # Calculate the intermediate position using lerp
        x = (1 - progress) * float(current_row[0]) + progress * float(next_row[0])
        y = (1 - progress) * float(current_row[1]) + progress * float(next_row[1])
        z = (1 - progress) * float(current_row[2]) + progress * float(next_row[2])

        # Update the positions and velocities based on the interpolated position
        self.model_state.model_name = 'multirotor'
        self.model_state.pose.position.x = x
        self.model_state.pose.position.y = y
        self.model_state.pose.position.z = z
        self.model_state.twist.linear.x = float(current_row[3])
        self.model_state.twist.linear.y = float(current_row[4])
        self.model_state.twist.linear.z = float(current_row[5])
        self.model_state.reference_frame = 'camera_link'

        # Publish the model state
        self.pub.publish(self.model_state)

        # If we've completed the transition, move to the next row
        if progress >= 1:
            self.current_row = (self.current_row + 1) % len(self.trajectory)
            self.elapsed_time = 0  # Reset the elapsed time for the next transition
        else:
            self.elapsed_time += 1  # Increment the elapsed time by the time step (assuming the time step is 1 second)


    def model_state_callback(self, data):
        a = data.pose[1].position.x
        print(a)
