#!/usr/bin/env python3
# Note this will be 'python' if running on ubuntu 16

import rospy
from std_msgs.msg import String
import rtde_io
import rtde_receive


class sr_driver:
    """
    Basic Soft Robotics Gripper Class Wrapper - controls Gripper state machine via received state command and pressure
    reading from the UR robot itself.

    If /sr_gripper receives "closed" system closes, if it receives "open" system opens
    """

    def __init__(self):
        self.state = 'neutral'
        self.previous_state = 'neutral'

        self.gripper_sub = rospy.Subscriber("sr_gripper", String, self.desired_state_sub_cb)

        # Testing IPs
        self.rtde_io = rtde_io.RTDEIOInterface("10.0.0.65")
        self.rtde_receive = rtde_receive.RTDEReceiveInterface("10.0.0.65")

        # Real System IPs
        # self.rtde_io = rtde_io.RTDEIOInterface("10.0.9.3")
        # self.rtde_receive = rtde_receive.RTDEReceiveInterface("10.0.9.3")

        self.voltage = self.rtde_receive.getStandardAnalogInput0()

        # ADC values for the gripper states
        self.release_volt = 2.7
        self.close_volt = 3.7

    def desired_state_sub_cb(self, data):
        """
        Callback updates local desired state

        :param data: std_msgs.msg.String
        :return: none
        """
        self.previous_state = self.state
        self.state = self.validate_state(data.data)

        rospy.loginfo("Current Desired State: %s", data.data)
        rospy.loginfo("Current State: %s", self.state)

        # Ensure you're updating analog read from proper analog pin for your gripper
        self.voltage = self.rtde_receive.getStandardAnalogInput0()
        self.update_robot_state()

    def update_robot_state(self):
        """
        Updates the digital outputs on the robot according to the desired state and the pressure sensed via the analog
        input pins. Relies on the max & min pressures and the current pressure. Decision made based upon current
        pressure and the current set state

        :return: none
        """
        if (self.state == 'open') and (self.previous_state == 'open'):  # Remain Open
            self.codrive_stop()
        elif (self.state == 'open') and (self.previous_state != 'open'):  # Open Gripper
            self.codrive_open()
        elif (self.state == 'closed') and (self.previous_state == 'closed'):  # Remain Closed
            self.codrive_stop()
        elif (self.state == 'closed') and (self.previous_state != 'closed'):  # Close Gripper
            self.codrive_close()

    def validate_state(self, desired_state):
        """
        Ensures that the newly given state is valid. Note: Cap sensitive

        :param desired_state: String containing the desired state
        :return: new_state: Returns prior state if invalid desired_state
        """
        if desired_state == 'closed':
            return 'closed'
        elif desired_state == 'open':
            return 'open'
        else:
            rospy.logerr("Invalid State Input: %s\tState will remain: %s", desired_state, self.state)
            return self.state  # Returns the previous state, no update made

    # Consult state diagram for info on these 3 functions
    def codrive_stop(self):
        """
        freeze gripper in current state

        :return: none
        """
        self.rtde_io.setStandardDigitalOut(0, False)
        self.rtde_io.setStandardDigitalOut(1, False)
        self.rtde_io.setStandardDigitalOut(2, False)

    def codrive_open(self):
        """
        open gripper - watchdog timer for pump system = counter value * 1 ms

        :return: none
        """
        self.rtde_io.setStandardDigitalOut(0, False)
        self.rtde_io.setStandardDigitalOut(1, True)
        self.rtde_io.setStandardDigitalOut(2, True)
        counter = 0
        while (self.voltage > self.release_volt) and (counter != 100):
            counter = counter + 1
            rospy.sleep(0.01)
            self.voltage = self.rtde_receive.getStandardAnalogInput0()
        self.codrive_stop()

    def codrive_close(self):
        """
        close gripper - watchdog timer for pump system = counter value * 1 ms

        :return: none
        """
        self.rtde_io.setStandardDigitalOut(0, True)
        self.rtde_io.setStandardDigitalOut(1, False)
        self.rtde_io.setStandardDigitalOut(2, True)
        counter = 0
        while (self.voltage < self.close_volt) and (counter != 100):
            counter = counter + 1
            rospy.sleep(0.01)
            self.voltage = self.rtde_receive.getStandardAnalogInput0()
        self.codrive_stop()


if __name__ == '__main__':
    rospy.init_node('sr_gripper', anonymous=True)
    my_driver = sr_driver()
    rospy.spin()
