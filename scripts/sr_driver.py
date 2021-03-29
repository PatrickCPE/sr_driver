#!/usr/bin/env python3
# Note this will be 'python' if running on ubuntu 16

import rospy
from std_msgs.msg import String
import rtde_io
import rtde_receive


class sr_driver():
    """
    Basic Soft Robotics Gripper Class Wrapper - controls Gripper state machine via recieved state command and pressure
    reading from the UR robot itself
    """

    def __init__(self):
        self.state = 'neutral'
        self.previous_state = 'neutral'
        # Pressure's in 10ths of a psi for all pressure values
        #TODO remove pressures if no longer used
        self.pressure = 0
        self.max_pressure = 100
        self.min_pressure = -50

        self.gripper_sub = rospy.Subscriber("sr_gripper", String, self.desired_state_sub_cb)

        self.rtde_io = rtde_io.RTDEIOInterface("10.0.0.65")
        self.rtde_receive = rtde_receive.RTDEReceiveInterface("10.0.0.65")


        # ADC values for the gripper states
        self.voltage = self.rtde_receive.getStandardAnalogInput0()
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
        # TODO add the constant, grip volt is what you need
        self.voltage = self.rtde_receive.getStandardAnalogInput0()
        # self.pressure = self.rtde_receive.getStandardAnalogInput0()
        self.update_robot_state()

    def update_robot_state(self):
        """
        Updates the digital outputs on the robot according to the desired state and the pressure sensed via the analog
        input pins. Relies on the max & min pressures and the current pressure. Decision made based upon current
        pressure and the current set state

        :return: none
        """
        '''
        basic functions to use
        pin_7_state = self.rtde_receive.getDigitalOutState(7)  #Read the value from the pin, used to confirm pin status
        self.rtde_io.setStandardDigitalOut(7, True)  #Sets pin 7 to high
        self.rtde_io.setAnalogOutputCurrent(1, 0.25) #Pin number than output current limit
        self.pressure = (constant factor) * (self.rtde_receive.GetStandardAnalogInput0())    #get the ADC value and
                                                                                             #convert to PSI
        '''
        if (self.state == 'open') and (self.previous_state == 'open'):  # Remain Open
            self.codrive_stop()
        elif (self.state == 'open') and (self.previous_state != 'open'):  # Open Gripper
            self.codrive_open()
        elif (self.state == 'closed') and (self.previous_state == 'closed'):  # Remain Closed
            self.codrive_stop()
        elif (self.state == 'closed') and (self.previous_state != 'closed'):  # Close Gripper
            self.codrive_close()

        # Tolerances are 5/10th of a psi for the neutral state
        # TODO Either strip this functionality out or implement custom logic for it or calc voltage for it
        elif (self.state == 'neutral') and (not ((-5 < self.pressure) and (self.pressure < 5))):  # Move to Neutral
            self.codrive_stop()
            pass    # TODO REMOVE OR DONT
        elif (self.state == 'neutral') and ((-5 < self.pressure) and (self.pressure < 5)):  # Remain Neutral
            self.codrive_stop()
            pass    # TODO REMOVE OR DONT

    def validate_state(self, desired_state):
        """
        Ensures that the newly given state is valid. Note: Cap sensitive

        :param desired_state: String containing the desired state
        :return: new_state: Returns prior state if invalid desired_state
        """
        if (desired_state == 'neutral'):
            return 'neutral'
        elif (desired_state == 'closed'):
            return 'closed'
        elif (desired_state == 'open'):
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
        open gripper

        :return: none
        """
        self.rtde_io.setStandardDigitalOut(0, False)
        self.rtde_io.setStandardDigitalOut(1, True)
        self.rtde_io.setStandardDigitalOut(2, True)
        while self.voltage > self.release_volt:
            rospy.sleep(0.01)
            self.voltage = self.rtde_receive.getStandardAnalogInput0()

    def codrive_close(self):
        """
        close gripper

        :return:
        """
        self.rtde_io.setStandardDigitalOut(0, True)
        self.rtde_io.setStandardDigitalOut(1, False)
        self.rtde_io.setStandardDigitalOut(2, True)
        while self.voltage < self.close_volt:
            rospy.sleep(0.01)
            self.voltage = self.rtde_receive.getStandardAnalogInput0()

    #TODO determine if I want to do this this way
    def codrive_neutral(self):
        pass


if __name__ == '__main__':
    rospy.init_node('sr_gripper', anonymous=True)
    my_driver = sr_driver()
    rospy.spin()
