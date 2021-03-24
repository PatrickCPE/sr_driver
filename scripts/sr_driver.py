#!/usr/bin/env python3
#Note this will be 'python' if running on ubuntu 16

import rospy
from std_msgs.msg import String
import rtde_io
import rtde_receive

class sr_driver():
    """
    Basic Soft Robotics Gripper Class Wrapper
    """
    def __init__(self):
        self.state = 'neutral'
        # Pressure's in 10ths of a psi for all pressure values
        self.pressure = 0
        self.max_pressure = 100
        self.min_pressure = -50

        self.gripper_sub = rospy.Subscriber("sr_gripper", String, self.desired_state_sub_cb)

        self.rtde_io = rtde_io.RTDEIOInterface("10.0.0.65")
        self.rtde_receive = rtde_receive.RTDEReceiveInterface("10.0.0.65")

    def desired_state_sub_cb(self, data):
        """
        Callback updates local desired state

        :param data: std_msgs.msg.String
        :return: none
        """
        self.state = self.validate_state(data.data)
        rospy.loginfo("Current Desired State: %s", data.data)
        rospy.loginfo("Current State: %s", self.state)
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

        pass

    def validate_state(self, desired_state):
        """
        Ensures that the newly given state is valid. Note: Cap sensitive

        :param desired_state: String containing the desired state
        :return: new_state: Returns prior state if invalid desired_state
        """
        if (desired_state == 'neutral'):
            return 'neutral'
        elif (desired_state == 'closed'):
            return 'close'
        elif (desired_state == 'open'):
            return 'open'
        else:
            rospy.logerr("Invalid State Input: %s\tState will remain: %s", desired_state, self.state)
            return self.state   #Returns the previous state, no update made



if __name__ == '__main__':
    rospy.init_node('sr_gripper', anonymous=True)
    my_driver = sr_driver()
    rospy.spin()

