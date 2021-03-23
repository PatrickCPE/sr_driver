#!/usr/bin/env python3
#Note this will be 'python' if running on ubuntu 16


import rospy
from std_msgs.msg import String

class sr_driver():
    """
    Basic Soft Robotics Gripper Class Wrapper
    """
    def __init__(self):
        self.state = 'neutral'
        self.pressure = 0;      #Presure in 10ths of a psi for all pressure values
        self.max_pressure = 100;
        self.min_pressure = -50;
        
        self.gripper_sub = rospy.Subscriber("sr_gripper", String, self.desired_state_sub_cb)

    def desired_state_sub_cb(self, data):
        """
        Callback updates local desired state

        :param data: std_msgs.msg.String
        :return: none
        """
        self.state = self.validate_state(data.data)
        rospy.loginfo("Current Desired State: %s", data.data)
        rospy.loginfo("Current State: %s", self.state)

    def validate_state(self, desired_state):
        """
        Ensures that the newly given state is valid. Note: Cap sensitive

        :param desired_state: String containing the desired state
        :return: new_state: Returns prior state if invalid desired_state
        """
        if (desired_state == 'neutral'):
            return 'neutral'
        elif (desired_state == 'close'):
            return 'close'
        elif (desired_state == 'open'):\
            return 'open'
        else:
            rospy.logerr("Invalid State Input: %s\tState will remain: %s", desired_state, self.state)
            return self.state   #Returns the previous state, no update made


def update_state():
    pass

if __name__ == '__main__':
    rospy.init_node('sr_gripper', anonymous=True)
    my_driver = sr_driver()
    rospy.spin()

