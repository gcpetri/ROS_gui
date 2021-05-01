#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64, String

def control_type(data):
    rospy.loginfo(rospy.get_caller_id() + "control_type: %s", data.data)

def control_mode(data):
    rospy.loginfo(rospy.get_caller_id() + "control_mode: %f", data.data)

def listener():
    print('Starting CAN_controller status monitoring widget')
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/can_bus/can_control_type', String, control_type)
    rospy.Subscriber('/can_bus/vehicle_control_mode', Int64, control_mode)
    rospy.spin()

if __name__ == '__main__':
    listener()


