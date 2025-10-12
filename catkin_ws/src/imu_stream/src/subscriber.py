#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + f"Orientation {data.orientation.x} {data.orientation.y} {data.orientation.z} {data.orientation.w}")
    rospy.loginfo(rospy.get_caller_id() + f"Angular velocity {data.angular_velocity.x} {data.angular_velocity.y} {data.angular_velocity.z}")
    rospy.loginfo(rospy.get_caller_id() + f"Linear acceleration {data.linear_acceleration.x} {data.linear_acceleration.y} {data.linear_acceleration.z}")

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("imu", Imu, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
