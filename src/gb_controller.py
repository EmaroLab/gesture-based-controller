#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16


class GestureController(object):

    # ROS initialization 
    def init(self):
        self.update_rate = 10   # Node frquency (Hz)

        # Setup publishers & subscriber
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_mode = rospy.Publisher('/cmd_mode', UInt16, queue_size=1)
        rospy.Subscriber('/inertial', Imu, self.callback_continuos_control)

    def callback_continuos_control(self, data):
        x = data.linear_acceleration.x
        y = data.linear_acceleration.y
        z = data.linear_acceleration.z
        self.update(x, y, z)

    # Controller starter
    def run(self):
        self.init()
        r = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            r.sleep()

    # Shutdown handler
    def reset(self):
        print "\n"
        rospy.loginfo("RESETTING VELOCITY COMMANDS ON SHUTDOWN")
        self.update(0, 0, 0)

    # Mapping of acceleration to robot angular and linear velocities
    def update(self, x, y, z):
        if rospy.is_shutdown():
            return
        twist = Twist()
        twist.linear.x = x * 0.05
        twist.angular.z = y * 0.05
        self.pub_twist.publish(twist)


def main():
    rospy.init_node('gb_controller')
    controller = GestureController()
    rospy.on_shutdown(controller.reset)
    controller.run()


if __name__ == '__main__':
    main()
