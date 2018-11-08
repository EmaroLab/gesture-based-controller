#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16

## Gesture Controller class subscribes to the "/inertial" topic and publishes to "/cmd_vel" the velocity commands generated using wrist linear accelerations.
class GestureController(object):
       
    ## Initializer function
    #
    #  @param self The object pointer
    def __init__(self):
        ## Node frequency (Hz) 
        self.update_rate = 10   

        ## Mapping coefficient for linear velocity
        self.linear_coefficent = rospy.get_param ('linear_coefficient', 0.05)
        ## Mapping coefficient for angular velocity
        self.angular_coefficent = rospy.get_param ('angular_coefficient', 0.05)

        ## Stores the last acceleration received by the node
        self.last_acc = [0,0,0]
        ## Time instant in which the last acceleration message arrived
        self.last_time = 0

        ## Velocity commands publisher
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        ## Control modality publisher
        self.pub_mode = rospy.Publisher('/cmd_mode', UInt16, queue_size=1)
        rospy.Subscriber('/inertial', Imu, self.callback_continuos_control)

    ## Callback manager
    #
    # @param self The object pointer
    # @param data The ROS message received
    def callback_continuos_control(self,data):
        self.last_acc[0] = data.linear_acceleration.x
        self.last_acc[1] = data.linear_acceleration.y
        self.last_acc[2] = data.linear_acceleration.z
        self.last_time = data.header.stamp.secs

    ## Controller starter
    #
    #  @param self The object pointer
    def run(self):
        self.init()
        r = rospy.Rate(self.update_rate)
        while True:
            try:
                now = rospy.get_rostime()
                if (now.secs-self.last_time) < 1:
                    self.update()
                    r.sleep()
                else:
                    self.acceleration_reset()
                    self.update()
                    r.sleep()
            except KeyboardInterrupt:
                self.acceleration_reset()
                self.reset()
                break

    ## Reset velocities to zero
    #
    #  @param self The object pointer
    def acceleration_reset(self):
        
        self.last_acc = [0,0,0]
    
    ## Shutdown handler
    #
    #  @param self The object pointer
    def reset(self):
        print "\n"
        rospy.loginfo("RESETTING VELOCITY COMMANDS ON SHUTDOWN")
        self.update()

    ## Mapping of acceleration to robot angular and linear velocities
    #
    #  @param self The object pointer
    def update(self):
        if rospy.is_shutdown():
            return
        twist = Twist()
        twist.linear.x = self.last_acc[0] * self.linear_coefficent
        twist.angular.z = self.last_acc[1] * self.angular_coefficent
        self.pub_twist.publish(twist)


def main():
    rospy.init_node('gb_controller', disable_signals=True)
    controller = GestureController()
    controller.run()


if __name__ == '__main__':
    main()
