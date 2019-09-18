#! /usr/bin/env python
import rospy
import sys
from numpy import sign
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

        ## Flag for the method to apply (nonlinear: 1, linear: any other value)
        self.nonlinear_flag = rospy.get_param('nonlinear', 0)
        if self.nonlinear_flag == 1:
            rospy.loginfo("USING NONLINEAR MAPPING")
        else:
            rospy.loginfo("USING LINEAR MAPPING")

        ## Mapping coefficient for linear velocity
        self.linear_coefficent = rospy.get_param ('linear_coefficient', 0.05)
        ## Mapping coefficient for angular velocity
        self.angular_coefficent = rospy.get_param ('angular_coefficient', 0.05)

        ## Halt threshold: if the linear velocity is less than this value, the mower doesn't move
        self.halt_threshold = rospy.get_param ('halt_threshold', 0)

        ## Saturation values for x and y acceleration from the smartwatch
        self.x_acc_saturation = rospy.get_param ('x_acceleration_saturation', 8)
        self.y_acc_saturation = rospy.get_param ('y_acceleration_saturation', 6)

        ## The two saturation and the threshold values must be positive.
        #  The halt threshold must be less than both values.
        #  This check if performed only if the nonlinear flag is high
        if self.nonlinear_flag == 1 and ( \
           self.halt_threshold <= 0 or \
           self.x_acc_saturation <= 0 or \
           self.y_acc_saturation <= 0 or \
           self.halt_threshold >= self.x_acc_saturation or \
           self.halt_threshold >= self.y_acc_saturation):
            rospy.logfatal("INVALID LAUNCHFILE PARAMETERS VALUES!")
            sys.exit(1)

        ## If the nonlinear flag is high, the two mapping coefficients are updated
        #  0.4 and 0.3 are taken as the maximum linear and angular velocities possible
        if self.nonlinear_flag == 1:
            self.linear_coefficent = 0.4 / (self.x_acc_saturation - self.halt_threshold)
            self.angular_coefficent = 0.3 / (self.y_acc_saturation - self.halt_threshold)

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
        #self.init()
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

    ## Nonlinear mapping function for the linear velocity
    #
    #  @param self The object pointer
    def v_nonlinear_mapping(self):
        x_acc = self.last_acc[0]
        if abs(x_acc) < self.halt_threshold:
            return 0
        elif abs(x_acc) >= self.x_acc_saturation:
            return 0.4 * sign(x_acc)
        else:
            return self.linear_coefficent * (x_acc - self.halt_threshold * sign(x_acc))

    ## Nonlinear mapping function for the angular velocity
    #
    #  @param self The object pointer
    def w_nonlinear_mapping(self):
        y_acc = self.last_acc[1]
        if abs(y_acc) < self.halt_threshold:
            return 0
        elif abs(y_acc) >= self.y_acc_saturation:
            return 0.3 * sign(y_acc)
        else:
            return self.angular_coefficent * (y_acc - self.halt_threshold * sign(y_acc))

    ## Mapping of acceleration to robot angular and linear velocities
    #
    #  @param self The object pointer
    def update(self):
        if rospy.is_shutdown():
            return
        twist = Twist()
        # LINEAR METHOD
        if self.nonlinear_flag != 1:
            twist.linear.x = self.last_acc[0] * self.linear_coefficent
            twist.angular.z = self.last_acc[1] * self.angular_coefficent
        # NONLINEAR METHOD
        else:
            twist.linear.x = self.v_nonlinear_mapping()
            twist.angular.z = self.w_nonlinear_mapping()
        self.pub_twist.publish(twist)


def main():
    rospy.init_node('gb_controller', disable_signals=True)
    controller = GestureController()
    controller.run()


if __name__ == '__main__':
    main()
