#!/usr/bin/python

# drive


#!/usr/bin/python

"""
Class for low level control of owr car. It assumes ros-12cpwmboard has been
installed
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import donkeycar as dk
import time
import threading
from motor import PCA9685, PWMSteering, PWMThrottle



class ControlCar():
    controller = None

    def __init__(self):
        rospy.loginfo("Setting Up the Node...")
        rospy.init_node('donkey')

        steering_controller = PCA9685(cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
        self.steering = PWMSteering(controller=steering_controller,
                               left_pulse=cfg.STEERING_LEFT_PWM,
                               right_pulse=cfg.STEERING_RIGHT_PWM)

        throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
        self.throttle = PWMThrottle(controller=throttle_controller,
                               max_pulse=cfg.THROTTLE_FORWARD_PWM,
                               zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                               min_pulse=cfg.THROTTLE_REVERSE_PWM)

        # -- Save the time
        self._last_time_cmd_rcv = time.time()
        self._timeout_s = 5

    @property
    def is_controller_connected(self):
        print(time.time() - self._last_time_cmd_rcv)
        return (time.time() - self._last_time_cmd_rcv < self._timeout_s)

    def set_actuators_from_cmdvel(self, message):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        # -- Save the time
        self._last_time_cmd_rcv = time.time()
        # print('message.linear', message.linear)
        # print('message.angular', message.angular)

        # -- Convert vel into servo values
        self.throttle.run(message.linear.x)  # - : backward, + : forward
        self.steering.run(message.angular.z)  # - : left, + : right
        rospy.loginfo("Got a command v = %2.1f  s = %2.1f" % (message.linear.x, message.angular.z))
        # self.send_servo_msg()

    def set_actuators_idle(self):
        # -- Convert vel into servo values
        self.throttle.run(0)  # - : backward, + : forward
        self.steering.run(0)  # - : left, + : right
        rospy.loginfo("Setting actutors to idle")

    def setController(self, message):
        if message == 'keyboard':
            self.isKeyboard = True

    def run(self):
        # --- Set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            ros_sub_twist = rospy.Subscriber("/cmd_vel", Twist, self.set_actuators_from_cmdvel)
            ros_controller = rospy.Subscriber("/cmd_controller", String, self.setController)
            # rospy.loginfo("> Subscriber corrrectly initialized")
            # rospy.spin()

            print(self._last_time_cmd_rcv, self.is_controller_connected)
            if not self.is_controller_connected:
                rospy.loginfo("controller disconnected")
                self.set_actuators_idle()

            rate.sleep()



if __name__=="__main__":
    cfg = dk.load_config()

    controlCar = ControlCar()
    controlCar.run()


