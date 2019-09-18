#!/usr/bin/python

# drive


# !/usr/bin/python

"""
Class for low level control of owr car. It assumes ros-12cpwmboard has been
installed
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import donkeycar as dk
from node_imu_topic_pub import ImuTopicPublisher
import time
import threading
from motor import PCA9685, PWMSteering, PWMThrottle


class ControlCar():
    controller = None
    isKeyboard = False
    isVoice = False

    voice_speed = 0.2
    voice_turn = 0.8

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
        self._last_time_voice_rcv = time.time()
        self._voice_timeout_s = 3

        ros_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self.set_actuators_from_cmdvel)
        ros_controller = rospy.Subscriber("/cmd_controller", String, self.setController)
        ros_voice_text = rospy.Subscriber("/voice_text", String, self.setVoiceControl)
        rospy.loginfo("> Subscriber corrrectly initialized")

    @property
    def is_controller_connected(self):
        # print(time.time() - self._last_time_cmd_rcv)
        return (time.time() - self._last_time_cmd_rcv < self._timeout_s)

    @property
    def is_voice_connected(self):
        # print(time.time() - self._last_time_voice_rcv)
        return (time.time() - self._last_time_voice_rcv < self._voice_timeout_s)

    def set_actuators_from_cmdvel(self, message):
        # -- Save the time
        self._last_time_cmd_rcv = time.time()

        # -- Convert vel into servo values
        self.throttle.run(message.linear.x)  # - : backward, + : forward
        self.steering.run(message.angular.z)  # - : left, + : right
        rospy.loginfo("Got a command v = %2.1f  s = %2.1f" % (message.linear.x, message.angular.z))

    def set_actuators_idle(self):
        # -- Convert vel into servo values
        self.throttle.run(0)  # - : backward, + : forward
        self.steering.run(0)  # - : left, + : right
        rospy.loginfo("Setting actutors to idle")

    def setController(self, message):
        print('setController message', message)
        print('setController message.data', message.data)
        if message.data == 'keyboard':
            self.isKeyboard = True
        else:
            self.isKeyboard = False

        if message.data == 'voice':
            print('get voice')
            self.isVoice = True
        else:
            self.isVoice = False

    def setVoiceControl(self, message):
        _message = message.data.strip().split()
        print('setVoiceControl message.data', _message)
        for msg in _message:
            _msg = msg.strip()
            print('_msg', _msg)
            if _msg in ['출발', '앞으로']:
                print('출발')
                self._last_time_voice_rcv = time.time()
                self.throttle.run(self.voice_speed)  # - : backward, + : forward
                self.steering.run(0)  # - : left, + : right
            if _msg in ['멈춰', '정지']:
                print('정지')
                self._last_time_voice_rcv = time.time()
                self.throttle.run(0)  # - : backward, + : forward
                self.steering.run(0)  # - : left, + : right
            if _msg in ['오른쪽','오른쪽으로']:
                print('오른쪽')
                self._last_time_voice_rcv = time.time()
                self.throttle.run(self.voice_speed)  # - : backward, + : forward
                self.steering.run(self.voice_turn)  # - : left, + : right
            if _msg in ['왼쪽','왼쪽으로']:
                print('왼쪽')
                self._last_time_voice_rcv = time.time()
                self.throttle.run(self.voice_speed)  # - : backward, + : forward
                self.steering.run(-self.voice_turn)  # - : left, + : right

    def run(self):
        # --- Set the control rate
        rate = rospy.Rate(10)

        # run imu node
        # imuTopicPublisher = ImuTopicPublisher()
        # imuTopicPublisher.run()
        # print('imuTopicPublisher',imuTopicPublisher)

        while not rospy.is_shutdown():
            # print('is_controller_connected', self.is_controller_connected)
            # print('is_voice_connected', self.is_voice_connected)
            if not self.is_voice_connected and not self.is_controller_connected:
                rospy.loginfo("disconnected")
                self.set_actuators_idle()

            # rospy.spin()
            rospy.sleep(0.1)


if __name__ == "__main__":
    cfg = dk.load_config()

    controlCar = ControlCar()
    controlCar.run()
