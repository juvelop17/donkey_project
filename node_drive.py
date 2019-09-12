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
from imu_topic_pub import ImuTopicPublisher
import time
import threading
from motor import PCA9685, PWMSteering, PWMThrottle



class ControlCar():
    controller = None
    isKeyboard = False
    isVoice = False

    voice_speed = 0.2
    voice_turn = 0.5

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
        self._last_time_voice_rcv = time.time()
        self._timeout_s = 5
	
        ros_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self.set_actuators_from_cmdvel)
        #ros_controller = rospy.Subscriber("/cmd_controller", String, self.setController)
        rospy.loginfo("> Subscriber corrrectly initialized")

    @property
    def is_controller_connected(self):
        #print(time.time() - self._last_time_cmd_rcv)
        return (time.time() - self._last_time_cmd_rcv < self._timeout_s)

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
        if message == 'keyboard':
            self.isKeyboard = True
        else:
            self.isKeyboard = False

        if message == 'voice':
            print('get voice')
            self.isVoice = True
            ros_voice_text = rospy.Subscriber("/voice_text", String, self.setVoiceControl)
        else:
            self.isVoice = False

    def setVoiceControl(self, message):
        if self.isVoice == True:
            _message = message.strip().split()
            print('message',_message)
            for m in _message:
            if msg in ['출발','앞으로','가']:
                self.throttle.run(self.voice_speed)  # - : backward, + : forward
                self.steering.run(0)  # - : left, + : right
            if msg in ['멈춰','정지']:
                self.throttle.run(0)  # - : backward, + : forward
                self.steering.run(0)  # - : left, + : right
            if msg in ['오른쪽']:
                self.throttle.run(self.voice_speed)  # - : backward, + : forward
                self.steering.run(self.voice_turn)  # - : left, + : right
            if msg in ['왼쪽']:
                self.throttle.run(self.voice_speed)  # - : backward, + : forward
                self.steering.run(-self.voice_turn)  # - : left, + : right

    def run(self):
        # --- Set the control rate
        rate = rospy.Rate(10)

        # run imu node
        imuTopicPublisher = ImuTopicPublisher()
        print('imuTopicPublisher',imuTopicPublisher)
        imuTopicPublisher.run()
        print('imuTopicPublisher',imuTopicPublisher)

        while not rospy.is_shutdown():
            #print(self._last_time_cmd_rcv, self.is_controller_connected)
            if not self.is_controller_connected:
                rospy.loginfo("controller disconnected")
                self.set_actuators_idle()

            #rospy.spin()
            rospy.sleep(0.1)



if __name__=="__main__":
    cfg = dk.load_config()

    controlCar = ControlCar()
    controlCar.run()


