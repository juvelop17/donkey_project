import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import sys, select, termios, tty
import donkeycar as dk

# import keyboard  # using module keyboard
from pynput import keyboard



#############################################################################################
# keyboard


moveBindings = {
    'w': (1, 0, 0, 0),
    'a': (0, 0, 0, -1),
    's': (-1, 0, 0, 0),
    'd': (0, 0, 0, 1),
    'wa': (1, 0, 0, -1),
    'wd': (1, 0, 0, 1),
    'sa': (-1, 0, 0, -1),
    'sd': (-1, 0, 0, 1)
}

speedBindings={
    'z': (1.1, 1),
    'x': (.9, 1),
    'c': (1, 1.1),
    'v': (1, .9)
}


msg = '''

Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
  w
a s d 

anything else : stop
z/x : increase/decrease only linear speed by 10%
c/v : increase/decrease only angular speed by 10%
CTRL-C to quit

'''


class KeyboardRun:

    key_w = False
    key_a = False
    key_s = False
    key_d = False
    key_z = False
    key_x = False
    key_c = False
    key_v = False
    key_ctrl = False

    cmd_controller_pub_str = 'keyboard'

    def __init__(self):
        # settings = termios.tcgetattr(sys.stdin)

        rospy.init_node('keyboard_node')
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cmd_controller_pub = rospy.Publisher('cmd_controller', String, queue_size=1)

        self.speed = rospy.get_param("~speed", 0.18)
        self.turn = rospy.get_param("~turn", 0.7)
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        self.status = 0
        self.switch = False

    def run(self):
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()
        self.switch = True

        try:
            print(msg)
            print(self.vels(self.speed, self.turn))

            while (self.listener.isAlive()):
                if self.key_w == True and self.key_a == False and self.key_d == False:
                    self.x, self.y, self.z, self.th = moveBindings['w']
                elif self.key_s == True and self.key_a == False and self.key_d == False:
                    self.x, self.y, self.z, self.th = moveBindings['s']

                elif self.key_w == True and self.key_a == True:
                    self.x, self.y, self.z, self.th = moveBindings['wa']
                elif self.key_w == True and self.key_d == True:
                    self.x, self.y, self.z, self.th = moveBindings['wd']
                elif self.key_s == True and self.key_a == True:
                    self.x, self.y, self.z, self.th = moveBindings['sa']
                elif self.key_s == True and self.key_d == True:
                    self.x, self.y, self.z, self.th = moveBindings['sd']

                elif self.key_d == True:
                    self.x, self.y, self.z, self.th = moveBindings['d']
                elif self.key_a == True:
                    self.x, self.y, self.z, self.th = moveBindings['a']

                elif self.key_z == True:
                    self.speed *= speedBindings['z'][0]
                    self.turn *= speedBindings['z'][0]
                    print(self.vels(self.speed, self.turn))
                    if (self.status == 14):
                        print(msg)
                    self.status = (self.status + 1) % 15
                elif self.key_x == True:
                    self.speed *= speedBindings['x'][0]
                    self.turn *= speedBindings['x'][0]
                    print(self.vels(self.speed, self.turn))
                    if (self.status == 14):
                        print(msg)
                    self.status = (self.status + 1) % 15
                elif self.key_c == True:
                    self.speed *= speedBindings['c'][0]
                    self.turn *= speedBindings['c'][0]
                    print(self.vels(self.speed, self.turn))
                    if (self.status == 14):
                        print(msg)
                    self.status = (self.status + 1) % 15
                elif self.key_v == True:
                    self.speed *= speedBindings['v'][0]
                    self.turn *= speedBindings['v'][0]
                    print(self.vels(self.speed, self.turn))
                    if (self.status == 14):
                        print(msg)
                    self.status = (self.status + 1) % 15
                elif self.key_ctrl == True:
                    print('exit')
                    self.switch = False
                else:
                    self.x = 0
                    self.y = 0
                    self.z = 0
                    self.th = 0

                twist = Twist()
                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = self.th * self.turn
                self.cmd_controller_pub.publish(self.cmd_controller_pub_str)
                self.cmd_vel_pub.publish(twist)

                print('twist value : ',twist.linear.x,twist.linear.y,twist.linear.z,twist.angular.x,twist.angular.y,twist.angular.z)

                time.sleep(0.1)

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.cmd_controller_pub.publish(self.cmd_controller_pub_str)
            self.cmd_vel_pub.publish(twist)

            # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def on_press(self, key):
        try:
            # print('alphanumeric key {0} pressed'.format(key.char))
            _key = key.char
            if _key == 'w':
                self.key_w = True
            elif _key == 'a':
                self.key_a = True
            elif _key == 's':
                self.key_s = True
            elif _key == 'd':
                self.key_d = True
            elif _key == 'z':
                self.key_z = True
            elif _key == 'x':
                self.key_x = True
            elif _key == 'c':
                self.key_c = True
            elif _key == 'v':
                self.key_v = True
            elif _key == keyboard.Key.esc:
                print('exit')
                return False
        except AttributeError:
            print('special key {0} pressed'.format(key.char))

    def on_release(self, key):
        # print('{0} released'.format(key))
        _key = key.char
        if _key == 'w':
            self.key_w = False
        elif _key == 'a':
            self.key_a = False
        elif _key == 's':
            self.key_s = False
        elif _key == 'd':
            self.key_d = False
        elif _key == 'z':
            self.key_z = False
        elif _key == 'x':
            self.key_x = False
        elif _key == 'c':
            self.key_c = False
        elif _key == 'v':
            self.key_v = False
        elif _key == keyboard.Key.esc:
            # Stop listener
            print('exit')
            return False

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed, turn)




if __name__=="__main__":
    cfg = dk.load_config()
    keyboardRun = KeyboardRun() #
    keyboardRun.run()





