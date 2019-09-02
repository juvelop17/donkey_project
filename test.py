import keyboard  # using module keyboard
import time
import sys
from pynput import keyboard

_switch = True


def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(key.char))
    except AttributeError:
        print('special key {0} pressed'.format(key))

def on_release(key):
    print('{0} released'.format(key))
    if key == keyboard.Key.esc:
        _switch = False
        sys.exit()
        # Stop listener
        # return False

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

while listener.isAlive():
    # print(_switch)
    print(listener.isAlive())
    time.sleep(0.1)


