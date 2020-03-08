#!/usr/bin/env python
from evdev import InputDevice, categorize, ecodes
import rospy
import platform
print(platform.python_version())
from sensor_msgs.msg import Joy

# absolute value events
stickLX = 0
stickLY = 1
stickRX = 2
stickRY = 5
rTrigger = 9
lTrigger = 10

# buttons
dPadX = 16
dPadY = 17
btnA = 304
btnB = 305
btnX = 306
btnY = 307
btnLB = 308
btnRB = 309
btnLMenu = 310
btnRMenu = 311
btnLStick = 312
btnRStick = 313


def controller():
    pub = rospy.Publisher('controller_values', Joy, queue_size=10)
    rospy.init_node('controller_publisher', anonymous=True)
    dev = InputDevice('/dev/input/event0')
    message = Joy()
    message.axes = [0]
    message.buttons = [0]
    for event in dev.read_loop():
        if not rospy.is_shutdown():
            if process_event(event, message):
                pub.publish(message)
        else:
            break

def process_event(event, message):
    changed = False
    if event.code == 9:
        throttle = event.value / 1023.0
        message.axes[0] = throttle
        changed = True
    if event.code == 304:
        message.buttons[0] = event.value
        changed = True
    return changed

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass

