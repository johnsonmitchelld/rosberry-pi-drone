#!/usr/bin/env python
from evdev import InputDevice, categorize, ecodes
import rospy
import platform
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

num_axes = 5
center_throttle = 32767.5



def controller():
    pub = rospy.Publisher('controller_values', Joy, queue_size=10)
    rospy.init_node('controller_publisher', anonymous=True)
    dev = InputDevice('/dev/input/event0')
    message = Joy()
    message.axes = num_axes*[0]
    message.buttons = [0]
    for event in dev.read_loop():
        if not rospy.is_shutdown():
            process_event(event, message)
            pub.publish(message)
        else:
            break

def normalize_axis(val, dead_zone):
    if val > dead_zone:
        return (val - dead_zone) / (1 - dead_zone)
    if val < -dead_zone:
        return (val + dead_zone) / (1 - dead_zone)
    else:
        return 0.0

def process_event(event, message):
    changed = False
    if event.type > 0:
        if event.code == rTrigger:
            throttle = event.value / 1023.0
            message.axes[0] = throttle
        if event.code == stickLX:
            value = (event.value - center_throttle) / center_throttle
            message.axes[1] = normalize_axis(value, 0.15)
        if event.code == stickLY:
            value = -(event.value - center_throttle) / center_throttle
            message.axes[2] = normalize_axis(value, 0.15)
        if event.code == stickRX:
            value = (event.value - center_throttle) / center_throttle
            message.axes[3] = normalize_axis(value, 0.15)
        if event.code == stickRY:
            value = -(event.value - center_throttle) / center_throttle
            message.axes[4] = normalize_axis(value, 0.15)

        if event.code == btnA:
            message.buttons[0] = event.value

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass

