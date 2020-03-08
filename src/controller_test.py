from evdev import InputDevice, categorize, ecodes
dev = InputDevice('/dev/input/event0')

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


for event in dev.read_loop():
    if event.type > 0:
        if event.code == 0:
            print(event.value)