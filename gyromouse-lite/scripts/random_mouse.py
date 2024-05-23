import random
from time import sleep
import mouse

while True:
    sleep(0.05)

    delta_x = random.randint(-50, 50)
    delta_y = random.randint(-50, 50)
    distance = (delta_x**2 + delta_y**2)**0.5
    button_left = random.randint(0, 1)
    mouse_left = 0
    button_right = random.randint(0, 1)
    mouse_right = 0
    
    print(f'dx: {delta_x}, dy: {delta_y}, left: {button_left}, right: {button_right}')

    # Move the mouse cursor by delta_x and delta_y
    mouse.move(int(delta_x), int(delta_y), absolute=False, duration=0.05)

    if button_left == 1:
        if (mouse_left == 0):
            mouse.press('left')
            mouse_left = 1
    else:
        if (mouse_left == 1):
            mouse.release('left')
            mouse_left = 0
    if button_right == '1':
        if (mouse_right == 0):
            mouse.press('right')
            mouse_right = 1
    else:
        if (mouse_right == 1):
            mouse.release('right')
            mouse_right = 0
