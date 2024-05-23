import sys
from time import sleep
import serial
import mouse
import pyautogui



port = '/dev/ttyUSB0'
# port = '/dev/ttyUSB1'
# port = '/dev/ttyUSB2'
# port = 'COM19'
# port = 'COM21'
# port = 'COM4'

if len(sys.argv) > 1:
    port = sys.argv[1]

serial = serial.Serial(port, 115200, timeout=1)


mouse_pressed_left = False
mouse_pressed_right = False

try:
    # Put atttached device into dongle mode (receive data from the mouse)
    # serial.write(b'$mode=dongle\n')
    # serial.write(b'$debugging_enabled=true\n')

    offsets = None

    print('Listening...')
    while True:
        data = serial.readline()

        if data:
            try:
                data = data.decode('ascii').replace('\r\n', '').replace('\n', '')
            except:
                print(f'Error decoding data: {data}')

            print(data)

            # Auto confirm we are still in dongle mode
            if (data == '$dongle=check'):
                serial.write(b'$mode=dongle\n')
            elif (data == '$state=init_user_end'):
                # Reset happened, re-enter dongle mode
                serial.write(b'$mode=dongle\n')
            elif (str(data).startswith('$data=mouse_x=')):
                # $data=mouse_x=0.041000, mouse_y=0.003000, buttons=4
                # Mouse data
                data = data.split(', ')
                mouse_x = float(data[0].split('=')[2])
                mouse_y = float(data[1].split('=')[1])
                buttons = int(data[2].split('=')[1])

                if buttons == 191:
                    # left click
                    if not mouse_pressed_left:
                        print('left click')
                        pyautogui.mouseDown(button='left')
                        # mouse.press('left')
                        mouse_pressed_left = True
                elif buttons == 223:
                    # right click
                    if not mouse_pressed_right:
                        print('right click')
                        pyautogui.mouseDown(button='right')
                        # mouse.press('right')
                        mouse_pressed_right = True
                else:
                    if mouse_pressed_left:
                        pyautogui.mouseUp(button='left')
                        # mouse.release('left')
                        mouse_pressed_left = False
                    if mouse_pressed_right:
                        pyautogui.mouseUp(button='right')
                        # mouse.release('right')
                        mouse_pressed_right = False
                
                if (offsets == None):
                    offsets = (mouse_x, mouse_y)
                    print(f'Offsets set to: {offsets}')
                
                mouse_x -= offsets[0]
                mouse_y -= offsets[1]
                
                mouse.move(int(mouse_x * 100), int(mouse_y * 100), absolute=False)
                
except KeyboardInterrupt:
    print('\033[20B\nExiting...')
    serial.close()
except serial.SerialException as e:
    print('\033[20B\nSerial error, either device was disconnected or is in use by another program.', e)
    serial.close()
except Exception as e:
    print('\033[20B\n', e)
    serial.close()
    raise e
