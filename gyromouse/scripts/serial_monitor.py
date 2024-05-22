import sys
from time import sleep
import serial

port = '/dev/ttyUSB0'
# port = 'COM19'
# port = 'COM21'
# port = 'COM4'

if len(sys.argv) > 1:
    port = sys.argv[1]

serial = serial.Serial(port, 115200, timeout=1)

try:
    # serial.write(b'$debugging_enabled=true\n')

    
    while True:
        data = serial.readline()

        if data:
            try:
                data = data.decode('ascii').replace('\r\n', '').replace('\n', '')
            except:
                print(f'Error decoding data: {data}')
            
            print(data)
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
