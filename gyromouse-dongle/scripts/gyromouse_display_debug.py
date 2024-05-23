import math
import random
from time import sleep
import time
import serial
import mouse
import tkinter as tk

window_root = tk.Tk()
screen_width = window_root.winfo_screenwidth()
screen_height = window_root.winfo_screenheight()


# port = '/dev/ttyUSB0'
# port = 'COM19'
port = 'COM21'

class IMUMeasurment:
    def __init__(self, time=.0, accelerometer=[.0, .0, .0], gyroscope=[.0, .0, .0], magnetometer=[.0, .0, .0], temperature=.0):
        self.time = time
        self.accelerometer = accelerometer
        self.gyroscope = gyroscope
        self.magnetometer = magnetometer
        self.temperature = temperature

class GyroMouseSerial:
    def __init__(self, serial_port: str, serial_baud=115200):
        self.serial = serial.Serial(serial_port, serial_baud)
        
        self.measurment = IMUMeasurment()
        self.previous_measurment = IMUMeasurment()
        self.measurment_delta = IMUMeasurment()

        self.calibration_done = False
        self.calibration_measurments = []

        self.measurment_started = False

        self.previous_print = 0


    def read_data(self):
        data = self.serial.readline()

        if data:
            try:
                data = data.decode('ascii').replace('\r\n', '')
            except:
                print(f'Error decoding data: {data}')
                return

            if (not data.startswith('$')):
                print(f'Unknown data received: {data}')
                return

            self._process_data(data)

    def _process_data(self, data: str):
        [variable, value] = data.split('=')

        if variable == '$button':
            [index, state] = value.split(',')
            if int(index) == 1:
                if int(state) == 1:
                    mouse.press('left')
                else:
                    mouse.release('left')
            elif int(index) == 2:
                if int(state) == 1:
                    mouse.press('right')
                else:
                    mouse.release('right')
        elif variable == '$accel':
            # self.previous_measurment.accelerometer = self.measurment.accelerometer
            self.measurment.accelerometer = [float(x) for x in value.split(',')]
        elif variable == '$gyro':
            # self.previous_measurment.gyroscope = self.measurment.gyroscope
            self.measurment.gyroscope = [float(x) for x in value.split(',')]
        elif variable == '$temp':
            # self.previous_measurment.temperature = self.measurment.temperature
            self.measurment.temperature = float(value)
        elif variable == '$mag':
            # self.previous_measurment.magnetometer = self.measurment.magnetometer
            self.measurment.magnetometer = [float(x) for x in value.split(',')]
            # self.measurment_delta.magnetometer = [self.measurment.magnetometer[i] - self.previous_measurment.magnetometer[i] for i in range(len(self.measurment.magnetometer))]
            # self.measurment_delta.magnetometer = self.measurment.magnetometer - self.previous_measurment.magnetometer
        elif variable == '$update':
            if value == 'start':
                self.previous_measurment = self.measurment
                self.measurment = IMUMeasurment()
                self.measurment_started = True
            elif value == 'done':
                if not self.measurment_started:
                    return
                self.measurment_started = False
                # self.previous_measurment.time = self.measurment.time
                self.measurment.time = time.time()
                self.measurment_delta.time = self.measurment.time - self.previous_measurment.time
                
                if not self.calibration_done:
                    self._process_calibration()
                    return

                self._process_measurment()
        else:
            print(f'Unknown variable received: {variable}={value}')

    def _process_measurment(self):
        mouse_move = [.0, .0]

        # roll = math.atan2(float(accel[1]), float(accel[2])) * 180 / math.pi
        # roll = (((math.atan2(float(accel[1]), float(accel[2])) * 180 / math.pi) % 360) + 180) % 360
        # if roll > 180:
        #     roll -= 360
        # pitch = math.asin(float(accel[2])) * 180 / math.pi
        # pitch = math.asin(float(accel[0]), float(accel[2])) * 180 / math.pi
        # yaw = math.atan2(float(accel[0]), float(accel[1])) * 180 / math.pi
        # yaw = 0

        # https://engineering.stackexchange.com/questions/3348/calculating-pitch-yaw-and-roll-from-mag-acc-and-gyro-data
        # pitch = 180 * math.atan(accel[0] / math.sqrt(accel[1]**2 + accel[2]**2)) / math.pi
        # roll = 180 * math.atan(accel[1] / math.sqrt(accel[0]**2 + accel[2]**2)) / math.pi
        # yaw = 180 * math.atan(float(accel[2]) / math.sqrt(float(accel[2])**2 + float(accel[2])**2)) / math.pi
        # pitch = 180 * math.atan2(accel[0], math.sqrt(accel[1]**2 + accel[2]**2)) / math.pi
        # roll = - 180 * math.atan2(accel[1], math.sqrt(accel[0]**2 + accel[2]**2)) / math.pi - 45
        # yaw = 180 * math.atan2(float(accel[2]), math.sqrt(float(accel[0])**2 + float(accel[2])**2)) / math.pi

        # if roll_old is None:
        #     roll_old = roll
        #     pitch_old = pitch
        #     # yaw_old = yaw
        
        # if roll_offset is None:
        #     roll_offset = roll
        #     pitch_offset = pitch
        #     yaw_offset = yaw

        # pitch -= pitch_offset
        # roll -= roll_offset
        # yaw -= yaw_offset

        # if roll < 0:
        #     # Somehow the roll is less sensitive in negative direction
        #     roll *= 2

        # roll_delta = roll - roll_old
        # pitch_delta = pitch - pitch_old
        # yaw_delta = yaw - yaw_old

        # mouse_delta_x += pitch_delta * 0.5
        # deadzone_delta = 0.5
        # if abs(pitch_delta) > deadzone_delta:
        #     mouse_delta_y += pitch_delta
        # deadzone_abs = 5
        # if abs(pitch) > deadzone_abs:
        #     mouse_delta_y += pitch * 0.5
            
        # if abs(roll_delta) > deadzone_delta:
        #     mouse_delta_x += roll_delta
        # deadzone_abs = 5
        # if abs(roll) > deadzone_abs:
        #     mouse_delta_x += roll * 0.5


        # gyro_only
        mouse_move = [.0, .0]
        
        # mouse_delta_x = - gyro_val[2] * (screen_width / 2 / 180)
        # mouse_delta_y = - gyro_val[1] * (screen_height / 2 / 180)
        mouse_move[0] = - self.measurment.gyroscope[2] * (screen_width / 2 / 180)
        mouse_move[1] = - self.measurment.gyroscope[1] * (screen_height / 2 / 180)

        deadzone = 5
        # Ramp up the mouse delta values if within the deadzone
        # if abs(mouse_move[0]) < deadzone:
        #     mouse_move[0] = 0
        # if abs(mouse_move[1]) < deadzone:
        #     mouse_move[1] = 0



        # Accelerometer
        # mouse_delta_x = 0
        # mouse_delta_y = 0

        # compensate the accelerometer readings from gravity. 
        # @param q the quaternion representing the orientation of a 9DOM MARG sensor array
        # @param acc the readings coming from an accelerometer expressed in g
        #
        # @return a 3d vector representing dinamic acceleration expressed in g
        # def gravity_compensate(q, acc):
        #     g = [0.0, 0.0, 0.0]
            
        #     # get expected direction of gravity
        #     g[0] = 2 * (q[1] * q[3] - q[0] * q[2])
        #     g[1] = 2 * (q[0] * q[1] + q[2] * q[3])
        #     g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
            
        #     # compensate accelerometer readings with the expected direction of gravity
        #     return [acc[0] - g[0], acc[1] - g[1], acc[2] - g[2]]
        # compensated_accel = gravity_compensate([0, 0, 0, 1], accel)
        
        # mouse_delta_x = - compensated_accel[1]* time_delta * (screen_width / 2)
        # mouse_delta_y = - compensated_accel[0]* time_delta * (screen_width / 2)

        # print(f'compensated_accel: {compensated_accel[0]:+0.5f} {compensated_accel[1]:+0.5f} {compensated_accel[2]:+0.5f}')




        # Update the mouse
        # print(f'dx: {mouse_delta_x:+.3f}, \tdy: {mouse_delta_y:+.3f} | \troll: {roll:+.5f}, \tpitch: {pitch:+.5f}, \tyaw: {yaw:+.5f} |  \taccel: {accel[0]:+0.5f} {accel[1]:+0.5f} {accel[2]:+0.5f} | \tgyro: {gyro[0]:+0.5f} {gyro[1]:+0.5f} {gyro[2]:+0.5f} | \tmag: {mag[0]:+0.5f} {mag[1]:+0.5f} {mag[2]:+0.5f} | \tmag_delta: {mag_delta[0]:+0.5f} {mag_delta[1]:+0.5f} {mag_delta[2]:+0.5f} | \ttemp: {temp:+0.2f} | \ttime_delta: {time_delta:.3f}')

        if time.time() - self.previous_print > 0.05:
            if self.previous_print == 0:
                print('\n\n\n\n\n\n')
            self.previous_print = time.time()
            debug_line = f'\r\033[F\033[F\033[F\033[F\033[K'
            debug_line += f'dx: {mouse_move[0]:+5.3f}, dy: {mouse_move[1]:+5.3f} \n\033[K'
            debug_line += f' a: {self.measurment.accelerometer[0]:+10.5f} {self.measurment.accelerometer[1]:+10.5f} {self.measurment.accelerometer[2]:+10.5f} \n\033[K'
            debug_line += f' g: {self.measurment.gyroscope[0]:+10.5f} {self.measurment.gyroscope[1]:+10.5f} {self.measurment.gyroscope[2]:+10.5f} \n\033[K'
            debug_line += f' m: {self.measurment.magnetometer[0]:+10.5f} {self.measurment.magnetometer[1]:+10.5f} {self.measurment.magnetometer[2]:+10.5f} \n'

            print(debug_line, end='')
        # print(f' m_d: {self.measurment_delta.magnetometer[0]:+.5f} {self.measurment_delta.magnetometer[1]:+.5f} {self.measurment_delta.magnetometer[2]:+.5f} \t|', end='')
        # print(f' t: {self.measurment.temperature:+.2f} \t|', end='')    
        # print(f' t_d: {self.measurment_delta.time:.3f} \t|', end='')
            #    \ta: {self.measurment[0]:+0.5f} {accel[1]:+0.5f} {accel[2]:+0.5f} | \tgyro: {gyro[0]:+0.5f} {gyro[1]:+0.5f} {gyro[2]:+0.5f} | \tmag: {mag[0]:+0.5f} {mag[1]:+0.5f} {mag[2]:+0.5f} | \tmag_delta: {mag_delta[0]:+0.5f} {mag_delta[1]:+0.5f} {mag_delta[2]:+0.5f} | \ttemp: {temp:+0.2f} | \ttime_delta: {time_delta:.3f}')
        # mouse.move(int(mouse_delta_x), int(mouse_delta_y), absolute=False, duration=0.005)
        # mouse.move(int(mouse_delta_x), int(mouse_delta_y), absolute=False, duration=time_delta/2)
        # mouse.move(int(mouse_delta_x), int(mouse_delta_y), absolute=False, duration=0.001)
        # mouse.move(int(mouse_delta_x), int(mouse_delta_y), absolute=True, duration=0.005)

    def _process_calibration(self):
        if self.calibration_done:
            return
        
        self.calibration_measurments.append(self.measurment)

        if len(self.calibration_measurments) < 50:
            return

        self._calibrate()


    def _calibrate(self):
        print('Calibrating...')

        # Calculate the average values
        avg_accel = [sum([m.accelerometer[i] for m in self.calibration_measurments]) / len(self.calibration_measurments) for i in range(3)]
        avg_gyro = [sum([m.gyroscope[i] for m in self.calibration_measurments]) / len(self.calibration_measurments) for i in range(3)]
        avg_mag = [sum([m.magnetometer[i] for m in self.calibration_measurments]) / len(self.calibration_measurments) for i in range(3)]
        avg_temp = sum([m.temperature for m in self.calibration_measurments]) / len(self.calibration_measurments)

        # Calculate the standard deviation
        std_accel = [math.sqrt(sum([(m.accelerometer[i] - avg_accel[i])**2 for m in self.calibration_measurments]) / len(self.calibration_measurments)) for i in range(3)]
        std_gyro = [math.sqrt(sum([(m.gyroscope[i] - avg_gyro[i])**2 for m in self.calibration_measurments]) / len(self.calibration_measurments)) for i in range(3)]
        std_mag = [math.sqrt(sum([(m.magnetometer[i] - avg_mag[i])**2 for m in self.calibration_measurments]) / len(self.calibration_measurments)) for i in range(3)]
        std_temp = math.sqrt(sum([(m.temperature - avg_temp)**2 for m in self.calibration_measurments]) / len(self.calibration_measurments))

        print(f'first_accel: {self.calibration_measurments[0].accelerometer[0]:+.5f} {self.calibration_measurments[0].accelerometer[1]:+.5f} {self.calibration_measurments[0].accelerometer[2]:+.5f}')
        print(f'first_gyro: {self.calibration_measurments[0].gyroscope[0]:+.5f} {self.calibration_measurments[0].gyroscope[1]:+.5f} {self.calibration_measurments[0].gyroscope[2]:+.5f}')
        print(f'first_mag: {self.calibration_measurments[0].magnetometer[0]:+.5f} {self.calibration_measurments[0].magnetometer[1]:+.5f} {self.calibration_measurments[0].magnetometer[2]:+.5f}')
        print(f'first_temp: {self.calibration_measurments[0].temperature:+.2f}')
        print()

        print(f'last_accel: {self.measurment.accelerometer[0]:+.5f} {self.measurment.accelerometer[1]:+.5f} {self.measurment.accelerometer[2]:+.5f}')
        print(f'last_gyro: {self.measurment.gyroscope[0]:+.5f} {self.measurment.gyroscope[1]:+.5f} {self.measurment.gyroscope[2]:+.5f}')
        print(f'last_mag: {self.measurment.magnetometer[0]:+.5f} {self.measurment.magnetometer[1]:+.5f} {self.measurment.magnetometer[2]:+.5f}')
        print(f'last_temp: {self.measurment.temperature:+.2f}')
        print()

        print(f'avg_accel: {avg_accel[0]:+.5f} {avg_accel[1]:+.5f} {avg_accel[2]:+.5f}')
        print(f'avg_gyro: {avg_gyro[0]:+.5f} {avg_gyro[1]:+.5f} {avg_gyro[2]:+.5f}')
        print(f'avg_mag: {avg_mag[0]:+.5f} {avg_mag[1]:+.5f} {avg_mag[2]:+.5f}')
        print(f'avg_temp: {avg_temp:+.2f}')
        print()

        print(f'std_accel: {std_accel[0]:+.5f} {std_accel[1]:+.5f} {std_accel[2]:+.5f}')
        print(f'std_gyro: {std_gyro[0]:+.5f} {std_gyro[1]:+.5f} {std_gyro[2]:+.5f}')
        print(f'std_mag: {std_mag[0]:+.5f} {std_mag[1]:+.5f} {std_mag[2]:+.5f}')
        print(f'std_temp: {std_temp:+.2f}')
        print(f'N_measurments: {len(self.calibration_measurments)}')
        print()

        print('Calibration done')
        # exit(-1)
        self.calibration_done = True


gyro_mouse = GyroMouseSerial(port)
try:
    while True:
        gyro_mouse.read_data()
except Exception as e:
    print(e)
    gyro_mouse.serial.close()
