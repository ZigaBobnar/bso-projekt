import math
import random
from time import sleep
import time
import serial
import mouse
# import Tkinter as tk

import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *

# window_root = tk.Tk()
screen_width = 1000#window_root.winfo_screenwidth()
screen_height = 1000#window_root.winfo_screenheight()

useQuat = False

# port = '/dev/ttyUSB0'
# port = 'COM19'
# port = 'COM21'
port = 'COM4'

class IMUMeasurment:
    def __init__(self, time=.0, accelerometer=[.0, .0, .0], gyroscope=[.0, .0, .0], magnetometer=[.0, .0, .0], temperature=.0, velocity=[.0, .0, .0], position=[.0, .0, .0], rotation=[.0, .0, .0]):
        self.time = time
        self.mcu_delta = .0
        self.accelerometer = accelerometer
        self.gyroscope = gyroscope
        self.magnetometer = magnetometer
        self.temperature = temperature
        self.velocity = velocity
        self.position = position
        self.rotation = rotation

class GyroMouseSerial:
    def __init__(self, serial_port: str, serial_baud=115200):
        self.serial = serial.Serial(serial_port, serial_baud)
        
        self.measurment = IMUMeasurment()
        self.previous_measurment = IMUMeasurment()
        self.measurment_delta = IMUMeasurment()

        self.gyrometer_offset = [0, 0, 0]
        self.gravity = 0

        self.calibration_done = False
        self.calibration_measurments = []

        self.measurment_started = False

        self.previous_print = 0
        self.current_measurment_commands = []
        self.print_queue = []



    # Delayed print to be able to update the measurment
    def print(self, *args, **kwargs):
        # self.print_queue.append([args, kwargs])
        pass

    def _flush_out_queue(self):
        for args, kwargs in self.print_queue:
            print(*args, **kwargs)
        self.print_queue = []


    def read_and_process_data_line(self):
        data = self.serial.readline()

        if data:
            try:
                data = data.decode('ascii').replace('\r\n', '').replace('\n', '')
            except:
                print(f'Error decoding data: {data}')
                return
            
            if (not data.startswith('$')):
                print(f'Unknown data received: {data}')
                return

            self._process_command_line(data)

    def _process_command_line(self, data: str):
        for command in data.split(';'):
            if not command:
                continue

            if command.count('=') != 1:
                print(f'Invalid command received: {command}')
                continue

            [command, value] = command.split('=')
            # value = None
            # if len(command) > 1:
            #     command = command[0]
            #     value = command[1]
            self._process_command(command, value)

    def _process_command(self, command: str, value: str):
        self.current_measurment_commands.append([command, value])

        if command == '$button':
            [index, state] = value.split(',')
            self.print(f'Button {index} state: {state}')
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
        elif command == '$dt':
            self.measurment.mcu_delta = float(value) / 1000
        elif command == '$accel':
            # self.previous_measurment.accelerometer = self.measurment.accelerometer
            self.measurment.accelerometer = [float(x) for x in value.split(',')]
        elif command == '$gyro':
            # self.previous_measurment.gyroscope = self.measurment.gyroscope
            self.measurment.gyroscope = [float(x) for x in value.split(',')]
            self.measurment.gyroscope = [self.measurment.gyroscope[i] + self.gyrometer_offset[i] for i in range(3)]
            # self.measurment.gyroscope[0] = -self.measurment.gyroscope[0]
        elif command == '$temp':
            # self.previous_measurment.temperature = self.measurment.temperature
            self.measurment.temperature = float(value)
        elif command == '$mag':
            # self.previous_measurment.magnetometer = self.measurment.magnetometer
            self.measurment.magnetometer = [float(x)/1000 for x in value.split(',')]
            # self.measurment_delta.magnetometer = [self.measurment.magnetometer[i] - self.previous_measurment.magnetometer[i] for i in range(len(self.measurment.magnetometer))]
            # self.measurment_delta.magnetometer = self.measurment.magnetometer - self.previous_measurment.magnetometer
        elif command == '$update':
            if value == 'start':
                self.previous_measurment = self.measurment
                self.measurment = IMUMeasurment()
                self.measurment_started = True
                self.current_measurment_commands = []
            elif value == 'done':
                if not self.measurment_started:
                    return
                self.measurment_started = False
                # self.previous_measurment.time = self.measurment.time
                self.measurment.time = time.time()

                if self.previous_measurment.time != 0:
                    self.measurment_delta.time = self.measurment.time - self.previous_measurment.time
                    self.measurment_delta.accelerometer = [self.measurment.accelerometer[i] - self.previous_measurment.accelerometer[i] for i in range(len(self.measurment.accelerometer))]
                    self.measurment_delta.gyroscope = [self.measurment.gyroscope[i] - self.previous_measurment.gyroscope[i] for i in range(len(self.measurment.gyroscope))]
                    self.measurment_delta.magnetometer = [self.measurment.magnetometer[i] - self.previous_measurment.magnetometer[i] for i in range(len(self.measurment.magnetometer))]
                    self.measurment_delta.temperature = self.measurment.temperature - self.previous_measurment.temperature
                    # self.measurment_delta.velocity = [self.measurment.velocity[i] - self.previous_measurment.velocity[i] for i in range(len(self.measurment.velocity))]
                    # self.measurment_delta.position = [self.measurment.position[i] - self.previous_measurment.position[i] for i in range(len(self.measurment.position))]
                    # self.measurment_delta.rotation = [self.measurment.rotation[i] - self.previous_measurment.rotation[i] for i in range(len(self.measurment.rotation))]
                
                if not self.calibration_done:
                    self._process_calibration()
                    self._flush_out_queue()
                    return

                self._process_measurment()
        elif command == '$info':
            self.print(f'\033[1;34mInfo: {value}\033[0m')
        elif command == '$error':
            self.print(f'\033[1;31mError: {value}\033[0m')
        elif command == '$debug':
            if value == 'IO::_update_module_leds':
                return
            elif value == 'IO::_update_pcf':
                return
            self.print(f'Debug: {value}')
        else:
            self.print(f'Unknown command received: {command}={value}')

    def _process_measurment(self):
        mouse_move = [.0, .0]

        if self.measurment_delta.time == 0:
            # Keep old values and return
            self.measurment.velocity = self.previous_measurment.velocity
            self.measurment.position = self.previous_measurment.position
            self.measurment.rotation = self.previous_measurment.rotation
            return

        # self.measurment.velocity = [self.previous_measurment.velocity[i] + self.measurment.accelerometer[i] * self.measurment_delta.time for i in range(3)]
        self.measurment.velocity = [self.previous_measurment.velocity[i] + self.measurment.accelerometer[i] * self.measurment_delta.time for i in range(3)]
        self.measurment.position = [self.previous_measurment.position[i] + self.measurment.velocity[i] * self.measurment_delta.time for i in range(3)]

        # alpha = 0.02
        alpha = 0.005
        self.measurment.rotation = [
            ((1-alpha) * (self.previous_measurment.rotation[0] + self.measurment.gyroscope[0] * self.measurment.mcu_delta) + alpha * (self.measurment.rotation[0])),
            ((1-alpha) * (self.previous_measurment.rotation[1] + self.measurment.gyroscope[1] * self.measurment.mcu_delta) + alpha * (self.measurment.rotation[1])),
            (self.previous_measurment.rotation[2] + self.measurment.gyroscope[2] * self.measurment_delta.time)
        ]

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

        def move_up(lines):
            return f'\033[{lines}A'
        def move_down(lines):
            return f'\033[{lines}B'
        def move_right(chars):
            return f'\033[{chars}C'
        def move_left(chars):
            return f'\033[{chars}D'
        def next_line(lines=1):
            return f'\033[{lines}E'
        def previous_line(lines=1):
            return f'\033[{lines}F'
        def clear_line():
            return f'\033[K'
            
        magnetometer_heading = math.atan2(self.measurment.magnetometer[1], self.measurment.magnetometer[0]) * 180 / math.pi

        if time.time() - self.previous_print > 0.05:
            debug_lines = 15
            debug_offset = 20
            total_debug_lines = debug_lines + debug_offset

            if self.previous_print == 0:
                print('\n' * (total_debug_lines+2), end='')
            self.previous_print = time.time()

            if len(self.print_queue) > 0:
                if len(self.print_queue) > debug_offset:
                    debug_line = f'{move_down(debug_lines+debug_offset+10)}{move_up(debug_lines)}'
                    for i in range(debug_lines):
                        debug_line += f'\r{clear_line()}\n'
                    print(debug_line, end='')

                print(f'{move_down(debug_lines+debug_offset+10)}{move_up(total_debug_lines)}', end='')
                for args, kwargs in self.print_queue:
                    print(f'{clear_line()}', end='')
                    print(*args, **kwargs)
                print(f'{move_down(total_debug_lines)}' + '\n' * (len(self.print_queue)), end='')
                self.print_queue = []


            debug_line = f'{move_down(debug_lines+debug_offset+10)}'
            debug_line += move_up(debug_lines)
            debug_line += f'\r{clear_line()}         mouse: {mouse_move[0]:+20.3f} {mouse_move[1]:+20.3f}\n'
            debug_line += f'\r{clear_line()} accelerometer: {self.measurment.accelerometer[0]:+20.5f} {self.measurment.accelerometer[1]:+20.5f} {self.measurment.accelerometer[2]:+20.5f} \n'
            debug_line += f'\r{clear_line()}     gyroscope: {self.measurment.gyroscope[0]:+20.5f} {self.measurment.gyroscope[1]:+20.5f} {self.measurment.gyroscope[2]:+20.5f} \n'
            debug_line += f'\r{clear_line()}  magnetometer: {self.measurment.magnetometer[0]:+20.5f} {self.measurment.magnetometer[1]:+20.5f} {self.measurment.magnetometer[2]:+20.5f}    heading: {magnetometer_heading:+5.5}\n'
            debug_line += f'\r{clear_line()}   temperature: {self.measurment.temperature:+20.5f}\xB0C \n'
            debug_line += f'\r{clear_line()}          time: {self.measurment.time:20.5f}    delta: {self.measurment_delta.time:10.5f}    mcu_delta: {self.measurment.mcu_delta:10.5f}\n'
            debug_line += f'\r{clear_line()}      velocity: {self.measurment.velocity[0]:+20.5f} {self.measurment.velocity[1]:+20.5f} {self.measurment.velocity[2]:+20.5f} \n'
            debug_line += f'\r{clear_line()}      position: {self.measurment.position[0]:+20.5f} {self.measurment.position[1]:+20.5f} {self.measurment.position[2]:+20.5f} \n'
            debug_line += f'\r{clear_line()}      rotation:  {self.measurment.rotation[0]:+19.5f}\xB0 {self.measurment.rotation[1]:+19.5f}\xB0 {self.measurment.rotation[2]:+19.5f}\xB0 \n'
            debug_line += f'\r{clear_line()} \n{move_down(debug_lines)}{move_up(total_debug_lines)}'
            
            # debug_line += next_line(debug_lines - 2)
            # debug_line += next_line(debug_bottom_offset)

            print(debug_line, end='')
            
        # mouse.move(int(mouse_move[0]), int(mouse_move[1]), absolute=False, duration=self.measurment_delta.time/2)
        # mouse.move(int(mouse_move[0]), int(mouse_move[1]), absolute=False)

        
        roll = gyro_mouse.measurment.rotation[0]
        pitch = gyro_mouse.measurment.rotation[1]
        yaw = gyro_mouse.measurment.rotation[2]
        x = gyro_mouse.measurment.position[0] / 100
        y = gyro_mouse.measurment.position[1] / 100
        z = gyro_mouse.measurment.position[2] / 100

        draw(1, yaw, pitch, roll, x, y, z)
        
        pygame.display.flip()

    def _process_calibration(self):
        if self.calibration_done:
            return
        
        if len(self.calibration_measurments) == 0:
            print('Starting calibration...')
        
        self.calibration_measurments.append(self.measurment)

        if len(self.calibration_measurments) < 50:
            return

        self._calibrate()


    def _calibrate(self):
        print('Calibrating...')

        self.serial.write(b'$get_info=firmware\n')

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

        self.gravity = math.sqrt(avg_accel[0]**2 + avg_accel[1]**2 + avg_accel[2]**2)

        # Constant gyro drift (offset)
        self.gyrometer_offset = [-x for x in avg_gyro]

        def rotation_matrix(A,B):
            import numpy as np
            # a and b are in the form of numpy array

            ax = A[0]
            ay = A[1]
            az = A[2]

            bx = B[0]
            by = B[1]
            bz = B[2]

            au = A/(np.sqrt(ax*ax + ay*ay + az*az))
            bu = B/(np.sqrt(bx*bx + by*by + bz*bz))

            R=np.array([[bu[0]*au[0], bu[0]*au[1], bu[0]*au[2]], [bu[1]*au[0], bu[1]*au[1], bu[1]*au[2]], [bu[2]*au[0], bu[2]*au[1], bu[2]*au[2]] ])

            return(R)
        
        position = [.0, .0, .0]
        rotation = [.0, .0, .0]

        # Derive rotations around axis for the accelerometer
        pitch = math.atan2(avg_accel[0], math.sqrt(avg_accel[1]**2 + avg_accel[2]**2)) * 180 / math.pi
        roll = math.atan2(avg_accel[1], math.sqrt(avg_accel[0]**2 + avg_accel[2]**2)) * 180 / math.pi
        yaw = 0 # Cannot be obtained from accelerometer

        rotation[0] = pitch
        rotation[1] = roll
        rotation[2] = yaw


        def spherical_to_xyz(r, theta, fi):
            x = r * math.sin(theta) * math.cos(fi)
            y = r * math.sin(theta) * math.sin(fi)
            z = r * math.cos(theta)
            return [x, y, z]
        
        def xyz_to_spherical(x, y, z):
            r = math.sqrt(x**2 + y**2 + z**2)
            theta = math.acos(z/r)
            fi = math.acos(x/math.sqrt(x**2+y**2))*math.copysign(1, y)
            return [r, theta, fi]
        
        def rotate_vector_rpy(vector, rpy):
            roll = math.radians(rpy[0])
            pitch = math.radians(rpy[1])
            yaw = math.radians(rpy[2])
            R_roll = [[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]]
            R_pitch = [[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]]
            R_yaw = [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]

            result = vector
            result = [sum([R_roll[i][j] * result[j] for j in range(3)]) for i in range(3)]
            result = [sum([R_pitch[i][j] * result[j] for j in range(3)]) for i in range(3)]
            result = [sum([R_yaw[i][j] * result[j] for j in range(3)]) for i in range(3)]

            return result

        
        def rotate_vector_rot_matrix(vector, R):
            result = vector
            result = [sum([R[i][j] * result[j] for j in range(3)]) for i in range(3)]

            return result

        
        def compensate_gravity(rotation, acceleration):
            # result = rotate_vector_rpy([0, 0, 1], rotation)
            rot = rotation_matrix([0, 0, 1], [0, 0, 1])

            print(rot)

            # result = rotate_vector_rpy(acceleration, rot)
            result = rotate_vector_rot_matrix(acceleration, rot)

            return result
            # r = 1
            # theta = math.radians(rotation[1])
            # fi = math.radians(rotation[2])
            # # [r, theta, fi] = xyz_to_spherical(rotation[0], rotation[1], rotation[2])


            # gravity = spherical_to_xyz(r, theta, fi)
            # # gravity = [gravity[i] for i in range(3)]

            # # return [acceleration[i] - gravity[i] for i in range(3)]
            # return gravity

        # rotation = xyz_to_euler(avg_accel[0], avg_accel[1], avg_accel[2])

        print(f'RAW acceleration:         {avg_accel}')
        # print(f'Compensated acceleration: {compensate_gravity([0, 90, 0], avg_accel)}')
        print(f'Compensated acceleration: {compensate_gravity(rotation, avg_accel)}')
        # print(f'Compensated acceleration: {compensate_gravity(rotation, avg_accel)}')
        
        # print(f'first_accel: {self.calibration_measurments[0].accelerometer[0]:+.5f} {self.calibration_measurments[0].accelerometer[1]:+.5f} {self.calibration_measurments[0].accelerometer[2]:+.5f}')
        # print(f'first_gyro: {self.calibration_measurments[0].gyroscope[0]:+.5f} {self.calibration_measurments[0].gyroscope[1]:+.5f} {self.calibration_measurments[0].gyroscope[2]:+.5f}')
        # print(f'first_mag: {self.calibration_measurments[0].magnetometer[0]:+.5f} {self.calibration_measurments[0].magnetometer[1]:+.5f} {self.calibration_measurments[0].magnetometer[2]:+.5f}')
        # print(f'first_temp: {self.calibration_measurments[0].temperature:+.2f}')
        # print()

        # print(f'last_accel: {self.measurment.accelerometer[0]:+.5f} {self.measurment.accelerometer[1]:+.5f} {self.measurment.accelerometer[2]:+.5f}')
        # print(f'last_gyro: {self.measurment.gyroscope[0]:+.5f} {self.measurment.gyroscope[1]:+.5f} {self.measurment.gyroscope[2]:+.5f}')
        # print(f'last_mag: {self.measurment.magnetometer[0]:+.5f} {self.measurment.magnetometer[1]:+.5f} {self.measurment.magnetometer[2]:+.5f}')
        # print(f'last_temp: {self.measurment.temperature:+.2f}')
        # print()

        print(f'avg_accel: {avg_accel[0]:+.5f} {avg_accel[1]:+.5f} {avg_accel[2]:+.5f}')
        print(f'avg_gyro: {avg_gyro[0]:+.5f} {avg_gyro[1]:+.5f} {avg_gyro[2]:+.5f}')
        print(f'avg_mag: {avg_mag[0]:+.5f} {avg_mag[1]:+.5f} {avg_mag[2]:+.5f}')
        print(f'avg_temp: {avg_temp:+.2f}')
        print()

        print(f'std_accel: {std_accel[0]:+.5f} {std_accel[1]:+.5f} {std_accel[2]:+.5f}')
        print(f'std_gyro: {std_gyro[0]:+.5f} {std_gyro[1]:+.5f} {std_gyro[2]:+.5f}')
        print(f'std_mag: {std_mag[0]:+.5f} {std_mag[1]:+.5f} {std_mag[2]:+.5f}')
        print(f'std_temp: {std_temp:+.2f}')
        print()

        print(f'gyro_offset: {self.gyrometer_offset[0]:+.5f} {self.gyrometer_offset[1]:+.5f} {self.gyrometer_offset[2]:+.5f}')
        print(f'gravity: {self.gravity:+.2f}')
        print(f'position: {position[0]:+.5f} {position[1]:+.5f} {position[2]:+.5f}')
        print(f'rotation: {rotation[0]:+.5f} {rotation[1]:+.5f} {rotation[2]:+.5f}')
        print(f'N_measurments: {len(self.calibration_measurments)}')
        print()

        self.measurment = IMUMeasurment()
        self.previous_measurment = IMUMeasurment()
        self.measurment_delta = IMUMeasurment()
        self.calibration_measurments = []

        self.measurment.rotation = rotation
        self.measurment.position = position

        print('Calibration done')
        self.calibration_done = True



def resizewin(width, height):
    """
    For resizing window
    """
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def draw(w, nx, ny, nz, x, y, z):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    drawText((-2.6, 1.8, 2), "GyroMouse", 18)
    drawText((-2.6, 1.6, 2), "Module to visualize quaternion or Euler angles data", 16)
    drawText((-2.6, -2, 2), "Press Escape to exit.", 16)

    if(useQuat):
        [yaw, pitch , roll] = quat_to_ypr([w, nx, ny, nz])
        drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
        glRotatef(2 * math.acos(w) * 180.00/math.pi, -1 * nx, nz, ny)
    else:
        yaw = nx
        pitch = ny
        roll = nz
        drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
        glRotatef(roll, 0.00, 0.00, 1.00)
        glRotatef(pitch, 1.00, 0.00, 0.00)
        glRotatef(yaw, 0.00, 1.00, 0.00)
        glTranslatef(x, z, y)

    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()


def drawText(position, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def quat_to_ypr(q):
    yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
    pitch = -math.asin(2.0 * (q[1] * q[3] - q[0] * q[2]))
    roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
    pitch *= 180.0 / math.pi
    yaw   *= 180.0 / math.pi
    yaw   -= -0.13  # Declination at Chandrapur, Maharashtra is - 0 degress 13 min
    roll  *= 180.0 / math.pi
    return [yaw, pitch, roll]

video_flags = OPENGL | DOUBLEBUF
pygame.init()
screen = pygame.display.set_mode((800, 600), video_flags)
pygame.display.set_caption("GyroMouse")
resizewin(800, 600)
init()
frames = 0
ticks = pygame.time.get_ticks()



gyro_mouse = GyroMouseSerial(port)
try:
    while True:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break


        gyro_mouse.read_and_process_data_line()

        frames += 1
except KeyboardInterrupt:
    print('\033[20B\nExiting...')
    gyro_mouse.serial.close()
except serial.SerialException as e:
    print('\033[20B\nSerial error, either device was disconnected or is in use by another program.', e)
    gyro_mouse.serial.close()
except Exception as e:
    print('\033[20B\n', e)
    gyro_mouse.serial.close()
    raise e
