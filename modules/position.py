#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: OPi-AT32-FC-OBCM
# Filename: position
# Created on: 2024/4/28

import threading
import time
import numpy as np
import math
from i2cylib.hid.anotc import AnoOpticalFlowSensor
from i2cylib.engineering import PID, IncPID

if __name__ == '__main__':
    from control import Controller, RC_MIN, RC_MAX, RC_MID
    from blackbox import BlackBoxLogger
else:
    from .control import Controller, RC_MIN, RC_MAX, RC_MID
    from .blackbox import BlackBoxLogger

"""
    +X
     |
+Y --+-- -Y
     |
    -X
"""

PID_X = (1.2, 0, 0.03)
PID_DX = (3.4, 0.7, 0.11)
PID_Y = (1.2, 0, 0.03)
PID_DY = (3.4, 0.7, 0.11)
PID_H = (1.0, 0, 0.01)
PID_DH = (0.47, 0.10, 0.08)

HOVER_THR = 700
ALT_MIN = 50
ALT_MAX = 150
ALT_SPEED_MAX = 100
XY_SPEED_MAX = 200

DOUBLE_PI = 6.283185307179586476925286766559


class PosCtl:

    def __init__(self, ctl: Controller, logger: BlackBoxLogger):
        """
        Position controller
        :param ctl: Controller interface
        """

        # logger
        self.logger = logger

        # hook on UAV control interface
        self.ctl = ctl

        # register this callback so the RC signals could transmit to PID controllers
        self.ctl.rx_dev.register_callback(self.__callback_transform_rc_to_motion_expectation)

        # initialize OFS
        self.ofs = AnoOpticalFlowSensor()

        # speed PIDs
        self.pid_dy = PID(*PID_DY, core_freq=250, dterm_lpf_cutoff_hz=12)
        self.pid_dy.out_limit = (-500, 500)
        self.pid_dy.integ_limit = (-500, 500)

        def level_ctl_core(pid: PID):  # new logger for pid_dx

            # calculate controllers remains (except for pid_dx)
            self.pid_dy.calc(pid.real_core_time)
            self.pid_x.calc(pid.real_core_time)
            self.pid_y.calc(pid.real_core_time)

            # position pid calculation
            self.pid_x.measures = self.ofs.ofs.x_ground
            self.pid_y.measures = self.ofs.ofs.y_ground

            # position control
            offset_angle = self.ofs.attitude.yaw - self.ofs.attitude.offset_yaw  # get current heading angle
            # speed_r = math.sqrt(self.pid_x.out ** 2 + self.pid_y.out ** 2)  # get speed
            # if speed_r == 0:
            #     cos_theta_ground = 1.0
            # else:
            #     cos_theta_ground = self.pid_x.out / speed_r  # calculate cos(theta) of speed in ground axis

            # # limit speed
            # if speed_r > self.max_horizontal_velocity_cm:
            #     speed_r = self.max_horizontal_velocity_cm

            # theta_real = math.acos(cos_theta_ground) - offset_angle  # calculate angle theta of real axis
            # cos_theta_real = math.cos(theta_real)  # calculate cos(real_theta)
            # sin_theta_real = math.sin(theta_real)  # calculate sin(real_theta)

            # self.pid_dx.expectation = cos_theta_real * speed_r  # transfer real axis motion signal to pid_dx
            # self.pid_dy.expectation = -sin_theta_real * speed_r  # transfer real axis motion signal to pid_dy

            cos_k = math.cos(offset_angle)
            sin_k = math.sin(offset_angle)
            self.pid_dx.expectation = self.pid_x.out * cos_k + self.pid_y.out * sin_k
            self.pid_dy.expectation = -self.pid_x.out * sin_k + self.pid_y.out * cos_k

            # logging
            self.logger.log(
                {
                    "dx_expectation": float(pid.expectation),
                    "dx_measurement": float(pid.measures),
                    "dx_output": float(pid.out),
                    "dx_integ": float(pid.integ),
                    "dx_dterm_mea": float(pid.dterm_lpf_val),

                    "dy_expectation": float(self.pid_dy.expectation),
                    "dy_measurement": float(self.pid_dy.measures),
                    "dy_output": float(self.pid_dy.out),
                    "dy_integ": float(self.pid_dy.integ),
                    "dy_dterm_mea": float(self.pid_dy.dterm_lpf_val),

                    "x_expectation": float(self.pid_x.expectation),
                    "x_measurement": float(self.pid_x.measures),
                    "x_output": float(self.pid_x.out),
                    "x_integ": float(self.pid_x.integ),
                    "x_dterm_mea": float(self.pid_x.dterm_lpf_val),

                    "y_expectation": float(self.pid_y.expectation),
                    "y_measurement": float(self.pid_y.measures),
                    "y_output": float(self.pid_y.out),
                    "y_integ": float(self.pid_y.integ),
                    "y_dterm_mea": float(self.pid_y.dterm_lpf_val),
                }
            )
            if self.ofs.ofs.quality > 150:
                pid.measures = self.ofs.ofs.dx
            self.ctl.signals.pitch = pid.out

            if self.ofs.ofs.quality > 150:
                self.pid_dy.measures = self.ofs.ofs.dy
            self.ctl.signals.roll = -self.pid_dy.out

        self.pid_dx = PID(*PID_DX, core_freq=250, dterm_lpf_cutoff_hz=12)
        self.pid_dx.out_limit = (-500, 500)
        self.pid_dx.integ_limit = (-500, 500)
        self.pid_dx.register_callback(level_ctl_core)  # register blackbox logger to PID core

        self.pid_dh = PID(*PID_DH, core_freq=20, dterm_lpf_cutoff_hz=8)
        self.pid_dh.out_limit = (-1000, 1000)
        self.pid_dh.integ_limit = (-2000, 2000)

        # position PIDs

        self.__h_mid_filter_window = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.__h_mid_filter_index = 0
        gaussian_filter_window_size = 5
        gaussian_k = 2 / (gaussian_filter_window_size + 1)

        self.__h_gaussian_filter_window = [0.0 for ele in range(gaussian_filter_window_size)]
        self.__h_gaussian_filter_weight = np.linspace(gaussian_k / gaussian_filter_window_size, gaussian_k,
                                                      gaussian_filter_window_size)

        self.__h_last_height = 0.0
        self.h_speed = 0.0
        self.__h_speed_prev_mea = 0.0
        self.h_speed_lpf_cutoff_hz = 7
        self.__h_speed_lpf_k = DOUBLE_PI * self.h_speed_lpf_cutoff_hz

        def height_ctl_core(pid: PID):  # new logger for pid_h
            name = "h"

            raw_height = self.ofs.distance.height
            if raw_height <= 0:
                # error correction
                raw_height = 0
            elif raw_height > 1000:
                # error correction
                raw_height = pid.measures + (self.pid_dh.dterm_lpf_val * pid.real_core_time)

            # median filtering
            # self.__h_mid_filter_window[self.__h_mid_filter_index] = raw_height
            # self.__h_mid_filter_index += 1
            # if self.__h_mid_filter_index == len(self.__h_mid_filter_window):
            #     self.__h_mid_filter_index = 0
            # med_height = np.median(self.__h_mid_filter_window)
            med_height = raw_height

            # gaussian filtering
            self.__h_gaussian_filter_window.pop(0)
            self.__h_gaussian_filter_window.append(med_height)
            height = sum(
                [ele * self.__h_gaussian_filter_weight[i] for i, ele in enumerate(self.__h_gaussian_filter_window)]
            )

            pid.measures = height
            h_speed_raw = (height - self.__h_last_height) / pid.real_core_time

            # error correction
            if h_speed_raw > 900 or h_speed_raw < -900:
                h_speed_raw = self.h_speed

            # speed loop LPF
            lpf_b = self.__h_speed_lpf_k * pid.real_core_time
            lpf_c = lpf_b / (lpf_b + 1)
            self.h_speed += lpf_c * (h_speed_raw - self.h_speed)

            self.__h_last_height = height

            # processing delta H
            self.pid_dh.expectation = pid.out
            self.pid_dh.measures = self.h_speed

            self.pid_dh.calc(pid.real_core_time)

            # add hover throttle base value to pid output for better controlling
            if self.landed and not self.landing and not self.taking_off:
                self.ctl.signals.throttle = 0  # basically to idle all motors
            elif self.landing and self.ofs.distance.height < 10:
                # to cancel the ground effect (decoupled)
                self.ctl.signals.throttle = (self.pid_dh.out + HOVER_THR * 0.95) / (self.ofs.attitude.cos_phi * 1.3)
            else:
                # normal in-air flight (decoupled)
                self.ctl.signals.throttle = (self.pid_dh.out + HOVER_THR) / (self.ofs.attitude.cos_phi * 1.3)

            # logging
            self.logger.log(
                {
                    "{}_mea_raw".format(name): float(raw_height),
                    # "{}_mea_median_filtered".format(name): float(med_height),

                    "{}_expectation".format(name): float(pid.expectation),
                    "{}_measurement".format(name): float(pid.measures),
                    "{}_output".format(name): float(pid.out),
                    "{}_integ".format(name): float(pid.integ),
                    "{}_dterm_mea".format(name): float(pid.dterm_lpf_val),

                    "d{}_mea_raw".format(name): float(h_speed_raw),
                    "d{}_expectation".format(name): float(self.pid_dh.expectation),
                    "d{}_measurement".format(name): float(self.pid_dh.measures),
                    "d{}_output".format(name): float(self.pid_dh.out),
                    "d{}_integ".format(name): float(self.pid_dh.integ),
                    "d{}_dterm_mea".format(name): float(self.pid_dh.dterm_lpf_val),
                }
            )

        self.pid_h = PID(*PID_H, core_freq=20, dterm_lpf_cutoff_hz=10)
        self.pid_h.integ_limit = (-2000, 2000)
        self.pid_h.register_callback(height_ctl_core)  # register blackbox logger to PID core

        self.max_horizontal_velocity_cm = XY_SPEED_MAX
        self.max_vertical_velocity_cm = ALT_SPEED_MAX

        self.pid_x = PID(*PID_X, core_freq=250, dterm_lpf_cutoff_hz=10)
        self.pid_y = PID(*PID_Y, core_freq=250, dterm_lpf_cutoff_hz=10)

        self.pid_x.out_limit = (-self.max_horizontal_velocity_cm, self.max_horizontal_velocity_cm)
        self.pid_y.out_limit = (-self.max_horizontal_velocity_cm, self.max_horizontal_velocity_cm)
        self.pid_h.out_limit = (-self.max_vertical_velocity_cm, self.max_vertical_velocity_cm)

        # flags
        self.threads_running = False
        self.paused = True
        self.landed = True
        self.taking_off = False
        self.landing = False
        self.__threads = []
        self.headless_mode = False
        self.returning_home = False

        self.__convert_kh = self.max_horizontal_velocity_cm * 2 / (RC_MAX - RC_MIN)
        self.__convert_kv = self.max_vertical_velocity_cm * 2 / (RC_MAX - RC_MIN)

        self.__timer_h = time.time()
        self.__cnt_ready_to_land = 0
        self.__cnt_ready_to_takeoff = 0
        self.__cnt_ready_to_set_landed = 0

        # tests
        self.test_pos = [
            [0, 0],
            [100, 0],
            [100, 100],
            [0, 100],
            [0, 0]
        ]
        self.test_start_pos = [0, 0]
        self.test_i = 0
        self.test_hold_t0 = 0
        self.test_hold_timeout = 1.0

    def start(self):
        """
        start controller
        :return:
        """
        self.threads_running = True
        self.ofs.start()  # initiate optical flow sensor receiver
        self.__threads.append(threading.Thread(target=self.__thread_mode_handler))
        [ele.start() for ele in self.__threads]

    def kill(self):
        """
        kill position controller
        :return:
        """
        self.threads_running = False
        [ele.join() for ele in self.__threads]
        self.pid_h.pause()
        self.pid_dx.pause()
        # self.pid_dy.pause()
        # self.pid_x.pause()
        # self.pid_y.pause()

    def set_speed_limit(self, horizontal_speed_limit=None, vertical_speed_limit=None):
        """
        set speed limit to controller in cm/s
        :param horizontal_speed_limit: X-Y speed limit in cm/s
        :param vertical_speed_limit: H speed limit in cm/s
        :return:
        """
        if horizontal_speed_limit is not None:
            self.max_horizontal_velocity_cm = horizontal_speed_limit
        if vertical_speed_limit is not None:
            self.max_vertical_velocity_cm = vertical_speed_limit

        self.__convert_kh = self.max_horizontal_velocity_cm * 2 / (RC_MAX - RC_MIN)
        self.__convert_kv = self.max_vertical_velocity_cm * 2 / (RC_MAX - RC_MIN)
        self.pid_x.out_limit = (-self.max_horizontal_velocity_cm, self.max_horizontal_velocity_cm)
        self.pid_y.out_limit = (-self.max_horizontal_velocity_cm, self.max_horizontal_velocity_cm)
        self.pid_h.out_limit = (-self.max_vertical_velocity_cm, self.max_vertical_velocity_cm)

    def __convert_rc_to_speed(self, thr, roll, pitch):
        """
        convert rc signals to speed in cm/s
        :param thr: int, RC value ranging from 0 to 2000
        :param roll: int, RC value ranging from 0 to 2000
        :param pitch: int, RC value ranging from 0 to 2000
        :return: v_speed, x_speed, y_speed
        """
        v_speed = self.__convert_kv * (thr - RC_MID)
        y_speed = self.__convert_kh * (RC_MID - roll)
        x_speed = self.__convert_kh * (pitch - RC_MID)

        return v_speed, x_speed, y_speed

    def __callback_transform_rc_to_motion_expectation(self, channels, flags):
        """
        transform RC signals to motion expectation
        :return:
        """
        t1 = time.time()
        dh = t1 - self.__timer_h
        self.__timer_h = t1
        # print(channels)
        v_speed, x_speed, y_speed = self.__convert_rc_to_speed(*channels[0:3])

        if -15 < v_speed < 15:
            # dead zone for vertical speed
            v_speed = 0

        if channels[6] > 1500:
            # enable headless control
            if not self.headless_mode:
                print('\nheadless mode')
                self.headless_mode = True
        else:
            if self.headless_mode:
                print('\nheading mode')
                self.headless_mode = False

        if channels[4] < 1500:
            # reset ofs position
            self.ofs.reset_position_zero()
            self.ofs.reset_yaw_direction_zero()

        if self.pid_h.expectation > 49 and self.taking_off and -2 < self.pid_h.err < 2:
            if self.landed:
                print('\nnow hovering')
                # toggle landed flag to True while taking off
                self.landed = False
                self.taking_off = False

        if not self.landed:
            # enable RC input in sky (if not landing)
            if not self.landing:
                # while not landing and not taking off

                if channels[5] > 1500:
                    if not self.returning_home:
                        # return home
                        print('\nnow returning')
                        self.returning_home = True
                        # self.pid_x.expectation = 0
                        # self.pid_y.expectation = 0
                        self.test_start_pos = [self.pid_x.measures, self.pid_y.measures]
                        self.test_i = 0
                        self.test_hold_t0 = time.time()
                else:
                    if self.returning_home:
                        print('\nholding current position (return canceled)')
                        self.returning_home = False
                        self.pid_x.expectation = self.pid_x.measures
                        self.pid_y.expectation = self.pid_y.measures

                if not self.returning_home:

                    if self.headless_mode:
                        self.pid_x.expectation += x_speed * dh
                        self.pid_y.expectation += y_speed * dh
                    else:
                        x_dis = x_speed * dh
                        y_dis = y_speed * dh
                        offset_angle = self.ofs.attitude.yaw - self.ofs.attitude.offset_yaw
                        cos_theta = math.cos(offset_angle)
                        sin_theta = math.sin(offset_angle)
                        self.pid_x.expectation += x_dis * cos_theta - y_dis * sin_theta
                        self.pid_y.expectation += x_dis * sin_theta + y_dis * cos_theta

                else:
                    # performing auto code
                    self.pid_x.expectation = self.test_start_pos[0] + self.test_pos[self.test_i][0]
                    self.pid_y.expectation = self.test_start_pos[1] + self.test_pos[self.test_i][1]

                    # wait for steady
                    if -4 < self.pid_x.err < 4 and -4 < self.pid_y.err < 4:
                        if time.time() - self.test_hold_t0 > self.test_hold_timeout:
                            self.test_i += 1

                    else:
                        self.test_hold_t0 = time.time()

                exp = self.pid_h.expectation
                exp += dh * v_speed

                # limit height max while not landing
                if exp >= ALT_MAX:
                    exp = ALT_MAX

                # limit height to 50cm while not landing
                if exp <= ALT_MIN:
                    exp = ALT_MIN
                    # detect landing command (by descending height expectation below 50cm for 2 seconds)
                    if channels[0] < RC_MIN + 100:
                        self.__cnt_ready_to_land += 1
                        if self.__cnt_ready_to_land > 300:
                            if not self.landing:
                                print("\nlanding")
                                self.landing = True
                                # dock on current position
                                self.pid_x.expectation = self.pid_x.measures
                                self.pid_y.expectation = self.pid_y.measures

                    else:
                        self.__cnt_ready_to_land = 0

                self.pid_h.expectation = exp

            else:
                # while landing
                exp = self.pid_h.expectation
                exp -= dh * 10  # landing with -10cm/s velocity

                if exp < -50:
                    exp = -50
                self.pid_h.expectation = exp

                if self.ofs.distance.height <= 3:
                    self.__cnt_ready_to_set_landed += 1
                    if self.__cnt_ready_to_set_landed > 150:
                        # toggle landed flag to True while landing
                        if not self.landed:
                            print("\nlanded")
                            self.landed = True
                            self.landing = False
                            self.pid_h.reset_i()
                            self.pid_x.reset_i()
                            self.pid_y.reset_i()
                            self.pid_dh.reset_i()
                            self.pid_dx.reset_i()
                            self.pid_dy.reset_i()

                else:
                    self.__cnt_ready_to_set_landed = 0

        else:
            # wait for takeoff command
            if channels[0] >= RC_MAX - 100:
                self.__cnt_ready_to_takeoff += 1
                if self.__cnt_ready_to_takeoff > 300:
                    if not self.taking_off:
                        print("\ntaking off")
                        self.taking_off = True
                        # dock on current position
                        self.pid_x.expectation = self.pid_x.measures
                        self.pid_y.expectation = self.pid_y.measures

            else:
                self.__cnt_ready_to_takeoff = 0

            if self.taking_off:
                exp = self.pid_h.expectation
                exp += dh * 5  # lift off with 5cm/s velocity
                if exp > 50:
                    exp = 50
                self.pid_h.expectation = exp

    def __thread_mode_handler(self):
        """
        monitor RC signals for mode switches
        :return:
        """
        while self.threads_running:

            if self.ctl.armed and not self.ctl.manual_control:
                # start PID controllers if armed and not in manual control mode
                if self.paused:
                    print("\nPID started")
                    self.paused = False
                    self.pid_h.start()
                    # self.pid_x.start()
                    # self.pid_y.start()
                    self.pid_dx.start()
                    # self.pid_dy.start()
                    self.pid_h.expectation = 0

            else:
                # pause PID controllers if not armed or in manual control mode
                if not self.paused:
                    print("\nPID stopped")
                    self.paused = True
                    self.pid_dx.pause()
                    # self.pid_dy.pause()
                    # self.pid_x.pause()
                    # self.pid_y.pause()
                    self.pid_h.pause()
                    # reset i for all pid controller (except dh)
                    self.pid_x.reset_i()
                    self.pid_y.reset_i()
                    self.pid_h.reset_i()
                    self.pid_dh.reset_i()
                    self.pid_dx.reset_i()
                    self.pid_dy.reset_i()
                    self.pid_h.expectation = 0
                    self.landed = True
                    self.taking_off = False
                    self.landing = False

            time.sleep(0.05)
