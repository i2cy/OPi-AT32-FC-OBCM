#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: OPi-AT32-FC-OBCM
# Filename: position
# Created on: 2024/4/28
import threading
import time
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
        def logger_dx(pid: PID):  # new logger for pid_dx
            name = "dx"
            self.logger.log(
                {
                    "{}_expectation".format(name): pid.expectation,
                    "{}_measurement".format(name): pid.measures,
                    "{}_output".format(name): pid.out,
                    "{}_integ".format(name): pid.integ
                }
            )
            if self.ofs.ofs.quality > 190:
                pid.measures = self.ofs.ofs.dx

        self.pid_dx = PID(kp=0.3, ki=0.1, core_freq=250)
        self.pid_dx.register_callback(logger_dx)  # register blackbox logger to PID core

        def logger_dy(pid: PID):  # new logger for pid_dy
            name = "dy"
            self.logger.log(
                {
                    "{}_expectation".format(name): pid.expectation,
                    "{}_measurement".format(name): pid.measures,
                    "{}_output".format(name): pid.out,
                    "{}_integ".format(name): pid.integ
                }
            )
            if self.ofs.ofs.quality > 190:
                pid.measures = self.ofs.ofs.dy

        self.pid_dy = PID(kp=0.3, ki=0.1, core_freq=250)
        self.pid_dy.register_callback(logger_dy)  # register blackbox logger to PID core

        # position PIDs
        def logger_h(pid: PID):  # new logger for pid_h
            name = "h"
            self.logger.log(
                {
                    "{}_expectation".format(name): pid.expectation,
                    "{}_measurement".format(name): pid.measures,
                    "{}_output".format(name): pid.out,
                    "{}_integ".format(name): pid.integ
                }
            )
            height = self.ofs.distance.height
            if height <= 0:
                height = 0
            pid.measures = height

        self.pid_h = PID(kp=0.3, kd=0.0, ki=0.0, core_freq=50)
        self.pid_h.register_callback(logger_h)  # register blackbox logger to PID core

        # self.pid_x = PID(kp=0.3, core_freq=50)
        # self.pid_y = PID(kp=0.3, core_freq=50)

        # flags
        self.threads_running = False
        self.paused = True
        self.__threads = []

        self.max_horizontal_velocity_cm = 200
        self.max_vertical_velocity_cm = 100
        self.__convert_kh = self.max_horizontal_velocity_cm * 2 / (RC_MAX - RC_MIN)
        self.__convert_kv = self.max_vertical_velocity_cm * 2 / (RC_MAX - RC_MIN)

        self.__timer_h = time.time()

    def start(self):
        """
        start controller
        :return:
        """
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
        self.pid_dy.pause()
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
        v_speed, x_speed, y_speed = self.__convert_rc_to_speed(*channels[0:3])

        if -10 < v_speed < 10:
            # dead zone for vertical speed
            v_speed = 0

        self.pid_dx.expectation = x_speed
        self.pid_dy.expectation = y_speed
        self.pid_h.expectation += dh * v_speed

    def __thread_mode_handler(self):
        """
        monitor RC signals for mode switches
        :return:
        """
        while self.threads_running:

            if self.ctl.armed and not self.ctl.manual_control:
                # start PID controllers if armed and not in manual control mode
                if self.paused:
                    self.paused = False
                    self.pid_h.start()
                    # self.pid_x.start()
                    # self.pid_y.start()
                    self.pid_dx.start()
                    self.pid_dy.start()

            else:
                # pause PID controllers if not armed or in mannual control mode
                if not self.paused:
                    self.paused = True
                    self.pid_dx.pause()
                    self.pid_dy.pause()
                    # self.pid_x.pause()
                    # self.pid_y.pause()
                    self.pid_h.pause()
                    # reset i for x-axis and y-axis
                    # self.pid_x.reset_i()
                    # self.pid_y.reset_i()
                    self.pid_dx.reset_i()
                    self.pid_dy.reset_i()

            time.sleep(0.05)

