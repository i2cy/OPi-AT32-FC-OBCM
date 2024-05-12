#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: OPi-AT32-FC-OBCM
# Filename: control
# Created on: 2024/5/8


from i2cylib.serial.sbus import SBUS, SBUS_SIGNAL_OK, SBUS_SIGNAL_LOST, SBUS_SIGNAL_FAILSAFE
import os

CM4_READY = 22
RC_SEL = 20

RC_MIN = 174
RC_MID = 992
RC_MAX = 1811


def init_gpio():
    """
    initialize all gpio that used by OPi-AT32-FC-OBCM
    :return:
    """
    os.system("gpio mode {} out".format(RC_SEL))
    os.system("gpio mode {} out".format(CM4_READY))


def switch_RC(to_OPi=True):
    """
    switch RC signal to OPi-AT32-FC-OBCM
    :param to_OPi: bool, default True
    :return:
    """
    if to_OPi:
        os.system("gpio write {} 0".format(RC_SEL))
    else:
        os.system("gpio write {} 1".format(RC_SEL))


def set_CM4_ready_LED(light_on=True):
    """
    set CM4 ready LED
    :param light_on: bool, default True
    :return:
    """
    if light_on:
        os.system("gpio write {} 1".format(CM4_READY))
    else:
        os.system("gpio write {} 0".format(CM4_READY))


class Controller:

    def __init__(self, rx_dev: str = "/dev/ttyACM0", tx_dev: str = "/dev/ttyACM1"):
        """
        control interface for OPi-AT32-FC-OBCM
        :param rx_dev: path to rx device
        :param tx_dev: path to tx device
        """
        init_gpio()  # initialize all gpio
        set_CM4_ready_LED(light_on=False)  # set ready signal to False
        switch_RC(to_OPi=False)  # switch RC signal to FC directly
        self.rx_dev = SBUS(rx_dev, multiprocess=False, rx=True, tx=False)  # initialize rx device
        self.__tx_dev = SBUS(tx_dev, multiprocess=False, rx=False, tx=True)  # initialize tx device

        self.rx_dev.register_callback(self.rc_signal_handler_callback)

        class Signals:
            """
            throttle: 0 - 1000
            roll, pitch, yaw: -500 - 500
            """
            throttle = 0
            roll = 0
            pitch = 0
            yaw = 0

            __convert_k = (RC_MAX - RC_MIN) / 1000

            def get_sbus_value(self):
                """
                convert value to sbus value
                :return: thr, roll, pitch, yaw
                """
                thr = self.throttle * self.__convert_k + RC_MIN
                roll = (self.roll + 500) * self.__convert_k + RC_MIN
                pitch = (self.pitch + 500) * self.__convert_k + RC_MIN
                yaw = (self.yaw + 500) * self.__convert_k + RC_MIN

                return int(thr), int(roll), int(pitch), int(yaw)

        self.armed = False
        self.signal_good = False
        self.manual_control = True
        self.signals = Signals()

    def rc_signal_handler_callback(self, channels, flags):
        """
        callback function for RC signal handling
        :param channels: list, 16-channels of RC signal
        :param flags: int, signal
        :return:
        """
        if channels[4] > 1500:
            self.armed = True
        else:
            self.armed = False

        if flags != SBUS_SIGNAL_OK:
            self.signal_good = False

        if flags == SBUS_SIGNAL_OK and channels[7] < 1500:
            thr, roll, pitch, yaw = self.signals.get_sbus_value()
            channels[0] = thr
            channels[1] = roll
            channels[2] = pitch
            channels[3] = yaw

        if channels[7] > 1500:
            self.manual_control = True
        else:
            self.manual_control = False

        self.__tx_dev.update_channels_and_flag(channels, flags)

    def start(self):
        """
        start OPi-AT32-FC-OBCM controller
        :return:
        """
        self.rx_dev.start()
        time.sleep(0.5)
        self.__tx_dev.start()
        switch_RC(to_OPi=True)
        set_CM4_ready_LED(light_on=True)

    def stop(self):
        """
        stop OPi-AT32-FC-OBCM controller
        :return:
        """
        switch_RC(to_OPi=False)
        self.rx_dev.kill()
        self.__tx_dev.kill()
        set_CM4_ready_LED(light_on=False)


if __name__ == '__main__':  # unit test
    import time
    ctl = Controller()
    ctl.start()
    time.sleep(1)
    print("set throttle to 200")
    ctl.signals.throttle = 200
    time.sleep(1)
    print("set pitch to 100")
    ctl.signals.pitch = 100
    time.sleep(1)
    print("set yaw to 100")
    ctl.signals.yaw = 100
    time.sleep(1)
    print("set roll to 100")
    ctl.signals.roll = 100
    time.sleep(1)
    input("Press enter to stop")
    print("set all controls to 0")
    ctl.signals.throttle = 0
    ctl.signals.roll = 0
    ctl.signals.pitch = 0
    ctl.signals.yaw = 0
    ctl.stop()
