#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: OPi-AT32-FC-OBCM
# Filename: control
# Created on: 2024/5/8


from i2cylib.serial.sbus import SBUS, SBUS_SIGNAL_OK, SBUS_SIGNAL_LOST, SBUS_SIGNAL_FAILSAFE
import os
import time

if __name__ == '__main__':
    from blackbox import BlackBoxLogger
else:
    from .blackbox import BlackBoxLogger

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

    def __init__(self, rx_dev: str = "/dev/ttyACM0", tx_dev: str = "/dev/ttyACM1", blackbox: BlackBoxLogger = None):
        """
        control interface for OPi-AT32-FC-OBCM
        :param rx_dev: path to rx device
        :param tx_dev: path to tx device
        :param blackbox: BlackBoxLogger instance
        """
        init_gpio()  # initialize all gpio
        set_CM4_ready_LED(light_on=False)  # set ready signal to False
        switch_RC(to_OPi=False)  # switch RC signal to FC directly
        self.rx_dev = SBUS(rx_dev, multiprocess=False, rx=True, tx=False)  # initialize rx device
        self.__tx_dev = SBUS(tx_dev, multiprocess=False, rx=False, tx=True)  # initialize tx device

        self.rx_dev.register_callback(self.rc_signal_handler_callback)
        self.blackbox = blackbox

        self.takeoff = False

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
                thr = (self.throttle) * self.__convert_k + RC_MIN
                roll = (self.roll + 500) * self.__convert_k + RC_MIN
                pitch = (self.pitch + 500) * self.__convert_k + RC_MIN
                yaw = (self.yaw + 500) * self.__convert_k + RC_MIN

                # limit RC output value here
                if thr <= RC_MIN:
                    thr = RC_MIN
                elif thr >= RC_MAX:
                    thr = RC_MAX

                if roll <= RC_MIN:
                    roll = RC_MIN
                elif roll >= RC_MAX:
                    roll = RC_MAX

                if pitch <= RC_MIN:
                    pitch = RC_MIN
                elif pitch >= RC_MAX:
                    pitch = RC_MAX

                if yaw <= RC_MIN:
                    yaw = RC_MIN
                elif yaw >= RC_MAX:
                    yaw = RC_MAX

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
        try:
            new_ch = [ele for ele in channels]

            if new_ch[4] > 1500:
                if not self.armed:
                    self.armed = True
                    print("\nnow armed")
            else:
                if self.armed:
                    self.armed = False
                    print("\nnow disarmed")

            if flags != SBUS_SIGNAL_OK:
                if self.signal_good:
                    self.signal_good = False
                    print("\nsignal lost")
            else:
                if not self.signal_good:
                    self.signal_good = True
                    print("\nsignal good")

            if flags == SBUS_SIGNAL_OK and new_ch[7] < 1500:
                # passing new RC signals to FC
                thr, roll, pitch, yaw = self.signals.get_sbus_value()
                new_ch[0] = thr
                new_ch[1] = roll
                new_ch[2] = pitch
                new_ch[5] = RC_MID

            if new_ch[7] > 1500:
                if not self.manual_control:
                    self.manual_control = True
                    print("\nmanual control")
            else:
                if self.manual_control:
                    self.manual_control = False
                    print("\nauto control")

            if self.blackbox is not None:
                if self.armed:
                    self.blackbox.log(
                        {
                            "rc_rx_thr": channels[0],
                            "rc_rx_roll": channels[1],
                            "rc_rx_pitch": channels[2],
                            "rc_rx_yaw": channels[3],
                            "rc_rx_ch5": channels[4],
                            "rc_rx_ch6": channels[5],
                            "rc_rx_ch7": channels[6],
                            "rc_rx_ch8": channels[7],
                            "rc_rx_ch9": channels[8],
                            "rc_rx_ch10": channels[9],
                            "rc_rx_ch11": channels[10],
                            "rc_rx_ch12": channels[11],
                            "rc_rx_ch13": channels[12],
                            "rc_rx_ch14": channels[13],
                            "rc_rx_ch15": channels[14],
                            "rc_rx_ch16": channels[15],
                            "rc_rx_flag": flags,
                            "rc_tx_thr": new_ch[0],
                            "rc_tx_roll": new_ch[1],
                            "rc_tx_pitch": new_ch[2],
                            "rc_tx_yaw": new_ch[3],
                            "rc_tx_ch5": new_ch[4],
                            "rc_tx_ch6": new_ch[5],
                            "rc_tx_ch7": new_ch[6],
                            "rc_tx_ch8": new_ch[7],
                            "rc_tx_ch9": new_ch[8],
                            "rc_tx_ch10": new_ch[9],
                            "rc_tx_ch11": new_ch[10],
                            "rc_tx_ch12": new_ch[11],
                            "rc_tx_ch13": new_ch[12],
                            "rc_tx_ch14": new_ch[13],
                            "rc_tx_ch15": new_ch[14],
                            "rc_tx_ch16": new_ch[15]
                        }
                    )
                else:
                    self.blackbox.log(
                        {
                            "rc_rx_ch5": channels[4],
                            "rc_tx_ch5": new_ch[4],
                            "rc_rx_flag": flags,
                        }
                    )

            self.__tx_dev.update_channels_and_flag(new_ch, flags)
        except Exception as e:
            print("\nerror while handling RC signal, {}".format(e))

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
