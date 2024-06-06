#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: OPi-AT32-FC-OBCM
# Filename: main
# Created on: 2024/4/28


import time
from modules.blackbox import BlackBoxLogger
from modules.control import Controller, set_CM4_ready_LED, switch_RC
from modules.position import PosCtl
import os


def main():
    # turn off NTP service (in case of on flight updating)
    print("stopping NTP service......", end="")
    os.system("sudo timedatectl set-ntp false")
    print("done")

    # initialize
    print("initializing blackbox......", end="")
    blackbox = BlackBoxLogger()
    print("done")

    print("initializing controller......", end="")
    ctl = Controller(blackbox=blackbox)
    print("done")

    print("initializing position controller......", end="")
    posCtl = PosCtl(ctl, blackbox)
    print("done")

    # start controllers
    print("starting controllers......", end="")
    ctl.start()
    blackbox.start()
    posCtl.start()
    print("done")

    print("Flight Info:")
    try:
        while True:
            print("\rCurHeight: {:.2f}cm, DesHeight: {:.2f}cm, "
                  "H_Speed: {:.2f}cm, CurX: {:.2f}cm, CurY: {:.2f}, DesX: {:.2f}, DesY: {:.2f}, DesDx: {:.2f}"
                  ", DesDy: {:.2f}".format(
                posCtl.pid_h.measures, posCtl.pid_h.expectation, posCtl.pid_dh.measures,
                posCtl.pid_x.measures, posCtl.pid_y.measures, posCtl.pid_x.expectation, posCtl.pid_y.expectation,
                posCtl.pid_dx.expectation, posCtl.pid_dy.expectation
            ), end="")
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass

    print("Stopping controllers......", end="")
    set_CM4_ready_LED(False)
    switch_RC(False)
    posCtl.kill()
    ctl.stop()
    time.sleep(0.5)
    blackbox.kill()
    time.sleep(0.5)
    print("done")

    print("starting NTP service......", end="")
    os.system("sudo timedatectl set-ntp true")
    print("done")


if __name__ == '__main__':
    main()
