#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: OPi-AT32-FC-OBCM
# Filename: read_blackbox.py
# Created on: 2024/5/13


from modules.blackbox import BlackBoxReader
import pathlib
from matplotlib import pyplot as plt
import os

from proplot import rc

rc["style"] = "default"
# 统一设置字体
rc["font.family"] = "Times New Roman"
# 统一设置轴刻度标签的字体大小
rc['tick.labelsize'] = 18
# 统一设置xy轴名称的字体大小
rc["axes.labelsize"] = 22
# 统一设置图例字体大小
rc["legend.fontsize"] = 18
# 统一设置轴刻度标签的字体粗细
rc["axes.labelweight"] = "light"
# 统一设置xy轴名称的字体粗细
rc["tick.labelweight"] = "bold"


def main():
    print("searching for blackbox files...")
    directory = pathlib.Path(__file__).parent.absolute().joinpath('blackbox')
    file_list = directory.glob('*.obbl')
    blackbox_list = []
    print("blackbox files found:")
    for i, file in enumerate(file_list):
        try:
            bb = BlackBoxReader(file.as_posix())
            if len(bb.list_columns()) > 4:
                print(" {}. {} ({:.2f} MB)".format(len(blackbox_list) + 1, file.name,
                                                   os.path.getsize(file.as_posix()) / 1_048_576))
                blackbox_list.append(bb)
        except Exception as e:
            continue

    sel = int(input("Select blackbox files to read (type desired index): ")) - 1
    blackbox = blackbox_list[sel]

    print("traces found:")
    for i, name in enumerate(blackbox.list_columns()):
        print(" {}. {}".format(i + 1, name))

    for i in range(len(blackbox.list_columns())):
        sel = input("Select trace to read (type desired index or multiple one seperated with space): ")
        sel = sel.replace(",", " ")
        sel = sel.replace("  ", " ")
        if " " in sel:
            sel = sel.split(" ")
        else:
            sel = [sel]

        for ele in sel:
            tr = blackbox.get_trace(blackbox.list_columns()[int(ele) - 1])
            x = []
            y = []
            for trace in tr:
                x.append(trace[0])
                y.append(trace[1])

            plt.plot(x, y, label=tr.name)


        if input("show plot? (type q to show)") == 'q':
            break

    plt.ylabel('Position (cm)')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.subplots_adjust(left=0.17, right=0.95, top=0.95, bottom=0.15)
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()
