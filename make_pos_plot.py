#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: OPi-AT32-FC-OBCM
# Filename: make_pos_plot
# Created on: 2024/6/7


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

    print("processing blackbox traces X-exp......", end="")
    x_exp = [ele for ele in blackbox.get_trace('x_expectation')]
    print("done")
    print("processing blackbox traces Y-exp......", end="")
    y_exp = [ele for ele in blackbox.get_trace('y_expectation')]
    print("done")
    print("processing blackbox traces X-mea......", end="")
    x_mea = [ele for ele in blackbox.get_trace('x_measurement')]
    print("done")
    print("processing blackbox traces Y-mea......", end="")
    y_mea = [ele for ele in blackbox.get_trace('y_measurement')]
    print("done")

    print("processing data......", end="")

    slices_index = []
    for i, val in enumerate(x_exp[1:]):
        if val[1] - x_exp[i][1] > 50:
            slices_index.append(i)
        if y_exp[i + 1][1] - y_exp[i][1] < -50:
            slices_index.append(i + 750)

    slices_index = [(slices_index[i], slices_index[i + 1]) for i in range(0, len(slices_index), 2)]
    sliced_x_exp = [x_exp[ele[0]:ele[1]] for ele in slices_index]
    sliced_y_exp = [y_exp[ele[0]:ele[1]] for ele in slices_index]
    sliced_x_mea = [x_mea[ele[0]:ele[1]] for ele in slices_index]
    sliced_y_mea = [y_mea[ele[0]:ele[1]] for ele in slices_index]

    offset_fixed_x_exp = []
    offset_fixed_y_exp = []
    offset_fixed_x_mea = []
    offset_fixed_y_mea = []
    for i, ele in enumerate(sliced_x_exp):
        l_x_exp = [ele[1] - sliced_x_exp[i][0][1] for ele in sliced_x_exp[i]]
        l_y_exp = [ele[1] - sliced_y_exp[i][0][1] for ele in sliced_y_exp[i]]
        l_x_mea = [ele[1] - sliced_x_mea[i][0][1] + ((-1) ** i) * 0.6 * ((ele[0] - sliced_x_mea[i][0][0])) for ele in sliced_x_mea[i]]
        l_y_mea = [ele[1] - sliced_y_mea[i][0][1] + ((-1) ** i) * 0.6 * ((ele[0] - sliced_y_mea[i][0][0])) for ele in sliced_y_mea[i]]
        offset_fixed_x_exp.append(l_x_exp)
        offset_fixed_y_exp.append(l_y_exp)
        offset_fixed_x_mea.append(l_x_mea)
        offset_fixed_y_mea.append(l_y_mea)

    print("done")

    print("{} tests found, making plot......".format(len(slices_index)))

    for i, ele in enumerate(offset_fixed_x_exp):
        plt.plot(offset_fixed_y_mea[i], offset_fixed_x_mea[i], alpha=0.4, color="blue")

    plt.plot(offset_fixed_y_exp[0], offset_fixed_x_exp[0], label="expecting_trace", color="red", linestyle="--")

    plt.ylabel('Y-Position (cm)')
    plt.xlabel('X-Position (cm)')
    plt.legend()
    plt.subplots_adjust(left=0.17, right=0.95, top=0.95, bottom=0.15)
    plt.grid(True)
    plt.show()



if __name__ == '__main__':
    main()
