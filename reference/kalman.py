#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: OPi-AT32-FC-OBCM
# Filename: kalman
# Created on: 2024/5/17


import numpy as np
import matplotlib.pyplot as plt

"""
Q 系统噪声
R 测量噪声
X(k|k-1)   上一次状态预测结果
X(k-1|k-1) 上一时刻的最优预测值
P(k|k-1)   X(k|k-1)对应的convariance协方差
P(k-1|k-1) X(k-1|k-1) 对应的convariance协方差
"""

x_last = 0
p_last = 0
Q = 0.1  # 系统噪声
R = 0.5  # 测量噪声


def kalman(z_measure, x_last=0, p_last=0, Q=0.018, R=0.0542):
    x_mid = x_last
    p_mid = p_last + Q
    kg = p_mid / (p_mid + R)
    x_now = x_mid + kg * (z_measure - x_mid)
    p_now = (1 - kg) * p_mid
    p_last = p_now
    x_last = x_now
    return x_now, p_last, x_last


real = np.sin(np.linspace(0, 10, 100))
chao = np.random.rand(100) - 0.5
x = real + chao
y = []
for i in range(len(x)):
    pred, p_last, x_last = kalman(x[i], x_last, p_last, Q, R)
    y.append(pred)

plt.plot(real, color="b")  # 真实值
plt.plot(x, color="g")  # 测量值
plt.plot(y, color="r")  # 预测值
plt.show()


