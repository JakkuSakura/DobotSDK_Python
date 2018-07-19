# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
from scipy import optimize


def get_dis_map():
    with open("data/distance_map.txt", "r") as f:
        data = f.readlines()
    spts = []
    for i, e in enumerate(data):
        spts.append(e.split())

    dis_map = {}
    for i, e in enumerate(spts):
        if e[0] == 'init':
            pass
        else:
            dis_map[int(e[0])] = int(round(float(e[1]) - float(spts[i - 1][1]), 3) * 10)
    return dis_map


dis_map = get_dis_map()
y = np.array(list(dis_map.values()))
x = np.array(list(dis_map.keys()))
func = lambda x_, k_, b_: k_ * x_ + b_
k, b = optimize.curve_fit(func, x, y)[0]


def get_dis_tick(mm):
    return (mm - b) / k


if __name__ == '__main__':
    print(dis_map)
    print(get_dis_tick(542 / 6))
    # 插值法之后的x轴值，表示从0到9间距为0.5的18个数
    xnew = np.arange(0, 21000, 20)
    ynew = np.array([func(e, k, b) for e in xnew])

    plt.plot(x, y, 'ro-')
    plt.plot(xnew, ynew)
    plt.show()
