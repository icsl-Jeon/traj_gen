#!/usr/bin/env python
# coding=utf-8

import numpy as np
from traj_gen import optim_trajectory as opt_t
import time

if __name__ == '__main__':
    dim = 3
    knots = np.array([0.0, 7.0])
    pntDensity = 5
    objWeights = np.array([1, 1, 1])
    optTraj = opt_t.OptimTrajGen(knots, dim, pntDensity)

    # 2. Pin
    # ts = np.array([0.0, 2.0, 4.0, 7.0])
    # Xs = np.array([[0.0, 0.0, 0.0],
    #                 [2.0, -1.0, 2.0],
    #                 [5.0, 3.0, 4.0],
    #                 [7.0, -5, 5]])
    ts = np.array([0.0, 2.0, 4.0, 7.0])
    Xs = np.array([[0.0, 0.0, 0.0],
                    [2.0, -1.0, 2.0],
                    [5.0, 3.0, 4.0],
                    # [6., 6., 5.5],
                    [7.0, -5, 5]])
    Xdot = np.array([0, 0, 0])
    Xddot = np.array([0, 0, 0])

    # create pin dictionary
    for i in range(Xs.shape[0]):
        pin_ = {'t':ts[i], 'd':0, 'X':Xs[i]}
        optTraj.addPin(pin_)
    pin_ = {'t':ts[0], 'd':1, 'X':Xdot,}
    optTraj.addPin(pin_)
    pin_ = {'t':ts[-1], 'd':2, 'X':Xddot,}
    optTraj.addPin(pin_)
    passCube = np.array([[3.0, 4.2], [-3.0, -2.], [ 1., 2] ]) # in shape [dim, xxx]
    pin_ = {'t':3, 'd':0, 'X':passCube}
    optTraj.addPin(pin_)
    passCube1 = np.array([[6.0, 7.0],[2.0, 3.0], [5.0, 5.7]])
    pin_ = {'t':6, 'd':0, 'X':passCube1}
    optTraj.addPin(pin_)
    passCube2 = np.array([[6.1, 7.1],[2.5, 3.0], [5.3, 5.7]])
    pin_ = {'t':6.5, 'd':0, 'X':passCube1}
    optTraj.addPin(pin_)

    # solve
    optTraj.setDerivativeObj(objWeights)
    print("solving")
    time_start = time.time()
    optTraj.solve()
    time_end = time.time()
    print(time_end - time_start)

    # # plot
    # ## showing trajectory
    print("trajectory")
    optTraj.showTraj(4)
    print('path')
    fig_title = 'minimzed derivatives order: {}'.format(np.where(optTraj.weight_mask>0)[0].tolist())
    optTraj.showPath(fig_title)