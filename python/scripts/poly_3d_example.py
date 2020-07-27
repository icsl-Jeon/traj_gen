#!/usr/bin/env python
# coding=utf-8

import numpy as np
from traj_gen import poly_trajectory as pt
import time

if __name__ == '__main__':
    dim = 3
    knots = np.array([0.0, 2.0, 4.0, 5.0, 7.0])
    order = 8
    optimTarget = 'end-derivative' #'end-derivative' 'poly-coeff'
    maxConti = 4
    objWeights = np.array([1, 1, 1])
    pTraj = pt.PolyTrajGen(knots, order, optimTarget, dim, maxConti)

    # 2. Pin
    ts = np.array([0.0, 2.0, 4.0, 5.0, 7.0])
    Xs = np.array([[0.0, 0.0, 0.0],
                    [2.0, -1.0, 2.0],
                    [5.0, 3.0, 4.0],
                    [6., 5., 5.5],
                    [7.0, -5, 5]])
    Xdot = np.array([0, 0, 0])
    Xddot = np.array([0, 0, 0])

    # create pin dictionary
    for i in range(Xs.shape[0]):
        pin_ = {'t':ts[i], 'd':0, 'X':Xs[i]}
        pTraj.addPin(pin_)
    pin_ = {'t':ts[0], 'd':1, 'X':Xdot,}
    pTraj.addPin(pin_)
    pin_ = {'t':ts[0], 'd':2, 'X':Xddot,}
    pTraj.addPin(pin_)
    passCube = np.array([[3.0, 4.2], [-3.0, -2.], [ 1., 2] ]) # in shape [dim, xxx]
    pin_ = {'t':3, 'd':0, 'X':passCube}
    pTraj.addPin(pin_)
    passCube1 = np.array([[6.0, 7.0],[2.0, 3.0], [5.0, 5.7]])
    pin_ = {'t':6, 'd':0, 'X':passCube1}
    pTraj.addPin(pin_)

    # solve
    pTraj.setDerivativeObj(objWeights)
    print("solving")
    time_start = time.time()
    pTraj.solve()
    time_end = time.time()
    print(time_end - time_start)

    # plot
    ## showing trajectory
    print("trajectory")
    pTraj.showTraj(4)
    print('path')
    fig_title ='poly order : {0} / max continuity: {1} / minimized derivatives order: {2}'.format(pTraj.N, pTraj.maxContiOrder, np.where(pTraj.weight_mask>0)[0].tolist())
    pTraj.showPath(fig_title)