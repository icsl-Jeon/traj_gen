#!/usr/bin/env python
# coding=utf-8

import numpy as np
from abc import abstractmethod
from matplotlib import pyplot as plt
from matplotlib import rc
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D

class TrajGen(object):
    def __init__(self, knots_, dim_,):
        # dimension of curve
        self.dim = dim_
        # time knots
        self.Ts = knots_
        # set of pin
        self.pinSet = None
        # is solved?
        self.isSolved = False
        # time nkots (length = M for polyTrajGen and length 2 for optimalTraj)
        self.weight_mask = None
        # pin sets
        ## fixed
        self.fixPinSet = {}
        self.loosePinSet = {}
        self.fixPinOrder = {}

    @abstractmethod
    def setDerivativeObj(self, weight):
        pass

    @abstractmethod
    def solve(self,):
        pass

    @abstractmethod
    def eval(self, t, d):
        pass

    def addPin(self, pin):
        assert pin['X'].shape[0] == self.dim, "dim of pin val != dim of this TrajGen"
        assert (pin['t']>=self.Ts[0] and pin['t']<=self.Ts[-1]), 't of this pin is out of range of knots'
        if self.pinSet is not None:
            self.pinSet.append(pin)
        else:
            self.pinSet = [pin]

    def addPinSet(self, pinSet_):
        for pin in pinSet_:
            self.addPin(pin)

    # def draw3DBox(self,):


    def showPath(self, fig_title):
        assert self.dim >=2 and self.dim < 4, 'Here you can only show the path in 2/3D.'
        rc('text', usetex=True)
        fig = plt.figure()
        if self.dim == 3:
            ax = fig.gca(projection='3d')
        else:
            ax = plt.gca(projection='2d')
        # draw pin set
        for pin in self.pinSet:
            if pin['d'] == 0:
                X_ = pin['X']
                if len(X_.shape) == 2:
                    ## loose pin
                    x_ = X_[0, 0]
                    y_ = X_[1, 0]
                    x_size_ = X_[0, 1] - X_[0, 0]
                    y_size_ = X_[1, 1] - X_[1, 0]
                    ### 2D
                    if self.dim == 2:
                        ax.add_patch(patches.Rectangle((x_, y_), x_size_, y_size_), facecolor='r', alpha=0.1)
                    ### 3D cube/bar
                    else:
                        z_ = X_[2, 0]
                        z_size_ = X_[2, 1] - X_[2, 0]
                        ax.bar3d(x_, y_, z_, x_size_, y_size_, z_size_, color='r', alpha=0.1)
                else:
                    ## fix pin
                    ### 2D
                    if self.dim == 2:
                        ax.scatter(X_[0], X_[1], color='b', marker='o',)
                    ### 3D
                    else:
                        ax.scatter(X_[0], X_[1], X_[2], color='b', marker='o',)
        # draw curve
        if self.isSolved:
            N_plot = 100
            ts = np.linspace(self.Ts[0], self.Ts[-1], N_plot)
            Xs = self.eval(ts, 0)
            if self.dim == 2:
                ax.plot(Xs[0], Xs[1], 'k-')
            else:
                ax.plot(Xs[0], Xs[1], Xs[2], 'k-')
        ax.set_xlabel(r'$x$')
        ax.set_ylabel(r'$y$')
        ax.set_zlabel(r'$z$')
        ax.set_title(fig_title)
        plt.show()


    def showTraj(self, plotOrder):
        assert plotOrder>=0, 'Invalid plot order'
        # plt.figure()
        ax_dict = {}
        fig_dict = {}
        rc('text', usetex=True)
        # rc('font', family='serif')
        # create subfigures in (dim, order+1)
        fig, axs = plt.subplots(self.dim, plotOrder+1)

        # print pins
        for dd in range(self.dim):

            for pin in self.pinSet:
                t_ = pin['t']
                d_ = pin['d']
                X_ = pin['X']
                axs[dd, d_].vlines(t_, -10.0, 10, color='k', linestyle='dashed', linewidth=.5)
                if len(X_.shape) == 2:
                    # loose pin
                    axs[dd, d_].errorbar(x=t_, y=np.mean(X_[dd]), yerr=X_[dd, 1]-X_[dd, 0], ecolor='k', linewidth=3, elinewidth=2, capsize=4)
                else:
                    # fix pin
                    axs[dd, d_].scatter(x=t_, y=X_[dd], color='r', marker='.', )

        # print curves
        title_list = [r'$x^{('+str(i)+')}$' for i in range(plotOrder+1)]
        if self.isSolved:
            for d in range(plotOrder+1):
                N_plot = 50
                ts = np.linspace(self.Ts[0], self.Ts[-1], N_plot)
                Xs = self.eval(ts, d)
                axs[0, d].set_title(title_list[d])
                for dd in range(self.dim):
                    if d > 0:
                        axs[dd, d].hlines(y=0.0, xmin=self.Ts[0]-0.5, xmax=self.Ts[-1]+0.5, colors='r', linestyles='dashed')
                    axs[dd, d].plot(ts, Xs[dd], 'k-')
                    for t_ in self.Ts:
                        axs[dd, d].vlines(x=t_, ymin=np.min(Xs[dd])-0.5, ymax=np.max(Xs[dd])+0.5, color='k', linestyle='dashed', linewidth=0.3)
                    axs[dd, d].set_xlim(self.Ts[0]-0.5, self.Ts[-1]+0.5)
                    axs[dd, d].set_ylim(np.min(Xs[dd])-0.5, np.max(Xs[dd])+0.5)
                    axs[dd, d].set_xlabel('t')
                    # axs[dd, d].rc('text', usetex=True)
        plt.show()


