#!/usr/bin/env python
# coding=utf-8

from .traj_gen_base import TrajGen
import numpy as np
import casadi as ca
from scipy.interpolate import interp1d
class OptimTrajGen(TrajGen):
    def __init__(self, knots_, dim_, pntDensity_):
        super().__init__(knots_, dim_)
        self.pntDensity = pntDensity_
        assert knots_.shape[0]==2, 'For optimalTraj, knots = [t0, tf]'
        self.num_variables = int(np.floor((knots_[-1]-knots_[0])*pntDensity_))
        self.dt = (knots_[-1]-knots_[0])/(self.num_variables-1)
        self.ts = np.linspace(knots_[0], knots_[-1], self.num_variables) # different from Ts
        self.Xs = np.zeros((self.dim, self.num_variables))

    def findStepIndex(self, t):
        time_diff = (self.ts-t)**2
        return np.where(time_diff==np.min(time_diff))[0][0]

    def setDerivativeObj(self, weight_mask):
        self.weight_mask = weight_mask

    def addPin(self, pin_):
        if pin_['d'] >= self.num_variables:
            print("Warning: The degree of the pin exceed the total number of variables. This pin ignored\n")
        super().addPin(pin_)
        X_ = pin_['X']
        m = 0
        if len(X_.shape) == 2: # 2 dimension ==> loose pin
            if m in self.loosePinSet.keys():
                self.loosePinSet[m].append(pin_)
            else:
                self.loosePinSet[m] = [pin_]
        elif len(X_.shape) == 1: # vector ==> fix pin
            if m in self.fixPinSet.keys():
                self.fixPinSet[m].append(pin_)
            else:
                self.fixPinSet[m] = [pin_]
        else:
            print("Warning: Dim of pin value is invalid\n")

    def getDiffMat(self, d_):
        if d_ == 0:
            mat_ = np.diag(np.ones(self.num_variables))
        else:
            mat_ = np.diag(np.ones(self.num_variables))
            for j in range(1, d_+1):
                D_ = np.zeros((self.num_variables-j, self.num_variables-j+1))
                # print(D_.shape)
                for i in range(self.num_variables-j):
                    D_[i, i:i+2] = np.array([-1, 1])
                D_ = D_/self.dt
                mat_ = np.dot(D_, mat_)
        # print(mat_)
        return mat_

    def loosePin2InequalityMat(self,):
        ASet = None
        BSet = None

        for pin in self.loosePinSet[0]:
            a_set_ = []
            b_set_ = []
            for dd in range(self.dim):

                n_ = np.min([self.findStepIndex(pin['t']), self.num_variables-pin['d']-1])
                a_ = np.zeros((2, self.num_variables-pin['d']))
                a_[:, n_] = np.array([1, -1])
                a_ = np.dot(a_, self.getDiffMat(pin['d']))
                a_set_.append(a_)
                b_ = np.array([pin['X'][dd, 1], -pin['X'][dd, 0]]).reshape(-1, 1)
                b_set_.append(b_)

            if ASet is None:
                ASet = np.array(a_set_)
                # print('bset {0} in shape {1}'.format(b_set_, np.array(b_set_).shape))
                BSet = np.array(b_set_).reshape(self.dim, -1, 1)
            else:
                ASet = np.concatenate((ASet, np.array(a_set_)), axis=1)
                BSet = np.concatenate((BSet, np.array(b_set_).reshape(self.dim, -1, 1)), axis=1)
            print('Bset final in {}'.format(BSet.shape))
        return ASet, BSet

    def fixPin2InequalityMat(self,):
        AeqSet = None
        BeqSet = None
        for pin in self.fixPinSet[0]:
            aeq_set_ = []
            beq_set_ = []
            for dd in range(self.dim):
                n_ = np.min([self.findStepIndex(pin['t']), self.num_variables-pin['d']-1])
                a_ = np.zeros(self.num_variables-pin['d'])
                a_[n_] = 1.0
                a_ = np.dot(a_, self.getDiffMat(pin['d']))
                aeq_set_.append(a_)
                # print(aeq_set_)
                b_ = pin['X'][dd]
                beq_set_.append(b_)
            if AeqSet is None:
                AeqSet = np.array(aeq_set_).reshape(self.dim, 1, -1)
                BeqSet = np.array(beq_set_).reshape(self.dim, 1, -1)
                # print(AeqSet.shape)
                # print(BeqSet.shape)
            else:
                AeqSet = np.concatenate((AeqSet, np.array(aeq_set_).reshape(self.dim, 1, -1)), axis=1)
                BeqSet = np.concatenate((BeqSet, np.array(beq_set_).reshape(self.dim, 1, -1)), axis=1)
                # print(BeqSet.shape)
        return AeqSet, BeqSet

    def getQPset(self,):
        # 1. objective
        QSet = np.zeros((self.dim, self.num_variables, self.num_variables))
        for dd in range(self.dim):
            Q_ = np.zeros((self.num_variables, self.num_variables))
            for d in range(1, self.weight_mask.shape[0]+1):
                temp_ = self.getDiffMat(d)
                Qd_ = np.dot(temp_.T, temp_)
                Q_ = Q_ + self.weight_mask[d-1]*Qd_
            QSet[dd] = Q_

        # 2. constraints
        ASet, BSet = self.loosePin2InequalityMat()
        AeqSet, BeqSet = self.fixPin2InequalityMat()

        return QSet, ASet, BSet, AeqSet, BeqSet

    def solve(self,):
        self.isSolved = True
        # prepare QP
        QSet, ASet, BSet, AeqSet, BeqSet = self.getQPset()
        print(BeqSet.shape)
        print(BSet.shape)
        for dd in range(self.dim):
            print('solving {}-th dimension.. \n'.format(dd))
            x_sym = ca.SX.sym('x', QSet[0].shape[0])
            opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
            obj = ca.mtimes([x_sym.T, QSet[dd], x_sym])
            a_set = np.concatenate((ASet[dd], AeqSet[dd]))
            Ax_sym = ca.mtimes([a_set, x_sym])
            b_set_u = np.concatenate((BSet[dd], BeqSet[dd]), axis=0) # Ax <= b_set_u
            b_set_l = np.concatenate((-np.inf*np.ones(BSet[dd].shape), BeqSet[dd]), axis=0) # Ax >= b_set_l
            nlp_prob = {'f': obj, 'x': x_sym, 'g':Ax_sym}
            solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)
            try:
                result = solver(lbg=b_set_l, ubg=b_set_u,)
                Phat_ = result['x']
                # print(Phat_)
                flag_ = True
            except:
                Phat_ = None
                flag_ = False
            if flag_:
                self.Xs[dd] = Phat_.full().flatten()
            else:
                self.isSolved = False
                print("Failure ..")

    def eval(self, t_, d_):
        val_ = np.zeros((self.dim, t_.shape[0]))
        for dd in range(self.dim):
            for idx in range(t_.shape[0]):
                t_i = t_[idx]
                if t_i < self.Ts[0] or t_i > self.Ts[-1]:
                    print("WARNING: Eval of t: out of bound. Extrapolation\n")
                Xsd_ = np.dot(self.getDiffMat(d_), self.Xs[dd].T)
                if d_ >0:
                    t_v_ = self.ts[:-d_]
                else:
                    t_v_ = self.ts
                # print(t_v_.shape)
                # print(Xsd_.shape)
                set_interp = interp1d(t_v_, Xsd_, kind='linear')
                # print(t_v_[-1])
                # print(t_[idx])
                if t_[idx] <= t_v_[-1]:
                    val_[dd, idx] = set_interp(t_[idx])
                else:
                    val_[dd, idx] = set_interp(t_v_[-1])
        return val_
