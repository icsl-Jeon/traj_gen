#!/usr/bin/env python
# coding=utf-8

# from .traj_gen_base import TrajGen
import numpy as np
import casadi as ca

class PolyTrajAgg():
    def __init__(self, N_, dim_, degree_):
        # super().__init__(knots_, dim_)
        """
        Args
            N_(int): number of coefficients of the underlying polynomial, muss be even
            dim_(int): dimension of the problem
        """
        self.N = N_
        self.dim = dim_
        self.derivate_degree = degree_ # 4 for snap
        self.vertices = None # fix points
        self.segment_times = None # time segmentation
        self.kHighesDerivateToOptimize = int(self.N/2) -1
        self.base_coefficients = self.computeBaseCoefficients()
        # print(self.base_coefficients[2])
        self.cost_matrices = None #

    def computeBaseCoefficients(self, N_=22):
        """
        This is to pre-calcuate the coefficients of the derivaty
        """
        base_coeff_ = np.zeros((N_, N_))
        base_coeff_[0] = np.ones(base_coeff_[0].shape)
        deg_ = N_ - 1
        order_ = N_ - 1
        for i in range(1, N_):
            for j in range(deg_-order_, N_):
                base_coeff_[i, j] = (order_-deg_+j)*base_coeff_[i-1, j]
            order_ = order_ - 1
        return base_coeff_

    def estimateSegmentTimesNfabian(self, vertices_, v_max_, a_max_, magic_fabian_const=6.5):
        """
        Args
        vertices_(np.array): pins
        v_max_(double): max velocity
        a_max_(double): max accleration

        """
        segment_times = []
        for i in range(vertices_.shape[0]-1):
            start_ = vertices_[i]
            end_ = vertices_[i+1]
            distance_ = np.linalg.norm(end_-start_)
            t_ = distance_/v_max_ * 2 *(1.+magic_fabian_const*v_max_/a_max_*np.exp(-distance_/v_max_*2))
            segment_times.append(t_)
        return np.array(segment_times)

    def computeQuadraticCostJacobian(self, d_, t_):
        cost_jacobian_ = np.zeros((self.N, self.N))
        for i in range(self.N- d_):
            for j in range(self.N-d_):
                exponent_ = (self.N-1-d_)*2.+1-i-j
                cost_jacobian_[self.N-1-i, self.N-1-j] = self.base_coefficients[d_, self.N-1-i]*self.base_coefficients[d_, self.N-1-j]*t_**exponent_*2.0/exponent_
        return cost_jacobian_

    def updateSegmentTimes(self, times_):
        # n_segment_times = times_.shape[0]
        for i in range(self.num_segments):
            cost_matrix_ = self.computeQuadraticCostJacobian(self.derivate_degree, times_[i])
            if self.cost_matrices is None:
                self.cost_matrices = np.expand_dims(cost_matrix_, axis=0)
            else:
                self.cost_matrices = np.concatenate((self.cost_matrices, np.expand_dims(cost_matrix_, axis=0)
))
                # self.cost_matrices =



    def setupFromVertices(self, vertices_, times_, derivative_to_optimize_):
        """
        Args
            vertices(np.array): fix pins
            times(np.array): time steps
            derivative_to_optimize(int): derivate degree
        """

        self.derivate_degree = derivative_to_optimize_
        self.vertices = vertices_
        self.segment_times = times_
        self.num_vertices = vertices_.shape[0]
        self.num_segments = self.num_vertices - 1

        # for i in range(self.num_vertices):
            # temp_vertex_ = vertices_[i]

        self.updateSegmentTimes(times_)
        # self.setupConstraintReorderingMatrix()



if __name__ == '__main__':
    obj = PolyTrajAgg(dim_=3, N_=10, degree_=4)
    points_ = np.array([[0, 0, 1],[1, 2, 3], [2,1, 5], [4, 5, 1.2]])
    time_segs = obj.estimateSegmentTimesNfabian(points_, v_max_=2, a_max_=2)
    print(time_segs)
    obj.setupFromVertices(points_, time_segs, 4)

