#!/usr/bin/env python
"""
Implementation of the Covariance Intersection algorithm.

Includes functions for approximating omega based on covariance matrices.

All functions are currently only implemented for the 2-sensor case, but the algorithm can be generalized to more
data sources.
"""
from numpy.linalg import inv as inverse
from numpy.linalg import det as det
from numpy import dot
import numpy as np


def dual_fast_omega(P_i, P_j):
    """
    Implementation of the approximation of omega when using the fast covariance intersection algorithm for two tracks.

    Follows the formula given on slide 24 of: http://www.ftm.mw.tum.de/uploads/media/17_Seeliger.pdf
    :param P_i: The covariance matrix of track i as a 2D numpy-array
    :param P_j: The covariance matrix of track j as a 2D numpy-array
    :return: omega, the weighting of the track i (first parameter). j should be weighted (1-omega)
    """
    # Code using +/-/*
    #   w = det(P_j) / (det(P_i) + det(P_j))
    # print("Determinants of the matrices: "+str(det(P_j))+"\t"+str(det(P_i)))  # DEBUG print determinants used
    w = np.divide(det(P_j), (np.add(det(P_i), det(P_j))))
    return w


def dual_improved_omega(P_i, P_j):
    """
    Implementation of the approximation of omega when using the improved fast covariance intersection algorithm for
    two tracks.

    Follows the formula given on slide 24 of: http://www.ftm.mw.tum.de/uploads/media/17_Seeliger.pdf
    :param P_i: The covariance matrix of track i as a 2D numpy-array
    :param P_j: The covariance matrix of track j as a 2D numpy-array
    :return: omega, the weighting of the track i (first parameter). j should be weighted (1-omega)
    """
    # Calculate the inverse of both matrices first instead of doing this multiple times
    P_i_inv = inverse(P_i)
    P_j_inv = inverse(P_j)

    # Now, calculate w using these two matrices
    # Splitting this into two steps (the entire thing is a big division)
    # First step is calculate everything before the division
    # Second step is calculate everything after the division
    # The following lines describe the operations with +/-/* , the actual code uses numpy functions:
    #    w = det(P_i_inv + P_j_inv) - det(P_j_inv) + det(P_i_inv)
    #    w /= 2 * det(P_i_inv + P_j_inv)

    w = np.add(np.subtract(det(np.add(P_i_inv, P_j_inv)), det(P_j_inv)), det(P_i_inv))
    w = np.divide(w, dot(2, det(np.add(P_i_inv, P_j_inv))))

    return w


def dual_cov_intersection(P_i, P_j, x_i, x_j, omega_fct=dual_fast_omega):
    """
    Implementation of the covariance intersection algorithm for two tracks, according to the formula given on slide 24
    of: http://www.ftm.mw.tum.de/uploads/media/17_Seeliger.pdf

    One parameter is a function to determine w/omega:
        In the basic CI algorithm, this is a convex optimization problem
        In the FCI/I-FCI algorithms, an approximate solution is calculated
    This omega_function should take two arguments: P_i and P_j (in that order). These parameters are the two covariance
    matrices of the tracks.

    This implementation follows the following naming:
    The two tracks are called i and j
    P is the final covariance matrix, P_i is the cov-matrix of track i
    x is the estimated result, x_i the tracking of track i
    w is the omega in the CI algorithm, i.e. the weighting of the two tracks based on their covariances.
        track i will be weighted w, while j will be weighted (1-w)
    :param P_i: The covariance matrix of track i as a 2D numpy-array
    :param P_j: The covariance matrix of track j as a 2D numpy-array
    :param x_i: The tracking estimation of track i as a 2D numpy-array
    :param x_j: The tracking estimation of track j as a 2D numpy-array
    :param omega_fct: Function that takes P_i and P_j as arguments and calculates w using this
    :return: (x, P, w) [estimation, covariance matrix estimation, omega used)
    """
    # All code is provided twice:
    #   Commented and spaced is the readable version of the algorithm
    # The actual code is the same, but using numpy operations instead of +/-/*
    w = omega_fct(P_i, P_j)
    # print("using w = "+str(w))
    # Calculate P and P^(-1)=P_inv first
    #   P_inv = w * inverse(P_i) + (1-w)*inverse(P_j)
    P_inv = np.add(dot(w, inverse(P_i)), dot((1 - w), inverse(P_j)))
    P = inverse(P_inv)
    # Now, proceed by calculating x
    #   x = P * (w * inverse(P_i) * x_i + (1-w)*inverse(P_j)*x_j)
    x = dot(P, np.add(dot(dot(w, inverse(P_i)),  x_i), dot(dot((1 - w), inverse(P_j)),  x_j)))
    return x, P, w

