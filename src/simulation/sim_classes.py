#!/usr/bin/env python
"""
Contains the SimulatedVehicle class definition. This class simulates a single vehicle, and provides functions to
acquire measured data or ground truth data.

The class also provides some static methods that might be useful in other situations.

A full simulation should consist of more than one of these objects, that are managed by a central entity, for example
a SimulationCoordinator.
"""
import rospy
import numpy as np
import bob_perception_msgs.msg as bobmsg
from std_msgs.msg import Float64MultiArray, Float64, MultiArrayLayout, MultiArrayDimension
import std_msgs.msg


class SimulatedVehicle:
    """
    Instances of this class represent a single vehicle with all relevant information.
    This class provides methods to manipulate the object data and acquire information about the object, including noisy
    measurement data.
    """
    # The following class variables are the core data that is needed for the TrackedOrientedBox
    object_id = -1
    real_center_x = 0
    real_center_y = 0
    real_angle = 0
    real_length = 0
    real_width = 0
    real_velocity_x = 0
    real_velocity_y = 0

    def __init__(self, oid, x=0.0, y=0.0, angle=0.0, length=0.0, width=0.0, vel_x=0.0, vel_y=0.0):
        """
        Initialize the Simulated Vehicle with its core attribute starting values
        :param oid: The object_id
        :param x: The x position
        :param y: The y position
        :param angle: The angle, less important than x/y and velocity
        :param length: length of the box, can be disregarded in most cases
        :param width: width of the box, can be disregarded in most cases
        :param vel_x: Velocity in x direction
        :param vel_y: Velocity in y direction
        """
        self.object_id = oid
        self.real_center_x = float(x)
        self.real_center_y = float(y)
        self.real_angle = float(angle)
        self.real_length = float(length)
        self.real_width = float(width)
        self.real_velocity_x = float(vel_x)
        self.real_velocity_y = float(vel_y)

    def get_box(self, cov_example_id=0):
        """
        Creates and returns the TrackedOrientedBox for this vehicle
        :param cov_example_id: The id of the example covariance to be used (imported from maven-1.bag)
        :return: Information about this object in the TrackedOrientedBox format
        """
        header = self.create_def_header()
        # ----
        # CURRENTLY THIS USES THE DEFAULT SETTING FOR COV: ONLY CENTER AND VELOCITY
        cov_center = True
        cov_angle = False
        cov_lw = False
        cov_vel = True
        # ----
        oriented_box = bobmsg.OrientedBox(header=header, center_x=self.real_center_x, center_y=self.real_center_y,
                                          angle=self.real_angle, length=self.real_length, width=self.real_width,
                                          velocity_x=self.real_velocity_x, velocity_y=self.real_velocity_y,
                                          covariance=self.get_example_cov(example_id=cov_example_id),
                                          covariance_center=cov_center, covariance_angle=cov_angle,
                                          covariance_length_width=cov_lw, covariance_velocity=cov_vel)

        return bobmsg.TrackedOrientedBox(object_id=self.object_id, box=oriented_box)

    def get_gaussian_box(self, sd_pos=0.5, sd_vel=0.1, sd_angle=0.1, sd_lw=-1, cov_example_id=0):
        """
        Creates and returns a TrackedOrientedBox for this vehicle, that has gaussian noise applied to it.
        The noise can be controlled by the args, it will be centered around the actual values with a standard deviation
        according to the respective sd parameter.
        Passing any parameter as negative will cause no noise to be applied to its respective value.
        :param sd_pos: Standard deviation for the position (both x and y)
        :param sd_vel: Standard deviation for the velocity (in both x and y direction)
        :param sd_angle: Standard deviation for the angle of the vehicle
        :param sd_lw: Standard deviation for the length and width of the box.
        :param cov_example_id: The id of the example covariance to be used (imported from maven-1.bag).
        Currently not used, since diagonal matrices are used for covariance matrices.
        :return: (Gaussian) Noisy information about this object in the TrackedOrientedBox format
        """
        # ----
        # Create a default header for this
        header = self.create_def_header()
        # ----
        # CURRENTLY THIS USES THE DEFAULT SETTING FOR COV: ONLY CENTER AND VELOCITY
        cov_center = True
        cov_angle = False
        cov_lw = False
        cov_vel = True
        # ----
        if sd_pos >= 0:
            noisy_center_x = np.random.normal(loc=self.real_center_x, scale=sd_pos)
            noisy_center_y = np.random.normal(loc=self.real_center_y, scale=sd_pos)
        else:
            noisy_center_x = self.real_center_x
            noisy_center_y = self.real_center_y

        if sd_vel >= 0:
            noisy_velocity_x = np.random.normal(loc=self.real_velocity_x, scale=sd_vel)
            noisy_velocity_y = np.random.normal(loc=self.real_velocity_y, scale=sd_vel)
        else:
            noisy_velocity_x = self.real_velocity_x
            noisy_velocity_y = self.real_velocity_y

        if sd_angle >= 0:
            noisy_angle = np.random.normal(loc=self.real_angle, scale=sd_angle)
        else:
            noisy_angle = self.real_angle

        if sd_lw >= 0:
            noisy_length = np.random.normal(loc=self.real_length, scale=sd_lw)
            noisy_width = np.random.normal(loc=self.real_width, scale=sd_lw)
        else:
            noisy_length = self.real_length
            noisy_width = self.real_width

        # ----
        # The following are options for acquiring a correctly formatted covariance matrix

        # Acquire a cov mat from the example list
        # cov_matrix = self.get_example_cov(example_id=cov_example_id)  # --currently not in use

        # Acquire a cov mat from the sd values, var = sd^2 so we square the given values
        cov_matrix = self.get_diagonal_cov(var_pos=sd_pos**2, var_vel=sd_vel**2)
        # ----
        oriented_box = bobmsg.OrientedBox(header=header, center_x=noisy_center_x, center_y=noisy_center_y,
                                          angle=noisy_angle, length=noisy_length, width=noisy_width,
                                          velocity_x=noisy_velocity_x, velocity_y=noisy_velocity_y,
                                          covariance=cov_matrix,
                                          covariance_center=cov_center, covariance_angle=cov_angle,
                                          covariance_length_width=cov_lw, covariance_velocity=cov_vel)

        return bobmsg.TrackedOrientedBox(object_id=self.object_id, box=oriented_box)

    @staticmethod
    def create_def_header(frame_id="ibeo_front_center"):
        """
        Creates a default header that can be used when creating objects for msgs.
        :param frame_id: The frame_id to be set. Defaults to "ibeo_front_center".
        :return: The default header
        """
        # Header can have seq = 0, so we don't set it
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()  # rospy.init_node() needs to be called before this works
        h.frame_id = frame_id
        return h

    @staticmethod
    def get_example_cov(example_id=0):
        """
        Creates an example covariance matrix. The data in the matrix was acquired from the maven-1.bag file.
        :param example_id: A constant id of the example in the list of stored examples. Minimum value is 0.
        Values above the max value will be set to 0 instead.
        Current maximum value is: 1
        :return: The covariance matrix in format std_msgs/Float64MultiArray
        """
        MAX_SAMPLES = 1
        if example_id > MAX_SAMPLES:
            example_id = 0
        nan = float("NaN")
        examples = []
        # Here, just copy-paste some more data from maven-1.bag so that you can choose which example you want
        examples.append([0.2256878004660412, 0.004743161046803848, nan, nan, nan, 0.4799785723084568, 6.694665570936009e-05, 0.004743161046804125, 0.03333087258142175, nan, nan, nan, 0.0064891656361629295, 0.08962944598353237, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, 0.47997857230845664, 0.006489165636162725, nan, nan, nan, 2.4285886983849885, -0.06696549234373295, 6.694665570941213e-05, 0.08962944598353195, nan, nan, nan, -0.06696549234373322, 1.424665892676366])
        examples.append([0.5758368975539181, -0.10477581457466455, nan, nan, nan, 3.003595362397707, -0.39924730334819275, -0.10477581457466437, 0.24081097327513568, nan, nan, nan, -0.4539729065847597, 1.439606811146181, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, 3.003595362397709, -0.45397290658476147, nan, nan, nan, 18.613535338951223, -1.9899796692884495, -0.39924730334819236, 1.4396068111461806, nan, nan, nan, -1.9899796692884442, 10.681110693507879])

        # -----------------
        cov_list = examples[example_id]
        # Create Float64MultiArray similar to those in the bag files
        dim_0 = MultiArrayDimension(label="", size=7, stride=49)
        dim_1 = MultiArrayDimension(label="", size=7, stride=7)
        cov_dim = [dim_0, dim_1]
        cov_layout = MultiArrayLayout(dim=cov_dim, data_offset=0)
        return Float64MultiArray(layout=cov_layout, data=cov_list)

    @staticmethod
    def vec_to_mat(vector):
        """
        Converts a vector to matrix with 0s everywhere, except for the diagonal which will have the vectors
        value as entries (i.e. converts a vector to a diagonal matrix)
        :param vector: The vector that will be converted to a matrix
        :return: A quadratic matrix of 0s with the vector entries on its diagonal
        """
        size = len(vector)
        mat = np.zeros((size, size))
        for i in range(size):
            mat[i][i] = vector[i]
        return mat

    @staticmethod
    def mirror(mat):
        """
        Takes a matrix and mirrors its along its diagonal, for example
        1 2 3           1 2 3
        0 4 5   --->    2 4 5
        0 0 6           3 5 6
        The diagonal remains unchanged.
        The upper triangular matrix will be mirrored (and remain unchanged!)
        :param mat: The matrix to be mirrored
        :return: The parameter matrix mirrored along its diagonal
        """
        size = len(mat)
        for i in range(size):
            for j in range(i + 1, size):
                mat[j][i] = mat[i][j]
        return mat

    @staticmethod
    def get_diagonal_cov(var_pos=None, var_angle=None, var_lw=None, var_vel=None):
        """
        Creates a random covariance matrix based on variance values.
        The resulting matrix will be a diagonal matrix (i.e. only the variance entries will be used)
        If any parameter is passed as none, its relevant entries in the final matrix will be NaN
        :param var_pos: Variance for the position (x and y) or None if NaN in the final matrix
        :param var_vel: Variance for the velocity (a and y) or None if NaN in the final matrix
        :param var_angle: Variance for the angle or None if NaN in the final matrix
        :param var_lw: Variance for the length and width or None if NaN in the final matrix
        :param max_fac: max_fac for the spread() algorithm
        :param min_fac: min_fac for the spread() algorithm
        :return: A covariance matrix in format std_msgs/Float64MultiArray
        """
        var_vec = []
        if var_pos is not None:
            # Append position variance twice (x+y)
            var_vec.append(var_pos)
            var_vec.append(var_pos)
        if var_angle is not None:
            # Append angle variance once
            var_vec.append(var_angle)
        if var_lw is not None:
            # Append length/width variance twice (x+y)
            var_vec.append(var_pos)
            var_vec.append(var_pos)
        if var_vel is not None:
            # Append velocity variance twice (x+y)
            var_vec.append(var_vel)
            var_vec.append(var_vel)

        # Create a matrix out of this list
        var_mat = SimulatedVehicle.vec_to_mat(var_vec)

        # This matrix is not in the correct format yet, its just a numpy 2D array without the necessary NaNs
        nan = float("NaN")

        # create cov_list, which is a list of covariance entries from the array
        cov_list = [nan for x in range(49)]
        # Start with 49 nan entries, now just need to fill in the values at the correct position

        # PROBLEM/TO DO:
        #   Currently this can only deal with the case of position+velocity covariance
        #   All other cases currently produce an error
        #   However, this is the only case present in the real data, so all testing was limited to it as +well.
        #   A more complex algorithm (than the simple insertion) would be needed to accommodate arbitrary combinations
        if (var_pos is not None) and (var_angle is None) and (var_lw is None) and (var_vel is not None):
            # Default case
            pass  # Here, insert values at their default (correct) positions
            # cov_mat has size 4*4 since we have pos+vel cov
            # cov_list has 49 (7*7) entries, but only the outer 2*2 squares in the corners need to be filled
            # since pos_cov has ids 0 and 1 and vel_cov has ids 5 and 6
            cov_list[0] = var_mat[0][0]
            cov_list[1] = var_mat[0][1]
            cov_list[5] = var_mat[0][2]
            cov_list[6] = var_mat[0][3]

            cov_list[7] = var_mat[1][0]
            cov_list[8] = var_mat[1][1]
            cov_list[12] = var_mat[1][2]
            cov_list[13] = var_mat[1][3]

            cov_list[35] = var_mat[2][0]
            cov_list[36] = var_mat[2][1]
            cov_list[40] = var_mat[2][2]
            cov_list[41] = var_mat[2][3]

            cov_list[42] = var_mat[3][0]
            cov_list[43] = var_mat[3][1]
            cov_list[47] = var_mat[3][2]
            cov_list[48] = var_mat[3][3]
        else:
            print("Unexpected Covariance entries, assuming that only position and velocity have covariance entries")
            raise RuntimeError("Received unexpected covariance entries (expecting only pos.+vel. cov.")

        # Create Float64MultiArray similar to those in the bag files
        dim_0 = MultiArrayDimension(label="", size=7, stride=49)
        dim_1 = MultiArrayDimension(label="", size=7, stride=7)
        cov_dim = [dim_0, dim_1]
        cov_layout = MultiArrayLayout(dim=cov_dim, data_offset=0)
        return Float64MultiArray(layout=cov_layout, data=cov_list)

    def get_sparse_gaussian_measurement(self, stddev_pos=0.15, stddev_vel=0.05):
        """
        Gets a sparse measurement of the vehicle, that has a gaussian error.
        Sparse means that only the position (x,y) and the velocity (vel_x, vel_y) are returned.
        Gaussian noise std. deviation can be controlled for position and velocity (independent of each other)
        :param stddev_pos: Standard deviation for the gaussian noise used for the position
        :param stddev_vel: Standard deviation for the gaussian noise used for the velocity
        :return: noisy x, y, velocity_x, velocity_y
        """
        x = np.random.normal(loc=self.real_center_x, scale=stddev_pos)
        y = np.random.normal(loc=self.real_center_y, scale=stddev_pos)
        vel_x = np.random.normal(loc=self.real_velocity_x, scale=stddev_vel)
        vel_y = np.random.normal(loc=self.real_velocity_y, scale=stddev_vel)
        return x, y, vel_x, vel_y

    def basic_move(self, steps=1):
        """
        Perform one or multiple steps along the current velocity on x and y, without changing anything else about the
        vehicle. One step is equivalent to moving along the x/y axis once according to the x/y velocity
        :param steps: How many steps should be performed.
        :return: x, y coordinates of the new position
        """
        self.real_center_x += self.real_velocity_x * steps
        self.real_center_y += self.real_velocity_y * steps
        return self.real_center_x, self.real_center_y

    def basic_accelerate(self, factor, factor_y=None):
        """
        Increase (or decrease) the velocity via multiplication with a floating points factor.
        The factor is the same for x and y vel. by default, but if the factor_y argument is passed, two different
        values can be used
        :param factor: The multiplicative factor for the velocity
        :param factor_y: If None, the factor parameter will be used for x and y velocity. If a value is passed, this
        value will be used for the y velocity, and the factor parameter will be used for x vel.
        :return:
        """
        self.real_velocity_x *= factor
        if factor_y is None:
            self.real_velocity_y *= factor
        else:
            self.real_velocity_y *= factor_y
