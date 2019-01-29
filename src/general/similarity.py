#!/usr/bin/env python
"""
File that implements functions/classes that are used for comparing two tracked objects wrt similarity.

This is outdated and therefore only remains as a simple example for easy distance methods based on different values.
The t2thistory function implements a
"""
from t2t_utils import *


def dist(p1, p2):
    """
    Takes two points as tuples (x,y) and returns the euclidean distance between the two points.
    :param p1: The first point as a tuple (x,y)
    :param p2: The first point as a tuple (x,y)
    :return: The distance between the two points
    """
    x1, y1 = p1
    x2, y2 = p2
    distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance


class SimilarityChecker:
    """
    This class is outdated and was represented with more elaborate distance estimation functions based on the paper:
        https://hal.archives-ouvertes.fr/hal-00740787/document
    these new functions include an optional track-to-track history and are easier to use.

    This is therefore not used in any real application, but merely to showcase a simple example distance calculation.

    -----------------------

    Class that implements methods for comparing two tracked objects wrt similarity.
    The methods of this class use hyperparameters (for example how to weight change in object angle vs change in pos.).
    Therefore, they cannot be easily implemented as static functions.

    Before using any of the functions, make sure that all relevant hyperparameters for this function are set to the
    desired values.

    All sim_<name> functions follow the same pattern:
    Parameters are old_obj and new_obj and time_diff, return is going to be a float that describes the similarity
    between the two objects that were passed as function of difference, therefore the same objects should return 0, and
    objects that are less similar return higher values.
    old_obj should always be the object that was tracked in the older time step (since for example the velocity of this
    object will be looked at)
    The objects should be in OrientedBox format. time_diff should be an int that describes how many
    time steps they were apart (0=same time step)
    """
    # List of Hyperparameters that will be used across the functions:
    dist_mult = 0.01  # How much velocity is weighted in the position comparison
    velo_add = 1  # A flat value added to all velocity values

    def __init__(self, dist_mult=0.01, velo_add=1):
        self.dist_mult = dist_mult
        self.velo_add = velo_add

    def sim_position(self, old_obj, new_obj, time_diff=0):
        """
        Similarity that only takes into account the position and the time passed, but not the velocity etc of the
        objects.
        This is according to the following formula:
        if distance_between_points <= time_passed * multiplicative_factor + additive_factor:
            return 0
        else:
            return (distance_between_points - time_passed * multiplicative_factor + additive_factor)
        :param old_obj: The first object in OrientedBox format
        :param new_obj: The second object in OrientedBox format
        :param time_diff: The numerical difference in time steps between the two objects, 0=same time step
        :return: A Float describing the similarity between the two objects as a function of difference (0 for similar)
        """
        phi = self.dist_mult
        gamma = self.velo_add

        max_dist = time_diff * phi + gamma

        # Calculate the euclidean distance between the two points
        p1 = (old_obj.center_x, old_obj.center_y)
        p2 = (new_obj.center_x, new_obj.center_y)
        pt_dist = dist(p1, p2)

        if pt_dist <= max_dist:
            return 0
        else:
            return pt_dist - max_dist

    def sim_velocity(self, old_obj, new_obj, time_diff=0):
        """
        Since the velocity in the data sets was barely used, this function is only not currently used anywhere.

        Compares two objects in OrientedBox format based on the positions. This takes into account velocity based on
        the following hyperparameter:
        - dist_mult: Multiplicative with velocity and time_diff to describe change in position

        This methods is basically already implementing a threshold:
        It draws a circle of size dist_mult*time_diff*velocity around the old_obj and returns 0 if new_obj is inside
        this circle (therefore 0 is a valid threshold for this functions similarity value). If new_obj is outside the
        circle, the distance to the circle is returned.
        :param old_obj: The first object in OrientedBox format
        :param new_obj: The second object in OrientedBox format
        :param time_diff: The numerical difference in time steps between the two objects, 0=same time step
        :return: A Float describing the similarity between the two objects as a function of difference (0 for similar)
        """
        phi = self.dist_mult
        gamma = self.velo_add
        # Calculate "full" velocity using pythagoras (since you only have x+y given)
        vel_x = np.abs(old_obj.velocity_x) + gamma
        vel_y = np.abs(old_obj.velocity_y) + gamma
        vel_full = np.sqrt(vel_x**2 + vel_y**2)
        # Calculate the euclidean distance between the two points
        p1 = (old_obj.center_x, old_obj.center_y)
        p2 = (new_obj.center_x, new_obj.center_y)
        pt_dist = dist(p1, p2)

        # Establish the "circle" size, i.e. the maximum distance that the two points can be apart to return a similarity
        # value of 0
        max_dist = time_diff * phi * vel_full

        # For debugging purposes, you can use the following code to document similarity values
        # For example if you need to find a threshold etc

        # FILE = "src/T2TF_SST/data/test.txt"  # name of the file to print similarity values to
        if pt_dist <= max_dist:
            # The two objects were in acceptable distance to each other (based on velocity)
            # with open(FILE, "a") as myfile:
            #    myfile.write("0"+"\n")
            return 0
        else:
            # The two objects were too far from each other, return the distance from the second one to the circle around
            # the first one
            # with open(FILE, "a") as myfile:
            #    myfile.write(str(pt_dist - max_dist)+"\n")
            return pt_dist - max_dist
