#!/usr/bin/env python
"""
Contains:
    TrackingHistory class that stores all historic tracks for an object identified by a tuple (sensor_id, object_id).
    t2tdistance functions that include history in their calculations.

All this needs to be used by the t2ta_historic function that is implemented in t2ta_algorithms.py.

Additionally, some helper functions related to this are implemented in this file.

The important function in this file that should be used by association algorithms is "t2t_distance_historic", which
is the equivalent of the distance "D" in the paper https://hal.archives-ouvertes.fr/hal-00740787/document.
The single-time distance "d" from that paper is implemented in  the function "t2t_distance_box".
"""
import rospy
import numpy
from t2ta_algorithms import *
from bob_perception_msgs.msg import *
import sys
import time


class TrackingHistory:
    """
    TrackingHistory class that stores all historic tracks for a tuple (sensor_id, object_id).
    These can be accessed based on time stamp using get_timed, which is the recommended way.
    """
    sensor_id_list = []  # List of strings of sensor ids

    # Data is a 3D List in the following format:
    # data is a list of the sensors, for example the first entry is the entry for the first entry in the sensor ids list
    # every entry in this list is then a list of all time steps where data from this sensor was received
    #   these vary between sensors
    # every entry in this list of observations is an array of TrackedOrientedBoxes that represent the sensors
    # measurement at this time step.
    # These are not sorted in any way, they simply represent what the sensor returned
    # This means it can be accessed in the following way:
    # data[sensor_id][timestep][no_track] where:
    #   sensor_id : The numerical id of the sensor (= the position of the nam in sensor_id_list)
    #   timestep : -1 to acquire the most recent one, 0 to acquire the first one, etc
    #   no_track : Which track of the measurement should be looked at
    # Keep in mind that no_track is not equal to object id, usually you will be iterating over all tracks like this:
    #   for track in data[sensor_id][timestep]:
    #       if track.object_id==wanted_object_id:
    #           # use track.box here ...
    # or something similar
    data = []

    def __init__(self):
        sensor_id_list = []  # List of strings of sensor ids
        data = []

    def new_sensor(self, sensor_name):
        """
        Appends a new sensor to the list of known sensors, and extends the data by a new empty array where measurements
        for this sensor will be stored
        :param sensor_name: The unique name of the sensor
        :return: True, if the sensor was appended, False if it already existed
        """
        if sensor_name in self.sensor_id_list:
            # name is already in the list (i.e. already in use)
            # don't add anything, instead just return False
            return False
        self.sensor_id_list.append(sensor_name)
        self.data.append([])
        return True

    def is_known(self, sensor_name):
        """
        Checks if a given sensor is known in the tracking history, without adding anything to the data structure if the
        given name is not known
        :param sensor_name: The unique name of the sensor
        :return: True, if the sensor is already known, else false
        """
        return sensor_name in self.sensor_id_list

    def tracking_steps(self, sensor_name):
        """
        Returns how many time steps were tracked for the given sensor, or -1 if the sensor is not known.
        This is equivalent to the length of the array that stores all measurements for this sensor
        :param sensor_name: The unique name of the sensor
        :return: How many time steps were tracked for the given sensor, or -1 if the sensor is not known.
        """
        if not self.is_known(sensor_name):
            return -1
        return len(self.data[self.sensor_id_list.index(sensor_name)])

    def add(self, sensor_name, measurement):
        """
        Adds a new measurement to the tracking history, based on the sensor name.
        If the sensor is not yet known, it will be added to the list of sensors
        :param sensor_name: The unique name of the sensor that produced this measurement
        :param measurement: An array of TrackedOrientedBoxes that represent the most recent measurement of the sensor.
        If this is empty, nothing will be added
        :return: True, if the measurement was successfully added, else false.
        """
        if len(measurement) == 0:
            return False
        if not self.is_known(sensor_name):
            self.new_sensor(sensor_name)
        sensor_id = self.sensor_id_list.index(sensor_name)  # The numerical id of the sensor
        # Append the measurement to the list of tracks for this sensor_id
        self.data[sensor_id].append(measurement)
        return True

    def get_absolute(self, object_id, sensor_name, time=-1, hist_size=1):
        """
        As outlined below, this function can only be used under certain assumption that real data will not fulfill.
        Therefore it is recommended to use get_timed instead

        Acquire tracks for the set of identification parameters.
        A hist_size of 1 means only a single track will be returned.

        This is based on absolute timesteps, that means an array with a length equivalent to hist_size will be returned.
        Due to this, sensors with different frequencies may cause issues when using this. For example: Sensor A sends 5x
        as many measurements as Sensor B. Giving a hist_size of 5 will now return 5 measurements of sensor A that all
        fit to the most recent measurement of Sensor B, but also 4 more measurements for B that don't have any match in
        the list of measurements from sensor A.
        :param object_id: The object_id of the vehicle to be analyzed
        :param sensor_name: The name of the sensor that tracked the vehicle to be analyzed
        :param time: The time for which the data should be returned. A time of -1 means that the most recently added
        tracks will be used. If a value that is greater than the number of measurements for this sensor is given, then
        it defaults to -1. -1 therefore is equivalent to giving (tracking_steps(sensor_name)-1)
        :param hist_size: How far into the past the tracks should go. A value of 1 means a single track will be returned
        (the one for time step time). If hist_size is too big, the maximum possible value will be used instead.
        :return: A list of TrackedOrientedBoxes for this set of identification parameters. The first object in this list
        will be the most recent one, and the last object in the list will be the one that was measured first (respecting
        the history size and the given time index)
        :raises: ValueError if the sensor name is not known
        """
        # --- First, go over parameters and correct them as necessary

        if not self.is_known(sensor_name):
            raise ValueError("Attempting to acquire data for unknown sensor \""+sensor_name+"\"!")
        sensor_id = self.sensor_id_list.index(sensor_name)  # The numerical id of the sensor
        if self.tracking_steps(sensor_name) <= time:
            # time is too big, set it to -1 to use the last track instead
            time = -1
        # Check hist_size. If a timepoint is specified, max_hist_size is one bigger than it (since time is 0-indexed and
        # history_size is 1-indexed). If -1 is specified, reset time to the maximum possible value now (which is equiv.)
        if time == -1:
            time = self.tracking_steps(sensor_name) - 1
        max_hist_size = time+1
        if hist_size > max_hist_size:  # If hist_size exceeds the upper limit, set it to this upper limit
            hist_size = max_hist_size

        # --- Finished preparing all parameters, now extract all tracks for this:

        historic_tracks = []  # The list of tracks including the history that will be returned
        sensor_tracks = self.data[sensor_id]  # All tracks of this sensor

        for i in range(hist_size):
            # go over all positions in the tracking array that fit the parameters
            measurement = sensor_tracks[time-i]  # Acquire the measurement for this time step
            # The measurement contains the oriented boxes for all objects that were tracked by the sensor at that time
            for oriented_box in measurement:  # Go over all objects in the measurement
                if oriented_box.object_id == object_id:  # If the object has a matching id, add it to the historic track
                    historic_tracks.append(oriented_box)
                    break  # Only add one box per measurement in case multiple boxes with the same id are contained

        return historic_tracks

    def get_timed(self, object_id, sensor_name, time=-1, hist_size=rospy.Duration(0)):
        """
        Acquire tracks for the set of identification parameters.
        A hist_size of rospy.Duration(0) means only a single track will be returned.

        This is not based on a number of time steps, but instead returns a dynamic number of time steps depending on
        how far behind the base time step (which is the one matching the time parameter) they are. For example,
        giving time=-1 and hist_size=rospy.Duration(1) will return all measurements that have a time stamp of at most
        1 second before the time stamp of the newest measurement.

        The data resulting from this will likely require further processing, since acquiring data for the same
        parameters but different sensor_names will result in arrays of different length, which are therefore not
        directly compatible with the t2t_distance function.
        :param object_id: The object_id of the vehicle to be analyzed
        :param sensor_name: The name of the sensor that tracked the vehicle to be analyzed
        :param time: The time for which the data should be returned. A time of -1 means that the most recently added
        tracks will be used. If a value that is greater than the number of measurements for this sensor is given, then
        it defaults to -1. -1 therefore is equivalent to giving (tracking_steps(sensor_name)-1)
        :param hist_size: A rospy.Duration that specifies how far into the past this should go. If time is Duration(0)
        (the default value), then only the measurement at time step "time" will be returned (still in array though!)
        :return: A list of TrackedOrientedBoxes for this set of identification parameters. The first object in this list
        will be the most recent one, and the last object in the list will be the one that was measured first (respecting
        the history size and the given time index). The size of this is determined by hist_size, so that the first and
        last object are at most hist_size (which is a rospy.Duration object) time apart.
        """
        # --- First, go over parameters and correct them as necessary

        if not self.is_known(sensor_name):
            raise ValueError("Attempting to acquire data for unknown sensor \"" + sensor_name + "\"!")
        sensor_id = self.sensor_id_list.index(sensor_name)  # The numerical id of the sensor
        if self.tracking_steps(sensor_name) <= time:
            # time is too big, set it to -1 to use the last track instead
            time = -1
        # If -1 is specified for time, reset time to the maximum possible value now (which is equiv.)
        if time == -1:
            time = self.tracking_steps(sensor_name) - 1

        # --- Finished preparing all parameters, now extract all tracks for this:

        sensor_tracks = self.data[sensor_id]  # All tracks of this sensor

        historic_tracks = []  # The list of tracks including the history that will be returned
        # Acquire a list of all tracks

        # Acquire the base time, which is the timing of the measurement performed at time step "time"
        # Can just take the first entry of the measurement, because the timing across a single measurement should be
        # the same for all objects, regardless of their id
        base_time = sensor_tracks[time][0].box.header.stamp

        keep_going = True  # Flag that determines if the data is still fitting the time constraint.
        for i in range(time, 0, -1):
            #  i decreases every iteration
            # add the measurement for this iteration to the batch (if it contained a time and object id matching track)
            measurement = sensor_tracks[i]
            # The measurement contains the oriented boxes for all objects that were tracked by the sensor at that time
            for oriented_box in measurement:  # Go over all objects in the measurement
                if oriented_box.object_id == object_id:  # If the object has a matching id, add it to the historic track
                    # but first, check its time stamp
                    if base_time - oriented_box.box.header.stamp <= hist_size:
                        historic_tracks.append(oriented_box)
                    else:
                        keep_going = False
                    break  # Only add one box per measurement in case multiple boxes with the same id are contained
            # Looped over all oriented_boxes in the measurement checking their obj_id, now check if the time constraint
            # was still fulfilled after the last box was appended
            if not keep_going:
                break

        return historic_tracks

    def get_timestep(self, object_id, sensor_name, time, hist_size=rospy.Duration(0)):
        """
        Analogue to get_timed, but using the time parameter to determine the starting position based on a Timestamp
        :param object_id: The object_id of the vehicle to be analyzed
        :param sensor_name: The name of the sensor that tracked the vehicle to be analyzed
        :param time: A timestamp that will be used to acquire a numerical time step that will then be used to acquire
        the data
        :param hist_size: A rospy.Duration that specifies how far into the past this should go. If time is Duration(0)
        (the default value), then only the measurement at time step "time" will be returned (still in array though!)
        :return: A list of TrackedOrientedBoxes for this set of identification parameters. The first object in this list
        will be the most recent one, and the last object in the list will be the one that was measured first (respecting
        the history size and the given time index). The size of this is determined by hist_size, so that the first and
        last object are at most hist_size (which is a rospy.Duration object) time apart.
        """
        # the way this works is as follows:
        #   find a numerical time step in the data that matches the stamp most closely
        #   (this is the index of the data with the timestep that matches the given stamp most closely)
        #   acquire this sensors relative data using the self.get_timed function
        # return this object
        numerical_time = 0  # The value that this is looking for, init with 0

        time_step = -1  # Init value for time step counter
        sensor_data = self.data[self.sensor_id_list.index(sensor_name)]

        min_stamp = rospy.Duration(99999999, 0)  # init value is a very long duration, so that the first comp is smaller

        for time_track in sensor_data:  # iterate over all timings
            time_step += 1  # Increase the time step counter
            for datapoint in time_track:
                if datapoint.object_id == object_id:
                    try:
                        diff = datapoint.box.header.stamp - time
                    except AttributeError as e:
                        raise e  # Could also handle the error if you want to be robust

                    diff.secs = abs(diff.secs)
                    diff.nsecs = abs(diff.nsecs)
                    if diff < min_stamp:
                        min_stamp = diff
                        numerical_time = time_step

        # Use this to acquire the data in the usual way and return that
        result = self.get_timed(object_id, sensor_name, numerical_time, hist_size)
        return result


def closest_match(data, stamp):
    """
    Selects the closest match wrt time from the list data compared to the timestamp stamp.
    Can work with Lists of TrackedOrientedBox objects and OrientedBox Objects.
    :param data: A list of objects with a header that have a time stamp
    :param stamp: Timestamp to compare to
    :return: The object from the list with the closest matching time stamp
    """
    if data is None or stamp is None or len(data) == 0:
        return None  # return None if no data was given
    min_val = rospy.Duration(99999999, 0)  # init value is a very long duration, so that the first comp is smaller
    min_pos = 0  # position of the minimum
    for i in range(len(data)):  # use a counter var to store min position
        datapoint = data[i]  # select the current data point from the list
        try:
            diff = datapoint.header.stamp - stamp
        except AttributeError:
            # might have gotten TrackedOrientedBox objects instead of OrientedBox Objects, try again if that's the case
            try:
                diff = datapoint.box.header.stamp - stamp
            except AttributeError as e:
                raise e

        diff.secs = abs(diff.secs)
        diff.nsecs = abs(diff.nsecs)
        if diff < min_val:
            min_val = diff
            min_pos = i
    # print("selected"+str(min_pos)+"/"+str(len(data))+" data with diff: "+
    #      str(min_val.secs)+" nsecs:"+str(min_val.nsecs))
    return data[min_pos]


def sec_diff(a, b):
    """
    Takes two timestamps a and b and returns the difference in time between the two as a floating point number for easy
    comparison. Most notably, this returns absolute values, i.e positive values instead of negative values if b is
    before a.
    :param a: rospy.Time object one
    :param b: rospy.Time object two
    :return: Absolute value of the diff between the two times as a float
    """
    d = b - a
    d.secs = abs(d.secs)
    d.nsecs = abs(d.nsecs)
    return d.to_sec()


def common_history_sorted(tracks):
    """
    Takes a list of tracks, which in turn are lists of TrackedOrientedBoxes. These lists are assumed to be generated by
    TrackingHistory.get_timed, and therefore have different sizes. This function converts all these into a list of best
    fitting tracks, with a maximum size (i.e. the size of the smallest track).
    For this, the smallest track is iterated, and for each time step the closest match w.r.t. its time step from all
    other tracks is selected.

    This version assumes that the tracks are already sorted. (!)
    :param tracks: A list of tracks, which are lists of TrackedOrientedBoxes. Can be produced by
        TrackingHistory.get_timed. These explicitly don't need to have the same length, but need to be sorted.
    :return: A list of tracks, which are lists of TrackedOrientedBoxes. These tracks will have the same length, and
    their content will be chosen based on time stamps. The new length will be the maximum possible length, which is the
    length of the smallest track
    """
    if not len(tracks) == 2:
        raise ValueError("Sorted common history requires exactly two tracks")

    # smallest/largest_track store the id of the respective track in tracks, which can only be 0 or 1 (since len==2)
    if len(tracks[0]) < len(tracks[1]):
        smallest_track = 0
        largest_track = 1
    else:
        smallest_track = 1
        largest_track = 0

    final_length = tracks[smallest_track]
    # Found smallest track and its length, proceed by iterating over this track
    adjusted_tracks = [[] for w in tracks]  # The new list of tracks after they were adjusted to fit to each others time

    # Algorithm now assumes that the tracks are already sorted before they are passed to this function.
    # Algorithm is basically as follows:
    # Counter s counts the position in the smallest track, beginning with 0
    # Counter l counts the position in the larger track, beginning with 0
    # LOOP:
    #   Add next entry (s) of smaller track to the final list
    #   Look for the next best entry in the larger track, by moving the bookmark l forward (without resetting it)
    #   As soon as the last entry of the larger track is reached, or increasing l would reduce the match quality: ADD IT

    l = 0  # Begin by bookmarking l
    for s in range(len(tracks[smallest_track])):  # iterate s over entire length of smallest track
        # Add next entry of smaller track
        adjusted_tracks[smallest_track].append(tracks[smallest_track][s])
        # now, find the next matching entry in the larget track
        while True:  # Breaking from inside once the condition is reached
            # If l is already at the last position, add it and then break
            if l == len(tracks[largest_track])-1:
                break
            # Else we can compare the current one and the next one
            stamp_s = tracks[smallest_track][s].box.header.stamp
            stamp_l = tracks[largest_track][l].box.header.stamp
            stamp_next = tracks[largest_track][l+1].box.header.stamp
            if sec_diff(stamp_s, stamp_l) < sec_diff(stamp_s, stamp_next):
                # "if the current l fits better than the next one, then break and don't look any further"
                break
            l += 1  # Check the next l
        # break was reached in the while, add the next element to the list of objects
        adjusted_tracks[largest_track].append(tracks[largest_track][l])

    return adjusted_tracks


def acq_timestamp(box):
    """
    Returns the timestamp from the TrackedOrientedBox in seconds
    Can be used as a key fct for the sorted inbuilt function.
    :param box: TrackedOrientedBox
    :return: box.box.header.stamp.to_sec()
    """
    return box.box.header.stamp.to_sec()


def t2t_distance_historic(track_a, track_b, state_space=(True, False, False, True), use_identity=False):
    """
    Calculates the distance between two tracks, based on their history.
    The tracks are passed as arrays of TrackedOrientedBoxes, and should (obviously) be ordered in the same way (i.e.
    both have the newest tracking as first/last etc)

    The tracks don't NEED to have the same length, if they don't have the same length the common_history function will
    be used to reduce the size of the longer and create two tracks that fit to each other wrt time.

    This is according to https://hal.archives-ouvertes.fr/hal-00740787/document, where this function is called D
    :param track_a: The first track in the form of an array of TrackedOrientedBoxes, where every box represents the
    object at a single time step.
    :param track_b: The second track in the form of an array of TrackedOrientedBoxes, where every box represents the
    object at a single time step.
    :param state_space: A vector describing which parameters should be used as the state space. Defaults to position+
    velocity. (Position - angle - length/width - velocity) as booleans
    :param use_identity: Boolean flag to determine whether to use the identity matrix for covariance matrices. Even if
    set to false, unless all True entries in the state space parameter have matching covariance entries in both tracks,
    the identity will be used
    :return: A float representing the distance between the two tracks.
    """
    timed = time.clock()  # DEBUG

    if len(track_a) != len(track_b):
        # build a common history for track_a and track_b if the tracks do not match in size

        # Using common_history_sorted for _significantly_ better performance, but this requires sorted tracks
        track_a = sorted(track_a, key=acq_timestamp)
        track_b = sorted(track_b, key=acq_timestamp)
        common_tracks = common_history_sorted([track_a, track_b])
        track_a = common_tracks[0]
        track_b = common_tracks[1]

    timed_common = time.clock()  # DEBUG

    distance = 0
    for i in range(len(track_a)):
        distance += t2t_distance_box(box_a=track_a[i], box_b=track_b[i], state_space=state_space,
                                     use_identity=use_identity)

    # Perform averaging (i.e. the multiplication with 1/n in the formula in the paper)
    distance /= len(track_a)

    return distance


def t2t_distance_box(box_a, box_b, state_space=(True, False, False, True), use_identity=False):
    """
    Calculates the distance between two tracks at a single time step. This should be used in conjunction with the
    t2t_distance_historic function, which calls this function for every single time step.

    The tracks are passed as TrackedOrientedBoxes.

    This is according to https://hal.archives-ouvertes.fr/hal-00740787/document, where this function is called d
    :param box_a: The first track in the form of a TrackedOrientedBox.
    :param box_b: The second track in the form of a TrackedOrientedBox.
    :param state_space: A vector describing which parameters should be used as the state space. Defaults to position+
    velocity. (Position - angle - length/width - velocity) as booleans
    :param use_identity: Boolean flag to determine whether to use the identity matrix for covariance matrices. Even if
    set to false, unless all True entries in the state space parameter have matching covariance entries in both tracks,
    the identity will be used
    :return: A float representing the distance between the two tracks
    """
    # ---
    # NOTATION
    # X_a is the state estimate for track a
    # X_b is the state estimate for track b
    # P_a is the covariance matrix for track a
    # P_b is the covariance matrix for track b
    # dim is the dimension of the state space, covariance matrices are of dimension "dim x dim"
    # ---

    # Calculate the dimension of all the matrices that will be used
    dim = 0
    if state_space[0]:
        dim += 2
    if state_space[1]:
        dim += 1
    if state_space[2]:
        dim += 2
    if state_space[3]:
        dim += 2

    # Check for covariance data. If no covariance data is given for either track, or the covariance does not
    # match the required state space, then both covariance matrices will be set to the identity matrix
    # For this, check that for every all "True" entries in the state space vector, both tracks have a covariance entry

    if state_space[0]:
        if (not box_a.box.covariance_center) or (not box_b.box.covariance_center):
            use_identity = True
    if state_space[1]:
        if (not box_a.box.covariance_angle) or (not box_b.box.covariance_angle):
            use_identity = True
    if state_space[2]:
        if (not box_a.box.covariance_length_width) or (not box_b.box.covariance_length_width):
            use_identity = True
    if state_space[3]:
        if (not box_a.box.covariance_velocity) or (not box_b.box.covariance_velocity):
            use_identity = True

    # Now, determine the covariance matrix
    if use_identity:
        # use identity for covariance matrices
        P_a = np.eye(dim, dim)
        P_b = np.eye(dim, dim)
    else:
        # extract covariance matrices from the tracks
        P_a = acquire_covariance(box_a.box)
        P_b = acquire_covariance(box_b.box)

    # Now, determine the state estimates as given by the matrix
    X_a = []
    X_b = []

    if state_space[0]:  # Append the position to the state estimate
        X_a.append(box_a.box.center_x)
        X_a.append(box_a.box.center_y)
        X_b.append(box_b.box.center_x)
        X_b.append(box_b.box.center_y)
    if state_space[1]:  # Append the angle to the state estimate
        X_a.append(box_a.box.angle)
        X_b.append(box_b.box.angle)
    if state_space[2]:  # Append length and width of the box to the state estimate
        X_a.append(box_a.box.length)
        X_a.append(box_a.box.width)
        X_b.append(box_b.box.length)
        X_b.append(box_b.box.width)
    if state_space[3]:  # Append the velocity to the state estimate
        X_a.append(box_a.box.velocity_x)
        X_a.append(box_a.box.velocity_y)
        X_b.append(box_b.box.velocity_x)
        X_b.append(box_b.box.velocity_y)
    # Convert these array to numpy arrays for further calcs
    X_a = np.array(X_a)
    X_b = np.array(X_b)
    # Successfully built a state estimate for both tracks with the correct dimension

    # Now, calculate the distance between the two tracks based on that according to the formula from the paper
    # this is split up to prevent stacking too many numpy operations
    part_1 = np.subtract(X_a, X_b)
    part_3 = np.copy(part_1)
    part_1 = np.transpose(part_1)

    part_2 = np.add(P_a, P_b)
    part_2 = np.linalg.inv(part_2)

    part_4 = np.log(np.linalg.det(np.add(P_a, P_b)))

    distance = np.dot(part_1, part_2)
    distance = np.dot(distance, part_3)
    distance = np.add(distance, part_4)

    return distance


def acquire_covariance(box):
    """
    Takes an OrientedBox as a parameter and returns a numpy covariance matrix, depending on the entries.
    :param box: OrientedBox that contains the covariance information
    :return: numpy array representing the cov. matrix stored in the parameter
    """
    # Summary of how nans are removed:
    # 1.Create a stub of 0s for the final cov mat
    #   Get a 7x7 array with all values incl. nan
    #   Create a logical indexing array for all non-nan entries
    # 2.Iterate over the logical array (2loops: x over y):
    #   If true, add this value to the cov mat
    #            increase the "x" counter
    #            mark that a value was found in this iteration on this dimension
    #   After every y run: reset the x counter
    #                      if any value was found in the previous iteration, increase the y counter by one
    #
    # Example:
    #  1  nan  2
    # nan nan nan
    #  3  nan  4
    #
    # P will be of size 2x2.
    # Find 1 --> insert at the first pos. increase x counter, mark that something was found
    # Find 2 --> insert at the second pos. increase x counter
    # Next row: something was found, so increase y counter (0 to 1). reset x counter
    #   nan - nan - nan , so nothing gets inserted and nothing got marked
    # Next row: y counter is at 1, x counter was reset to 0
    # find 3, insert at 0,1. find 4, insert at 1,1.
    # done.

    # Determine the dimension of the matrix and create a stub for it
    no_entries = 0
    if box.covariance_center:
        no_entries += 2
    if box.covariance_velocity:
        no_entries += 2
    if box.covariance_length_width:
        no_entries += 2
    if box.covariance_angle:
        no_entries += 2
    P = np.zeros((no_entries, no_entries))  # covariance matrix that will be returned
    d = box.covariance.data  # array of covariance entries
    M = np.reshape(d, (7, 7))  # reshaped into matrix
    L = ~numpy.isnan(M)  # logical indexing array for inserting M into P (only not [~] nan[isnan])

    # now do the iteration as described above
    bm_x = 0  # bookmark x positions in P
    for x in range(L.shape[0]):
        bm_y = 0  # bookmark y positions in P
        any = False  # boolean that stores if "any" value was added to P in this row
        for y in range(L.shape[1]):
            # iterate over all entries in the logical indexing array (7x7)
            if L[x][y]:
                # use this entry for P
                P[bm_x][bm_y] = M[x][y]
                bm_y += 1
                any = True
        if any:
            bm_x += 1

    return P
