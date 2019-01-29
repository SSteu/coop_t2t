#!/usr/bin/env python
"""
Performs a test run of a simulation
Output: rmse (measurement-groundtruth and fusion-groundtruth)

Performs 2 measurements and CI with those

No Visualization is done

Parameters:
    1: std_dev base used for the measurements
    2: additional std_dev used on top of std_dev (literally additive) for the second sensor

Example call to simulate two sensors with std_dev 2 and 3.5:
rosrun coop_t2t sim_RMSE.py 2 1.5


---
Note regarding global variable use:
The executable files all make heavy use of global variables, even though this is usually considered bad programming
practice. All basic functions for association, fusion, etc. are not affected by this.
It was simply used to allow for faster implementation, since this way information can be shared between callbacks for
ROS subscribers and other functions.

A more clean implementation would probably make use of a central instance of a class that can store all this
information, which would then be passed using kwargs.
---
"""
import rospy
import numpy
import sys
from bob_perception_msgs.msg import *
from simulation.sim_coordinator import *
import time
import math
import copy
from general.t2tf_algorithms import *
import threading as thr


def create_publishers(no_measures=2, qsize=10):
    """
    Creates ROS publishers for all necessary topics and a SimulationCoordinator object
    Topics used are:
        t2t_sim/truth  - for the ground truth
        t2t_sim/measured_X - 0<=X<no_measures - topics for the measurement (one each)
    :param no_measures: How many sensors should be simulated
    :param qsize: queue_size parameter for all ROS publishers
    :return: (ROS publisher for the ground truth, array of ROS publishers for measurements, SimulationCoordinator)
    """
    # Setup a publisher for the ground truth
    pub_truth = rospy.Publisher("t2t_sim/truth", TrackedOrientedBoxArray, queue_size=qsize)
    pub_measure = []
    # Setup a set of publishers, all stored in the pub_measure array
    for i in range(no_measures):
        topic_name = "t2t_sim/measured_"+str(i)
        pub_measure.append(rospy.Publisher(topic_name, TrackedOrientedBoxArray, queue_size=qsize))

    coordinator = SimulationCoordinator()  # Create a new SimulationCoordinator object

    return pub_truth, pub_measure, coordinator


def publish_all(pub_truth, pub_measure, coordinator):
    """
    Publish data on all publishers: the truth publisher and for every publisher in the array of measurement publishers.
    Additional information (std_deviation) is acquired from a global variable stddev.

    On top of the stddev variable, the global dev_step is used to determine the linear change of standard deviation
    between sensors.
    Sensor 0 will be use stddev, sensor 1 will use stddev+1*dev_step
                                 sensor 2 will use stddev+2*dev_step

    :param pub_truth: ROS publisher for the ground truth
    :param pub_measure: array of ROS publishers for the measurements
    :param coordinator: SimulationCoordinator object
    """
    global stddev, ground_truth_array
    pub_truth.publish(coordinator.get_box_array())
    ground_truth_array.append(coordinator.get_box_array())
    step_pos = 0
    for pub_m in pub_measure:
        global dev_step
        sd = stddev + dev_step*step_pos  # Standard deviation for the position of the current measurement

        pub_m.publish(coordinator.get_gaussian_box_array(sd_pos=sd))
        step_pos += 1


def setup():
    """
    Basic setup that creates all necessary global variables, creates subscribers and publishers and then proceeds
    to loop over the publish_all function to create the steps of the simulation (while moving the vehicles along the
    road)
    Uses the "small_highway" scenario defined by the SimulationCoordinator class.
    """
    global ground_truth_array
    ground_truth_array = []
    rospy.init_node("sim_testing", anonymous=True)
    # Setup all global variables
    global stddev, lock, storage, found_seqs
    stddev = 2  # Default value for the standard deviation of the measured data
    if len(sys.argv) > 1:
        stddev = float(sys.argv[1])
    # Default values if nothing was passed via sys.argv
    no_measures = 2
    qsize = 10
    sleep_time = 0.05
    no_steps = 15

    global dev_step
    dev_step = 0
    if len(sys.argv) > 2:
        dev_step = float(sys.argv[2])

    lock = thr.Lock()
    storage = []
    found_seqs = []
    subscriber(2)

    # Sample application for repeatedly moving a few cars along a "highway" and publishing this data to the topics
    pub_truth, pub_measure, coordinator = create_publishers(no_measures=no_measures, qsize=qsize)
    coordinator.small_highway_init()
    for s in range(no_steps):
        publish_all(pub_truth, pub_measure, coordinator)
        coordinator.move_all(steps=1)
        time.sleep(sleep_time)  # makes the function somewhat hard to stop, but is necessary for slowed exec.


def callback_ci(data):
    """
    Callback that performs covariance intersection information fusion
    association is performed via object ids (which are unchanged from the ground truth and therefore provide perfect
    assoc. results)
    
    Measures RMSE values and prints them to the command line.
    """
    global storage, found_seqs, pub, lock, ground_truth_array
    lock.acquire()

    seq = data.header.seq  # get seq. number
    seq -= 1  # account for indexing
    if seq in found_seqs:
        # found this sequence in the data, assuming that its position matches its sequence number
        fused_data = copy.deepcopy(data)  # fused_data will store the results, so copy to prevent overwriting
        other_data = storage[seq].tracks
        new_data = data.tracks
        c = 0  # counter for which box is currently being edited
        for tracked_box in new_data:
            # first, find a matching box in the other_data:
            oid = tracked_box.object_id
            for other_tracked_box in other_data:
                if other_tracked_box.object_id == oid:
                    # found a matching one
                    # Perform ci on the data
                    fused_data.tracks[c].box, omega = fusion(tracked_box.box, other_tracked_box.box)
                    break  # don't need to look further since we found a matching box already
            c += 1  # increment counter
        pub.publish(fused_data)

        # analyse fusion results
        x_diff = []
        y_diff = []
        global rmse_ci
        try:
            rmse_ci
        except NameError:
            rmse_ci = []
        for i in range(len(fused_data.tracks)):
            fusion_box = fused_data.tracks[i]
            real_box = ground_truth_array[-1].tracks[i]
            next_x_diff = fusion_box.box.center_x - real_box.box.center_x
            x_diff.append(next_x_diff * next_x_diff)  # square error for x
            next_y_diff = fusion_box.box.center_y - real_box.box.center_y
            y_diff.append(next_y_diff * next_y_diff)  # square error for y
        rmse_x_ci = math.sqrt(sum(x_diff)/len(x_diff))
        rmse_y_ci = math.sqrt(sum(y_diff) / len(y_diff))
        rmse_ci.append(math.sqrt(rmse_x_ci ** 2 + rmse_y_ci ** 2))  # pythagoras for xy distance

        # data contains the necessary information:
        x_diff = []
        y_diff = []
        global rmse_data
        try:
            rmse_data
        except NameError:
            rmse_data = []
        for i in range(len(data.tracks)):
            fusion_box = data.tracks[i]
            real_box = ground_truth_array[-1].tracks[i]
            next_x_diff = fusion_box.box.center_x - real_box.box.center_x
            x_diff.append(next_x_diff * next_x_diff)  # square error for x
            next_y_diff = fusion_box.box.center_y - real_box.box.center_y
            y_diff.append(next_y_diff * next_y_diff)  # square error for y
        rmse_x_data = math.sqrt(sum(x_diff) / len(x_diff))
        rmse_y_data = math.sqrt(sum(y_diff) / len(y_diff))
        rmse_data.append(math.sqrt(rmse_x_data ** 2 + rmse_y_data ** 2))  # pythagoras for xy distance

        # other_data contains the necessary information:
        x_diff = []
        y_diff = []
        global rmse_other_data
        try:
            rmse_other_data
        except NameError:
            rmse_other_data = []
        for i in range(len(other_data)):
            fusion_box = other_data[i]
            real_box = ground_truth_array[-1].tracks[i]
            next_x_diff = fusion_box.box.center_x - real_box.box.center_x
            x_diff.append(next_x_diff * next_x_diff)  # square error for x
            next_y_diff = fusion_box.box.center_y - real_box.box.center_y
            y_diff.append(next_y_diff * next_y_diff)  # square error for y
        rmse_x_other_data = math.sqrt(sum(x_diff) / len(x_diff))
        rmse_y_other_data = math.sqrt(sum(y_diff) / len(y_diff))
        rmse_other_data.append(math.sqrt(rmse_x_other_data ** 2 + rmse_y_other_data ** 2)) # pythagoras for xy distance)

        # ---
        global em_count
        try:
            em_count += 1
        except NameError:
            em_count = 0
        print("ERROR MEASUREMENT #"+str(em_count)+"[omega="+str(omega)+"]:")
        print("\tCI: " + str(sum(rmse_ci) / len(rmse_ci)))
        print("\tM1: " + str(sum(rmse_data)/len(rmse_data)))
        print("\tM2: " + str(sum(rmse_other_data)/len(rmse_other_data)))
        # m,c = min(averages of the measuements), average for ci
        m = min(sum(rmse_data)/len(rmse_data), sum(rmse_other_data)/len(rmse_other_data))
        c = sum(rmse_ci) / len(rmse_ci)
        # use m+c to calculate the improvement of the CI over the better measurement
        # if c=2, and m=3, then the improvement should be 33.3% == (m-c)/m*100
        print("\tI%: " + str((m-c)/m*100))

    else:
        if not len(storage) == seq:
            print("Issue at seq "+str(seq)+"\tdoesnt match storage with length"+str(len(storage)))
        storage.append(data)
        found_seqs.append(seq)
    lock.release()


def to_cov_mat(box):
    """
    Takes an OrientedBox as input and returns a numpy array of the covariance matrix
    :param box: The OrientedBox from which the covariance matrix should be extracted
    :return: A np array that includes the covariance matrix from the box, not including nans
    """
    cov_list = box.covariance.data
    cov_list = [x for x in cov_list if not math.isnan(x)]  # Remove all nans
    cov_list = np.array(cov_list)  # Turn into numpy array
    dim = 0  # the resulting array will be of shape (dim x dim)

    # ASSUMING THAT THE ORDER GIVEN IN THE DOCUMENTATION OF ORIENTED BOX IS CORRECT
    # usually you will only have box.covariance_center=True and box.covariance_velocity=True
    # And the resulting matrix will be a 4x4 matrix
    if box.covariance_center:
        dim += 2
    if box.covariance_angle:
        dim += 1
    if box.covariance_length_width:
        dim += 2
    if box.covariance_velocity:
        dim += 2
    cov_matrix = np.reshape(cov_list, (dim, dim))
    return cov_matrix


def fusion(track_i, track_j):
    """
    Takes in two tracks and fuses them according to the CI algorithm
    :param track_i: First track, in OrientedBox Format
    :param track_j: Second track, in OrientedBox Format
    :return:    Fused Track, in OrientedBox Format
                omega used
    """
    estimate = copy.deepcopy(track_i)  # Create an OrientedBox based on track i
    # PRESETS:
    t2tf = dual_cov_intersection
    # w_calc = dual_improved_omega  # Alt.: dual_fast_omega
    w_calc = dual_fast_omega
    # ---
    P_i = to_cov_mat(track_i)  # Covariance matrix of track i
    P_j = to_cov_mat(track_j)  # Covariance matrix of track j

    x_i = [track_i.center_x, track_i.center_y, track_i.velocity_x, track_i.velocity_y]  # Tracking estimate of track i
    x_i = np.array(x_i)
    x_j = [track_j.center_x, track_j.center_y, track_j.velocity_x, track_j.velocity_y]  # Tracking estimate of track j
    x_j = np.array(x_j)
    x, P, w = t2tf(P_i, P_j, x_i, x_j, omega_fct=w_calc)
    # Update the estimate based on x
    estimate.center_x = x[0]
    estimate.center_y = x[1]
    estimate.velocity_x = x[2]
    estimate.velocity_y = x[3]

    return estimate, w


def subscriber(no_measurements):
    """
    Currently known issue: because of the way data is stored, no_measurements should be 2.

    Subscribes to the measurement topics and creates a publisher for CI results that is stored in the global variable
    pub.
    :param no_measurements: Number of subscribers to create
    """
    # no_measurements should be 2 for now
    # since I use only one array to store data (so only the incoming track and one stored track are compared)
    # some generalizing is necessary first, but then an arbitrary number of sensors can be simulated
    global pub
    for i in range(no_measurements):
        tname = "/t2t_sim/measured_"+str(i)
        rospy.Subscriber(tname, TrackedOrientedBoxArray, callback_ci)
    pub = rospy.Publisher("t2t_sim/fused_CI", TrackedOrientedBoxArray, queue_size=10)


if __name__ == '__main__':
    print("Running main of rmse simulation testing")
    setup()
