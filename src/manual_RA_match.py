#!/usr/bin/env python
"""
This script plays multiple bags (the two bags from the roundabout situation). One bag is played with additional delay
(the one that starts slightly early). Therefore the two bags are played at the same time.

This program sets up subscribers for both files, and converts information from the viewcar2 car to UTM and from
there into the fixed frame ibeo_front_center of the fascare car.

Afterwards, the data is associated, and results are printed.

This does not need any parameters, so it can simply be started using
    rosrun coop_t2t manual_RA_match.py

Optionally, the start time and the play rate can be given as parameters:
    rosrun coop_t2t manual_RA_match.py 252 0.5
This would start the bags at time=252secs and halve the speed at which they are played.

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
from __future__ import print_function
from general.tracking_visuals import *
import matplotlib.pyplot as plt
import subprocess
from tf2_msgs.msg import TFMessage
import tf
from geometry_msgs.msg import PointStamped, Point
from simulation.sim_classes import SimulatedVehicle
import general.t2ta_algorithms as t2ta
import os
from general.t2t_history import *
import copy
import pickle
from visualization_msgs.msg import MarkerArray
import general.visual_publishing as vis_pub


def velocity_cut(data):
    """
    Takes a TrackedLaserScan and removes all objects from it that have a velocity that is lower than the global
    variable threshold "velo_threshold"
    :param data: TrackedLaserScan of data to be cut
    :return: TrackedLaserScan that contains only objects that meet the above threshold
    """
    global velo_treshold
    cut_data = copy.deepcopy(data)
    to_remove = []
    for tracked_box in cut_data.boxes:
        velocity = (tracked_box.box.velocity_x ** 2 + tracked_box.box.velocity_y ** 2) ** 0.5
        if velocity < velo_threshold:
            to_remove.append(tracked_box)

    for rm in to_remove:
        cut_data.boxes.remove(rm)

    return cut_data


def callback_fascare(data):
    """
    Callback function for the fascare car.

    In addition to the received data, data from the viewcar2 data buffer is used
    """
    global do_velo_cut
    if do_velo_cut:
        data = velocity_cut(data)

    global viewcar2_odom_data, visuals, transformer_fascare, do_ego_plot
    try:
        viewcar2_odom_data
    except NameError:
        return  # No viewcar2 data was received so far, simply skip this step

    if len(data.boxes) == 0:
        # no boxes included in current message (possibly due to velocity cutting)
        return

    transparency = 0.2  # Used across all plots that should be transparant
    focus_association = True  # If True: No annotation for the rest of the plot, extra plot for associated objects

    global do_visual_plot
    if do_visual_plot:
        visuals.scatter_box_array(data.boxes, append=False, color="b", alpha=transparency, annotate=(not focus_association))

    # Currently the most recently stored entry is used as viewcar data.
    # In a more general case, where varying sensor transmission frequency needs to be taken into account, this should
    # instead use a function that determines the closest match w.r.t. time stamps.
    # For this, the difference in time stamps is minimal (only up to 0.02 seconds difference)
    vc_data = viewcar2_odom_data[-1]

    # Transform the last set of viewcar2_data into this frames ibeo_front_center
    src_id = "odom"
    dest_id = "ibeo_front_center"
    tracks = vc_data.boxes
    head = SimulatedVehicle.create_def_header(frame_id=src_id)
    for track in tracks:
        x_pos = track.box.center_x
        y_pos = track.box.center_y
        try:
            point = PointStamped(header=head, point=Point(x=x_pos, y=y_pos, z=0))
            point.header.stamp = rospy.Time(0)
            tf_point = transformer_fascare.transformPoint(target_frame=dest_id, ps=point)
            track.box.center_x = tf_point.point.x
            track.box.center_y = tf_point.point.y

            # if transforms of more than x/y pos. are necessary, insert them here

            track.box.header.frame_id = dest_id  # Changed the frame that the point is in

        # For all exceptions: print them, but keep going instead of stopping everything
        except tf.ExtrapolationException as e:
            # Extrapolation error, print but keep going (possible just because only one frame was received so far)
            print(e)
        except tf.ConnectivityException as e:
            # print("Connectivity Exception during transform")
            print(e)
        except tf.LookupException as e:
            # print("Lookup Exception during transform")
            print(e)
    vc_data.header.frame_id = dest_id

    global do_visual_plot
    if do_visual_plot:
        visuals.scatter_box_array(vc_data.boxes, append=True, color="y", alpha=transparency, annotate=(not focus_association))

    if do_ego_plot:
        # Plot the ego objects with alpha=1
        ego_fascare = copy.deepcopy(data.boxes[0])
        ego_fascare.box.center_x = 0
        ego_fascare.box.center_y = 0
        ego_fascare.object_id = -1

        ego_viewcar2 = copy.deepcopy(data.boxes[0])
        # Acquire the center of the point cloud
        try:
            if do_ego_plot:
                ego_point = PointStamped(header=head, point=Point(x=0, y=0, z=0))
                ego_point.header.stamp = rospy.Time(0)
                ego_point.header.frame_id = dest_id  # Start in frame dest_id=ibeo_front_center of viewcar2
                global transformer_viewcar2
                # Transform into src_id = "odom" first (from the viewcar2 ibeo_front_center)
                ego_point = transformer_viewcar2.transformPoint(target_frame=src_id, ps=ego_point)
                # and then use the other transformer to move it back into "ibeo_front_center", but for fascare
                ego_point = transformer_fascare.transformPoint(target_frame=dest_id, ps=ego_point)
        # For all exceptions: print them, but keep going instead of stopping everything
        except tf.ExtrapolationException as e:
            # Extrapolation error, print but keep going (possible just because only one frame was received so far)
            print(e)
        except tf.ConnectivityException as e:
            # print("Connectivity Exception during transform")
            print(e)
        except tf.LookupException as e:
            # print("Lookup Exception during transform")
            print(e)
        ego_viewcar2.box.center_x = ego_point.point.x
        ego_viewcar2.box.center_y = ego_point.point.y
        ego_viewcar2.object_id = -2

        # acquired 2 objects that can now be plotted
        global do_visual_plot
        if do_visual_plot:
            visuals.scatter_box_array([ego_fascare], append=True, color="b", alpha=1, annotate=False)
            visuals.scatter_box_array([ego_viewcar2], append=True, color="y", alpha=1, annotate=False)
    # ---

    # now do the association:
    # first, add the two tracks to the history
    global do_assoc
    if do_assoc:
        global history
        history.add("fascare", data.boxes)
        history.add("viewcar2", vc_data.boxes)
        assoc = t2tassoc(data.boxes, vc_data.boxes)

        global do_visual_plot  # check if visual plot is wanted
        if do_visual_plot and focus_association:  # Extra plot for associated objects
            assoc_boxes = non_singletons(assoc)
            visuals.scatter_box_array(assoc_boxes, append=True, color="r", alpha=1, annotate=True)

    # ---

    # Visualize in MarkerArrays for rviz:
    global vis_publisher
    marker_color = (0, 0, 1, 0.5)  # blue boxes for the fascar boxes
    fascar_markers = vis_pub.boxes_to_marker_array(data.boxes, marker_color)
    marker_color = (0, 0.4, 0, 0.5)  # green boxes for the viewcar boxes
    viewcar2_markers = vis_pub.boxes_to_marker_array(vc_data.boxes, marker_color)
    marker_color = (1, 0, 0, 1)  # red, non-opaque boxes for the visualization
    assoc_boxes = avg_fusion(assoc)  # use this to average between assoc results and display that
    # assoc_markers = non_singletons(assoc)  # use this two display 2 boxes (only association overlay)
    assoc_markers = vis_pub.boxes_to_marker_array(assoc_boxes, marker_color)

    global publish_assoc_markers
    if publish_assoc_markers:
        all_markers = vis_pub.merge_marker_array([fascar_markers, viewcar2_markers, assoc_markers])
    else:
        all_markers = vis_pub.merge_marker_array([fascar_markers, viewcar2_markers])  # skip assoc markers
    vis_publisher.publish(vis_pub.delete_with_first(all_markers))


def t2tassoc(data_a, data_b):
    """
    Associates the two datasets a and b (in the form of Arrays of TrackedOrientedBoxes)
    information is simply printed to the commandline and returned
    :param data_a: The list of TrackedOrientedBoxes for the fascare
    :param data_b: The list of TrackedOrientedBoxes for the viewcar2
    :return: Result of the association
    """

    global t2ta_thresh, hist_size, history, state_space, use_identity

    number_of_associations = 0

    try:
        # generate object ids and sensor names
        fascare_ids = []
        viewcar2_ids = []
        for obj in data_a:
            fascare_ids.append(obj.object_id)
        for obj in data_b:
            viewcar2_ids.append(obj.object_id)
        obj_ids = [fascare_ids, viewcar2_ids]

        sensor_names = ["fascare", "viewcar2"]

        # Data acquisition from the history object should be based on the most recently received objects stamp
        timing = data_a[0].box.header.stamp
        assoc = t2ta.t2ta_historic(obj_ids, sensor_names, t2ta_thresh, hist_size, history, time=timing,
                                   state_space=state_space, use_identity=use_identity)
        ids = []  # this list will hold lists where each entry is an object id in a cluster
        for a in assoc:  # get a list of all associations
            temp = []  # stores ids for one association
            for box in a:  # all tracking boxes in the current association
                temp.append(box.object_id)
            ids.append(temp)  # ids is a list made up of lists of object ids
            if len(a) > 1:
                # If a non-singleton cluster was found, print all ids that belong to it
                print("<<  Non-singleton Cluster: " + str(temp) + "  >>")
                number_of_associations += 1
    # --- except:
    except ValueError as e:
        print("ValueError during association")
        # print(e)
    except IndexError as e:
        print("IndexError during association, likely because not enough data was received yet.")
    print("Finished LIDAR STEP with "+str(number_of_associations)+" associations.")
    return assoc


def non_singletons(data):
    """
    Takes the result of an association step, and extracts all non-singleton clusters from
    :param data: List of Clusters, where each Cluster is a list of TrackedOrientedBox objects that belong to the same
     tracked object
    :return: List of TrackedOrientedBoxes that were found in non-singleton clusters in the input
    """
    object_list = []
    for cluster in data:
        if len(cluster) > 1:
            for oriented_box in cluster:
                object_list.append(oriented_box)

    for o in object_list:
        o.object_id = o.object_id

    return object_list


def avg_fusion(data):
    """
    Takes the result of an association step, removes all singleton clusters, and creates a list of TrackedOrientedBoxes.
    Every box in this list corresponds to the average of one of the non singleton clusters in the original data.
    :param data: List of Clusters, where each Cluster is a list of TrackedOrientedBox objects that belong to the same
     tracked object
    :return: List of TrackedOrientedBoxes representing the average of non-singleton clusters of the original data
    """
    avg_boxes = []
    non_single = []
    # First, remove all clusters:
    for cluster in data:
        if len(cluster) > 1:
            non_single.append(cluster)

    # Now, average each cluster
    for cluster in non_single:
        next_box = copy.deepcopy(cluster[0])
        next_box.object_id *= 100  # Append 2 0's to the object id
        # Currently, only doing x/y averaging
        x = 0
        y = 0
        for oriented_box in cluster:
            x += oriented_box.box.center_x
            y += oriented_box.box.center_y
        x /= len(cluster)
        y /= len(cluster)
        next_box.box.center_x = x
        next_box.box.center_y = y
        avg_boxes.append(next_box)

    return avg_boxes


def callback_viewcar2(data):
    """
    Callback function for the viewcar2 object.
    Received information is transformed to UTM and then saved to a buffer, so that the callback_fascare function
    can process the given information
    """
    # data: TrackedLaserScan

    global do_velo_cut
    if do_velo_cut:
        data = velocity_cut(data)

    if len(data.boxes) == 0:
        # empty, return
        return

    global append_ego
    if append_ego:
        ego_box = copy.deepcopy(data.boxes[0])  # copy any box
        ego_box.object_id = -2
        ego_box.box.center_x = 0
        ego_box.box.center_y = 0
        data.boxes.append(ego_box)

    global transformer_viewcar2

    src_id = "ibeo_front_center"  # Transform data from this frame...
    dest_id = "odom"  # ... to this.   # Using odom, but might need to use some other global frame?
    tracks = data.boxes
    head = SimulatedVehicle.create_def_header(frame_id=src_id)
    for track in tracks:
        x_pos = track.box.center_x
        y_pos = track.box.center_y
        try:
            point = PointStamped(header=head, point=Point(x=x_pos, y=y_pos, z=0))
            point.header.stamp = rospy.Time(0)
            tf_point = transformer_viewcar2.transformPoint(target_frame=dest_id, ps=point)
            track.box.center_x = tf_point.point.x
            track.box.center_y = tf_point.point.y

            # if transforms of more than x/y pos. are necessary, insert them here

            track.box.header.frame_id = dest_id  # Changed the frame that the point is in
        # For all exceptions: print them, but keep going instead of stopping everything
        except tf.ExtrapolationException as e:
            # Extrapolation error, print but keep going (possible just because only one frame was received so far)
            print(e)
        except tf.ConnectivityException as e:
            # print("Connectivity Exception during transform")
            print(e)
        except tf.LookupException as e:
            # print("Lookup Exception during transform")
            print(e)
    data.header.frame_id = dest_id

    global viewcar2_odom_data
    try:
        viewcar2_odom_data
    except NameError:
        viewcar2_odom_data = []
    viewcar2_odom_data.append(data)


def callback_tf_fascare(data):
    """
    Tf callback for the fascare data
    """
    global transformer_fascare

    for tf_obj in data.transforms:
        # Forcing time==0 for the transformation to prevent issues that can potentially happen due to buffering or
        # data pre-processing/manipulation.
        tf_obj.header.stamp = rospy.Time(0)
        transformer_fascare.setTransform(tf_obj)


def callback_tf_viewcar2(data):
    """
    Tf callback for the viewcar2 data
    """
    global transformer_viewcar2

    for tf_obj in data.transforms:
        # Forcing time==0 for the transformation to prevent issues that can potentially happen due to buffering or
        # data pre-processing/manipulation.
        tf_obj.header.stamp = rospy.Time(0)
        transformer_viewcar2.setTransform(tf_obj)


def listener(start=0.0, speed=1.0):
    """
    Creates the ROS node, starts playing the bag files etc.
    In general, starts execution of the program.
    :param start: starttime for the ROS bag playing
    :param speed: playback speed for the ROS bag
    """
    rospy.init_node('listener_dual_roundabout', anonymous=True)

    rospy.Subscriber("/tracked_objects/scan", TrackedLaserScan, callback_fascare)  # General subscriber (tracking data)
    rospy.Subscriber("/tracked_objects/scan_viewcar2", TrackedLaserScan, callback_viewcar2)

    rospy.Subscriber("/tf", TFMessage, callback_tf_fascare)
    rospy.Subscriber("/tf_viewcar2", TFMessage, callback_tf_viewcar2)

    # setup a publisher for marker arrays
    global vis_publisher
    vis_publisher = rospy.Publisher("assoc_markers", MarkerArray, queue_size=100)

    # Create global variables for the transformers
    global transformer_fascare, transformer_viewcar2
    transformer_viewcar2 = tf.TransformerROS(True)
    transformer_fascare = tf.TransformerROS(True)

    # Don't subscribe to static data, instead:
    # Use pickle to load the stored messages, so that even a starting delay does not cause any issues
    pkl_foldername = "./src/T2TF_SST/data/"  # folder
    pkl_filename = pkl_foldername+"tf_static_dump_fascare.pkl"  # filename
    with open(pkl_filename, 'rb') as pklinput:
        tf_static_data = pickle.load(pklinput)
        callback_tf_fascare(tf_static_data)
    # Do the same again but this time for the viewcar2 static tf data
    pkl_filename = pkl_foldername + "tf_static_dump_viewcar2.pkl"  # filename
    with open(pkl_filename, 'rb') as pklinput:
        tf_static_data = pickle.load(pklinput)
        callback_tf_viewcar2(tf_static_data)

    # Now, start the two processes that play the two bag files

    # viewcar2 starts slightly before fascare so the bag needs to be started with slight delay
    offset_viewcar2 = 2.58000016212  # THIS VALUE NEEDS TO BE EXACTLY THIS; THIS IS THE DIFF OF START TIMES!
    # the above value should only be modified if delay due to rosbag play launching causes them to lose sync again

    rate = '-r' + str(speed)
    starttime = '-s ' + str(start)
    starttime_early = '-s ' + str(start+offset_viewcar2)
    FNULL = open(os.devnull, 'w')  # redirect rosbag play output to devnull to suppress it

    # Start the fascare player
    fname = "roundabout_viewcar2_affix_early.bag"
    viewcar2_proc = subprocess.Popen(['rosbag', 'play', rate, starttime_early, fname], cwd="data/", stdout=FNULL)
    fname = "roundabout_fascare_later.bag"
    fascare_proc = subprocess.Popen(['rosbag', 'play', rate, starttime, fname], cwd="data/", stdout=FNULL)


    global do_visual_plot
    if do_visual_plot:
        plt.show()  # Start the graphics. This stops execution until the matplotlib window is closed
    else:
        rospy.spin()

    # Kill the processes after the matplotlib window was closed / node execution was stopped using ctrl-C
    fascare_proc.terminate()
    viewcar2_proc.terminate()


def setup():
    """
    Setup all necessary variables etc, and start the listener (which inits the execution of the program)
    """
    # VARIABLE DEFINITIONS
    global start_time, play_rate, t2ta_thresh, hist_size, state_space, use_identity, do_ego_plot, do_assoc, velo_threshold, do_velo_cut
    start_time = 252    # max is ~449.386
    if len(sys.argv) > 1:
        start_time = float(sys.argv[1])
    play_rate = 1
    if len(sys.argv) > 2:
        play_rate = float(sys.argv[2])
    t2ta_thresh = 50
    hist_size = rospy.Duration(0)
    state_space = (True, False, False, False)
    use_identity = True
    do_ego_plot = True
    do_assoc = True
    velo_threshold = 3  # Threshold for velocity cutoff (everything with smaller velocity will be thrown away)
    #   4 is pretty rough (empty at times) but clears everything up nicely
    #   3 was used widely during testing, works ok but leaves a bit of clutter
    #   2.5 has some clutter remaining, but still does fine overall
    do_velo_cut = True  # True, if velocity should be cut down using the above threshold

    global publish_assoc_markers
    # If set to false, no markers for association/fusion will be included in the assoc_markers topic used by rviz
    publish_assoc_markers = True

    global append_ego
    append_ego = True  # If true, the viewcar2 will append an ego position with coord 0,0 and id=-2

    global do_visual_plot
    do_visual_plot = False  # if set to false, no visualization code will be called
    visual_size = 100

    # ---
    # FURTHER SETUP
    global visuals
    if do_visual_plot:
        visuals = TrackVisuals(limit=visual_size, neg_limit=-visual_size, color='b')

    global history
    history = TrackingHistory()

    # Start the listener
    listener(start=start_time, speed=play_rate)


if __name__ == '__main__':
    setup()
