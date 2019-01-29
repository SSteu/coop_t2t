#!/usr/bin/env python
from __future__ import print_function
"""
Similar to general_plot.py, but uses historic T2TA instead.

This can play a bag file that includes multiple data sources and fuse them using T2TA approach that is incorporating
history into association.
The bag data is plotted as usual.
Association results are printed to the commandline, but can of course also be published to a new topic or handed over
to a fusion centre that then performs T2TF with the resulting data

Takes one optional argument: the name of the bag file in the data/ folder. if nothing is provided, the program idles 
until rosbag play abc.bag is used to play another bag file.
See README.txt for more information about directory structure.
EXAMPLE CALL:
rosrun coop_t2t historic_assoc.py mavenNew_small.bag
(assuming you are in a directory that has the subdirectory data/ which contains mavenNew_small.bag)

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

from general.tracking_visuals import *
import matplotlib.pyplot as plt
import subprocess
from tf2_msgs.msg import TFMessage
import threading as thr
import tf
from geometry_msgs.msg import PointStamped, Point
from simulation.sim_classes import SimulatedVehicle
import general.t2ta_algorithms as t2ta
import os
from general.t2t_history import *
import copy
import tf_conversions as tf_c
import pickle
from visualization_msgs.msg import MarkerArray
import general.visual_publishing as vis_pub
from manual_RA_match import avg_fusion


# --- callback functions
def callback_tracking(data):
    """
    Plot data from the original laser scans and from the c2x scan to the visuals object and additionally perform
    T2TA on this data, printing the results to the command line
    """
    # insert the basic code like in general_plot.py here to display everything etc
    # additionally, add the t2t_history code that updates the t2th object and calls the t2ta_historic code
    global visuals, lock, transforms, steps, c2x, inc_c2x, history, t2ta_thresh, state_space, use_identity, c2x_offset_x, c2x_offset_y, c2x_offset_test
    lock.acquire()
    c2x_selection = closest_match(c2x, data.header.stamp)
    if c2x_selection is not None:
        # Remove the selected c2x entry from the list of all c2x entries so that it doesn't get used twice
        c2x.remove(c2x_selection)

    global plot_bounding_boxes, do_visual_plot
    if do_visual_plot:
        if plot_bounding_boxes:
                visuals.plot_box_array_rectangles(data.boxes, color='b', append=False)
        else:
            visuals.scatter_box_array(data.boxes, append=False)

    # Append the current information to the history object
    history.add("lidar_0", data.boxes)

    if c2x_selection is not None:
        # Also include c2x data in the plot

        # ---
        # The following is the c2x data transformation, where the c2x pos/vel etc are transformed into the same coord.
        # frame as the tracking data that was received in this time step.
        # For this, all tracks in the c2x data are transformed using the transformer object.
        # Afterwards, T2TA is attempted on the resulting data. Results of this are printed to console, but currently
        # not passed on in any way. A future implementation should probably include a publisher, that publishes the
        # resulting data to a new topic, that a t2tf client can subscribe to.
        # ---

        tracks = c2x_selection.tracks
        # transformer: tf.TransformerROS
        for track in tracks:
            time_obj = rospy.Time(0)
            x_pos = track.box.center_x
            y_pos = track.box.center_y
            # Experimental transform of box l/w, velocity
            x_vel = track.box.velocity_x
            y_vel = track.box.velocity_y
            length = track.box.length
            width = track.box.width
            try:
                next_point = (x_pos, y_pos, track.object_id, "y")
                if plot_bounding_boxes:
                    global do_visual_plot
                    if do_visual_plot:
                        visuals.plot_box_array_rectangles([track], color="y", append=True)
                else:
                    global do_visual_plot
                    if do_visual_plot:
                        # visuals.plot_points_tuple([next_point], append=True)
                        visuals.scatter_box_array([track], color="y", append=True)
            except tf.ExtrapolationException as e:
                # Extrapolation error, print but keep going (possible just because only one frame was received so far)
                print(e)
            except tf.ConnectivityException as e:
                # print("Connectivity Exception during transform")
                print(e)
        # End of for going over tracks
        steps += 1  # c2x was used in another step, increase the counter

        # Now to the T2TA with history:
        try:
            # generate object ids and sensor names
            lidar_ids = []
            c2x_ids = []
            for obj in data.boxes:
                lidar_ids.append(obj.object_id)
            for obj in c2x_selection.tracks:
                c2x_ids.append(obj.object_id)
            obj_ids = [lidar_ids, c2x_ids]

            sensor_names = ["lidar_0", "c2x_0"]

            # Data acquisition from the history object should be based on the most recently received objects stamp
            timing = data.boxes[0].box.header.stamp
            assoc = t2ta.t2ta_historic(obj_ids, sensor_names, t2ta_thresh, hist_size, history, time=timing,
                                       state_space=state_space, use_identity=use_identity)
            ids = []  # this list will hold lists where each entry is an object id in a cluster
            if not do_visual_plot:
                # Simple print if no visual plot was done to indicate if the program is still running
                print("[Number of clusters: "+str(len(assoc))+" ]")
            for a in assoc:  # get a list of all associations
                temp = []  # stores ids for one association
                for box in a:  # all tracking boxes in the current association
                    temp.append(box.object_id)
                ids.append(temp)  # ids is a list made up of lists of object ids
                if len(a) > 1:
                    # If a non-singleton cluster was found, print all ids that belong to it
                    print("<<  Non-singleton Cluster: " + str(temp) + "  >>")
                    pass


                # DEBUG
                # The following block checks for association clusters of size 2
                # and then proceeds to store the distance between the boxes in these clusters
                # Additionally, a print of the distance with the number of assoc. made is printed
                if len(a) == 2:
                    global avg_dist
                    dist = t2t_distance_box(box_a=a[0], box_b=a[1], state_space=state_space, use_identity=use_identity)
                    try:
                        avg_dist
                    except NameError:  # not yet initialized
                        avg_dist = []
                    avg_dist.append(dist)
                    # print("No. assoc:"+str(len(avg_dist))+"\t Avg Distance: "+str(sum(avg_dist)/len(avg_dist)))

                # DYNAMIC OFFSET CHANGING -- only for testing purposes!
                # dynamically modify the offset of the c2x vehicle
                # in every step, check if reducing or increasing the offset by c2x_offset_test will improve the
                # distance between the 2 associated objects
                # if yes, modify the offset parameter by that value
                if c2x_offset_test != 0.0:
                    if len(a) == 2:  # TESTING INCREMENTAL OFFSET
                        # found an association between the two tracks
                        dist = t2t_distance_box(box_a=a[0], box_b=a[1], state_space=state_space, use_identity=use_identity)
                        # id == 100 in the c2x data for the ego-vehicle --> compare to that
                        # test_box_a will gets its value changed
                        if a[0].object_id == 100:
                            test_box_a = copy.deepcopy(a[0])
                            test_box_b = copy.deepcopy(a[1])
                        else:
                            test_box_b = copy.deepcopy(a[0])
                            test_box_a = copy.deepcopy(a[1])
                        # calc dist_up
                        test_box_a.box.center_x += c2x_offset_test
                        dist_up = t2t_distance_box(box_a=test_box_a, box_b=test_box_b,
                                                   state_space=state_space, use_identity=use_identity)
                        # calc dist_down (subtract c2x_offset_test twice, because you added it once previously)
                        test_box_a.box.center_x -= 2*c2x_offset_test
                        dist_down = t2t_distance_box(box_a=test_box_a, box_b=test_box_b,
                                                     state_space=state_space, use_identity=use_identity)
                        if dist_up < dist:
                            # found an improvement by increasing the distance
                            c2x_offset_x += c2x_offset_test
                            print("Offset++ ->"+str(c2x_offset_x)+"\t distance -:"+str(dist-dist_up))
                        if dist_down < dist:
                            # found an improvement by increasing the distance
                            c2x_offset_x -= c2x_offset_test
                            print("Offset-- ->" + str(c2x_offset_x) + "\t distance -:" + str(dist - dist_down))
                # END OF DYNAMIC CHANGING
        except ValueError as e:
            print("ValueError during association")
            # print(e)
        except IndexError as e:
            print("IndexError during association, likely because not enough data was received yet.")

    # Visualize in MarkerArrays for rviz:
    if c2x_selection is not None:
        global vis_publisher
        marker_color = (0, 0, 1, 0.5)  # blue boxes for the lidar boxes
        lidar_markers = vis_pub.boxes_to_marker_array(data.boxes, marker_color)
        marker_color = (0, 0.4, 0, 0.5)  # yellow boxes for the c2x boxes
        c2x_markers = vis_pub.boxes_to_marker_array(c2x_selection.tracks, marker_color)
        for marker in c2x_markers.markers:  # need to fix the frame id to prevent unnecessary transform
            marker.header.frame_id = "ibeo_front_center"  # ensure that rviz does not try to transform markers further
        marker_color = (1, 0, 0, 1)  # red, non-opaque boxes for the visualization of assoc/fusion

        assoc_boxes = avg_fusion(assoc)  # use this to average between assoc results and display that
        # assoc_boxes = non_singletons(assoc)  # use this two display 2 boxes (only association overlay)

        assoc_markers = vis_pub.boxes_to_marker_array(assoc_boxes, marker_color)

        global publish_assoc_markers
        if publish_assoc_markers:  # check if association/fusion markers should be published or only raw data
            all_markers = vis_pub.merge_marker_array([lidar_markers, c2x_markers, assoc_markers])  # merge markers
        else:
            all_markers = vis_pub.merge_marker_array([lidar_markers, c2x_markers])  # merge markers
        vis_publisher.publish(vis_pub.delete_with_first(all_markers))  # publish

    # ---  finish
    lock.release()


def callback_tf_static(data):
    """
    Acquires static tf data and adds it to the transformer object.

    Can also be used to add "normal" tf data to the transformer, but will rewrite the timestamp of such data to
    rospy.Time(0)
    """
    global transformer

    for tf_obj in data.transforms:
        # Forcing time==0 for the transformation to prevent issues that can potentially happen due to buffering or
        # data pre-processing/manipulation.
        tf_obj.header.stamp = rospy.Time(0)
        transformer.setTransform(tf_obj)


def callback_c2x_tf(data):
    """
    Stores the last c2x message, but transforms it beforehand.
    Transformation is only for position and velocity data, since this is the only data that is actively used by the
    program. The only exception is the plotting of bounding boxes. These will therefore not be rotated correctly, but
    their size will be correct anyway.
    """
    global history, steps, constant_velo, c2x_offset_x, c2x_offset_y
    tracks = data.tracks
    head = SimulatedVehicle.create_def_header(frame_id=src_id)
    # transformer: tf.TransformerROS
    for track in tracks:
        time_obj = rospy.Time(0)
        x_pos = track.box.center_x
        y_pos = track.box.center_y
        x_vel = track.box.velocity_x
        y_vel = track.box.velocity_y
        try:
            # Try to transform the point
            # Create a point with z=0 for the source frame (out of c2x tracks x/y) coordinate
            point = PointStamped(header=head,
                                 point=Point(x=x_pos, y=y_pos, z=0))
            # Don't use the current timestamp, use 0 to use the latest available tf data
            point.header.stamp = rospy.Time(0)
            # Now transform the point using the data
            tf_point = transformer.transformPoint(target_frame=dest_id, ps=point)

            # Acquire the transformation matrix for this
            tf_mat = tf_c.toMatrix(tf_c.fromTf(transformer.lookupTransform(target_frame=dest_id, source_frame=src_id, time=rospy.Time(0))))
            # print(tf_mat)

            # tf_vel stores the transformed velocity
            tf_vel = np.dot(tf_mat[0:2, 0:2], [x_vel, y_vel])
            # Update the track with the transformed data
            track.box.center_x = tf_point.point.x + c2x_offset_x
            track.box.center_y = tf_point.point.y + c2x_offset_y
            track.box.velocity_x = tf_vel[0]
            track.box.velocity_y = tf_vel[1]
            # DEBUG
            # print(str(track.box.center_x)+"\t"+str(track.box.center_y))
            # print("steps: "+str(steps)+"\tvelx: "+str(point_vel.point.x)+" vely: "+str(point_vel.point.y))
        except tf.ExtrapolationException as e:
            # Extrapolation error, print but keep going (possible just because only one frame was received so far)
            print(e)
        except tf.ConnectivityException as e:
            # print("Connectivity Exception during transform")
            print(e)
        except tf.LookupException as e:
            print(e)
            pass
    # Now change the frame_id to dest_id, since the tf was already performed
    data.header.frame_id = dest_id

    c2x.append(data)
    history.add("c2x_0", data.tracks)  # Add the data to the history object under the name c2x_0

    # The following block adds several fake measurements to the c2x tracking that are time delayed to the original
    # measurement from the current timestep
    # They also have changed x/y data based on velocity, so that the track doesn't need to be reused later on with the
    # outdated coordinates and instead can work with "updated" coordinates for time steps that are not included in the
    # real c2x messages.
    add_points = 4  # how many "fake" points should be added between this and the next measuremen
    # Currently the number and timing of "fake" points are fixed. They are selected to fit the data sets 1&2, but
    # in general it can be better to use previous data to estimate the time difference and number of points
    if len(data.tracks) > 0:  # only do the following if the track contained something
        for i in range(add_points):
            # add 4 more measurements, each based on the data, but with manipulated time and position based on velocity
            fake_data = copy.deepcopy(data)  # Create a copy of the data
            # now change the timestamp of this new data

            # c2x data arrives every 0.2 seconds, so to fit an additional 4 measurements
            time_shift = rospy.Duration(0, 40000000)  # increase by 1/20 of a sec
            # this would need to change if it should automatically fit to data of other data sets.

            fake_data.header.stamp = fake_data.header.stamp + time_shift
            for track in fake_data.tracks:
                track.box.header.stamp = track.box.header.stamp + time_shift
                track.box.center_x += constant_velo * track.box.velocity_x * (i+1)
                track.box.center_y += constant_velo * track.box.velocity_y * (i+1)
            c2x.append(fake_data)
            history.add("c2x_0", fake_data.tracks)

    steps = 0


def listener(args):
    """
    Prepare the subscribers and setup the plot etc
    :param args: The programs arguments, usually sys.argv unless you called setup from a different functionx
    """
    global visuals, transformer
    rospy.init_node('listener_laser_scan', anonymous=True)

    transformer = tf.TransformerROS(True)
    # print("FrameList:\t" + transformer.allFramesAsString())
    # Start the subscriber(s)
    rospy.Subscriber("tracked_objects/scan", TrackedLaserScan, callback_tracking)  # General subscriber (tracking data)

    # Currently using _static here because plotting should happen upon receiving lidar data (incl c2x plotting)
    rospy.Subscriber("tf", TFMessage, callback_tf_static)  # Acquire transform messages

    rospy.Subscriber("/FASCarE_ROS_Interface/car2x_objects", TrackedOrientedBoxArray, callback_c2x_tf)
    rospy.Subscriber("tf_static", TFMessage, callback_tf_static)  # Acquire static transform message for "ibeo" frames
    # rospy.Subscriber("tracked_objects/scan", TrackedLaserScan, callback_org_data)

    # setup a publisher for marker arrays
    global vis_publisher
    vis_publisher = rospy.Publisher("assoc_markers", MarkerArray, queue_size=100)

    if len(args) > 1:
        fname = args[1]  # Get the filename
        # now start a rosbag play for that filename
        FNULL = open(os.devnull, 'w')  # redirect rosbag play output to devnull to suppress it

        global play_rate
        rate = '-r' + str(play_rate)
        # using '-r 1' is the usual playback speed - this works, but since the code lags behind (cant process everything
        # in realtime), you will then get results after the bag finished playing (cached results)
        # using '-r 0.25' is still too fast for maven-1.bag
        # using '-r 0.2' works (bag finishes and no more associations are made on buffered data afterwards)

        global start_time
        time = '-s ' + str(start_time)
        if start_time > 0:
            pkl_filename = "./src/T2TF_SST/data/"  # folder
            pkl_filename += "tf_static_dump.pkl"  # filename
            with open(pkl_filename, 'rb') as pklinput:
                tf_static_data = pickle.load(pklinput)
                callback_tf_static(tf_static_data)

        player_proc = subprocess.Popen(['rosbag', 'play', rate, time, fname], cwd="data/")#, stdout=FNULL)

    global do_visual_plot
    if do_visual_plot:
        plt.show()  # DON'T NEED TO SPIN IF YOU HAVE A BLOCKING plt.show
    else:
        rospy.spin()

    # Kill the process (if it was started)
    if len(args) > 1:
        player_proc.terminate()


def setup(args=None):
    """
    Setup everything and call the listener function
    :param args: If None, sys.argv will be used as a parameter for the listener, else this should simulate cmdline
    parameters, so it should be ['path/to/file.py', 'bag_name'] where bagname is the name of the bag in the data folder.
    """
    global steps, src_id, dest_id, c2x, lock, history, hist_size, state_space, use_identity, transforms, t2ta_thresh, constant_velo, visuals, c2x_offset_x, c2x_offset_y, c2x_offset_test
    steps = 0  # how many steps passed since the last time the c2x message was used
    src_id = "odom"  # transformations are performed FROM this frame
    dest_id = "ibeo_front_center"  # transformations are performed TO this frame
    c2x = []  # Array that is used to store all incoming c2x messages
    lock = thr.Lock()
    history = TrackingHistory()
    # hist_size = rospy.Duration(0) => history will be only 1 track (i.e. no history)
    # hist size Duration(4) causes significant lag already!
    hist_size = rospy.Duration(0, 500000000)  # .5 secs
    hist_size = rospy.Duration(0)

    state_space = (True, False, False, False)  # usual state space: (TFFT), only pos: (TFFF)
    # The threshold NEEDS TO BE ADJUSTED if you use something other than TFFF!

    use_identity = True

    transforms = []
    # Create a new Visualization object with the axis limits and "blue" as default plotting color
    global do_visual_plot
    do_visual_plot = False  # do you want to plot data in matplotlib? (boolean)

    global publish_assoc_markers
    # False if you dont want to publish markers for association/fusion
    publish_assoc_markers = True

    if do_visual_plot:
        visuals = TrackVisuals(limit=65, neg_limit=-40, limit_y=50, neg_limit_y=-40, color='b')

    # define a similarity function for t2ta
    # Init the similarity checker that provides the similarity function
    # 20 works safe, but leads to some wrong associations between c2x data and road boundaries in very few scenarios
    # reduced it to 13 for now, but without doing extensive testing
    t2ta_thresh = 13  # Threshold for using only position data
    # t2ta_thresh = 25  # Threshold for using position + velocity data

    # value that is multiplied with velocity to acquire the position of the c2x objects in "untracked" time steps
    # i.e. between messages (since the frequency of the messages is lacking)
    # factor that needs to be used for velo when looking at change between 2 consecutive steps
    constant_velo = 0.05  # 0.05 performs best in my tests using maven-1.bag (if using c2x_offset_x=4)

    # Constant offsets for all c2x data
    # I assume the reason that this works/is necessary could be because the trackers have different "centers of tracking
    # boxes, and therefore this offset is necessary to "skip" the distance between the points
    c2x_offset_x = 4
    c2x_offset_y = 0

    c2x_offset_test = 0.0  # value that will be added+subtracted in every step to improve the above offset_x
    # set to 0.0 if you don't want dynamic offset changing
    # (should be 0.0 if you are not testing different offsets in a single run)

    global plot_bounding_boxes
    plot_bounding_boxes = False  # Set to True if you want to plot bounding boxes, False for a scatterplot of objects
    # (Bounding box data is not transformed, so the sizes will be correct but the rotation might be off)

    global play_rate, start_time
    play_rate = 0.5
    start_time = 0

    # Check which arguments should be used (parameter if possible, else sys.argv)
    if args is None:
        listener(sys.argv)
    else:
        listener(args)


if __name__ == '__main__':
    # Simply call the setup function, don't pass args so that sys.argv is used instead
    setup()


