"""
Includes functions that are used to convert TrackedBoxes to MarkerArrays that can be published to visualize results in
rviz.
"""
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from bob_perception_msgs.msg import *
import copy


def boxes_to_marker_array(boxes, color=(1,1,0,0)):
    """
    Converts a set of tracked oriented boxes to a markerarray and returns that
    :param boxes: Array of TrackedOrientedBoxes
    :param color: Color to be used, should be a tuple (rgba)
    :return: A MarkerArray
    """
    marker_array = MarkerArray()

    point_array = []

    for tracked_box in boxes:
        next_marker = Marker()
        # Copy timestamp and frame id of the point
        next_marker.header.frame_id = tracked_box.box.header.frame_id
        next_marker.header.stamp = tracked_box.box.header.stamp

        next_marker.ns = "T2TF_SST_namespace"  # ?
        next_marker.id = tracked_box.object_id
        next_marker.type = Marker.CUBE
        next_marker.action = Marker.ADD

        next_marker.pose.position.x = tracked_box.box.center_x
        next_marker.pose.position.y = tracked_box.box.center_y
        next_marker.pose.position.z = 0  # 3D data projected into 2D by ignoring z coordinate

        next_marker.pose.orientation.x = 0
        next_marker.pose.orientation.y = 0
        next_marker.pose.orientation.z = 0
        next_marker.pose.orientation.w = 1  # ?

        next_marker.scale.x = 1
        next_marker.scale.y = 1
        next_marker.scale.z = 0.1

        # color values of the marker
        next_marker.color.r = color[0]
        next_marker.color.g = color[1]  # rgb values
        next_marker.color.b = color[2]
        next_marker.color.a = color[3]  # alpha

        point_array.append(next_marker)

    marker_array.markers = point_array

    return marker_array


def merge_marker_array(arrays):
    """
    Takes n MarkerArrays and returns a merged version of them
    :param arrays: Array of MarkerArrays
    :return: Merge of all arrays in the parameters
    """
    if len(arrays) < 2:
        raise IndexError("Would attempt to merge an array of size 1 or smaller")
    full = arrays[0].markers
    for i in range(1, len(arrays)):
        full.extend(arrays[i].markers)

    extended_MA = MarkerArray()
    extended_MA.markers = full

    return extended_MA


def delete_with_first(marker_array):
    """
    Adds a new marker to the array that deletes all previous markers.
    Use this on an array that you want to display, but without showing all previous markers (i.e. clear before display)
    :param marker_array: MarkerArray to be used
    :return: MarkerArray similar to marker_array, but with a new marker added that deletes all previous markers
    """
    del_marker = copy.deepcopy(marker_array.markers[0])
    del_marker.action = Marker.DELETEALL  # Set the action
    # Set some stuff to prevent the marker from showing up at all
    del_marker.color.a = 0
    del_marker.scale.x = 0
    del_marker.scale.y = 0
    del_marker.scale.z = 0

    manipulated = []
    manipulated.append(del_marker)
    manipulated.extend(marker_array.markers)

    return manipulated
