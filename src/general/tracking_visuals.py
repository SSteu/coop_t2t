#!/usr/bin/env python
"""
This includes the TrackVisuals class, which can be used to display tracking data.

Alternatively, visualization can be done in rviz by publishing MarkerArrays. This allows for easy overlaying of tracking
data and original point clouds and is significantly faster.
"""
import numpy as np
import pandas as pd
import rospy
from bob_perception_msgs.msg import *
import matplotlib.animation as plt_animation
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from bob_perception_msgs.msg import *
import copy


class TrackVisuals:
    """
    Class that stores relevant information regarding the visuals and has all methods to plot further data etc.
    Use this to visualize tracking data.
    Displays points by x/y coordinates, annotates them with an id (numeric) and allows for setting of color.

    For example, point 3 will be drawn at (x[3], y[3]) in color color[3], and will be annotated based on id[3]

    Axis can be either symmetric by giving "None" as an upper limit for the y-Axis or can be specified during the init.

    This is not optimized for performance.
    """

    fig = None  # Figure of the matplotlib graph
    ax = None  # Axes object in the graph
    ids = []  # List of the ids that are currently in use
    x = []  # List of x Coordinates
    y = []  # List of y Coordinates
    ann_list = []  # List of references to all annotations that are in use
    colormap = []  # List
    def_color = 'b'  # Default color, if no color map is given, this will be used for all points
    neg_limit = -50
    limit = 50
    limit_y = 50
    neg_limit_y = 50
    annotation_method = None
    window_name = "Tracking Visualization"
    bounding_boxes = []  # List of bounding boxes as tuple (x,y,l,w,id,color)
    rectangle_refs = []  # List of bounding box patch rectangle reference (for deletion)

    def __init__(self, limit=50, neg_limit=-50, limit_y=None, neg_limit_y=-50, color='b'):
        """
        Initializes the TrackVisuals object.
        :param limit: The axis upper limit
        :param neg_limit: The axis lower limit
        :param limit_y: If passed, (neg-)limit will be used for x-Axis only and this will be used for the y-Axis. If not
                        passed or None, the limits will be symmetric instead
        :param neg_limit_y: The y-axis lower limit. Only used if limit_y was passed and is not None
        :param color: The default color to be used if no colormap is passed while plotting
        """
        # Set limits
        self.limit = limit
        self.neg_limit = neg_limit
        if limit_y is None:
            self.limit_y = limit
            self.neg_limit_y = neg_limit
        else:
            self.limit_y = limit_y
            self.neg_limit_y = neg_limit_y

        # Set everything to parameters/init everything else
        self.fig = plt.figure(self.window_name)
        self.ids = []
        self.ax = plt.axes(xlim=(self.neg_limit, self.limit), ylim=(self.neg_limit_y, self.limit_y))
        self.x, self.y = [], []
        self.bounding_boxes = []
        self.def_color = color
        # self.ax.autoscale(enable=False)  # Appears to change nothing, should prevent axis autoscaling
        self.sc = self.ax.scatter(self.x, self.y, c=color)

        # Set up the plot annotation, currently just the defaults
        self.default_annotation()
        # Needs to be called more than once to preserve it through updates, so store it
        self.annotation_method = self.default_annotation

    def plot_points(self, x_new, y_new, uid, color=[], append=False, alpha=1, annotate=True):
        """
        Plots tracking data in the graph, marking it with annotations of the corresponding UID.
        This resets all previously plotted data, use add_point if you want to add a single point.
        All parameter arrays should have the same size (they will be cut down to the size of the smallest one, excluding
        an empty color array)
        A color array can be specified, in this array every point gets assigned its color. If this is not specified or
        "[]" is passed, the default color that was specified during init will be used for all points. This does not
        affect annotation color.
        The append parameter allows you to specify whether you want to overwrite all currently drawn points. Appending
        points can be useful to plot data from different sources in the same plot. If the new color map is empty, the
        entire colormap will be reset as well.
        If you have multiple data sources, one should be set to append=False and the rest to append=True. Then, one will
        periodically wipe all data, and the others will add new information afterwards. Appending data without
        synchronized sources is not recommended due to this.
        :param x_new: The x position of the points as an array
        :param y_new: The y position of the points as an array
        :param uid: The numerical identifier of the points (which will be used as a label as well)
        :param color: The colormap for this set. If not specified or "[]", the default color will be used for all points
        :param append: If True, the old points will not be deleted. If False, only the newly passed points will be shown
        :param alpha: Alpha value of the plotted points (opacity)
        :param annotate: If True, every point will be annotated with object ids. If False, no such annotation will be
        done
        """
        # The arrays are all cut down to the size of the smallest one that was passed
        # This always includes a check whether or not color was passed as an argument
        if len(color) > 0:
            min_length = min([len(x_new), len(y_new), len(uid), len(color)])
        else:
            min_length = min([len(x_new), len(y_new), len(uid)])
        if append:
            self.x.extend(x_new[0:min_length])
            self.y.extend(y_new[0:min_length])
            self.ids.extend(uid[0:min_length])
            if len(color) > 0:
                self.colormap = np.hstack([self.colormap, color[0:min_length]])
            else:
                self.colormap = []  # No colors were specified
        else:
            self.x = x_new[0:min_length]  # Expand list of points by the next point
            self.y = y_new[0:min_length]  # as above
            self.ids = uid[0:min_length]
            # Cut colors down in size if colors were specified
            if len(color) > 0:
                self.colormap = color[0:min_length]
            else:
                self.colormap = []  # No colors were specified

        # Remove annotations
        for i, a in enumerate(self.ann_list):
            try:
                a.remove()
            except ValueError:
                # In case some multi-threading has already cleared all the data, don't attempt to remove it
                pass
        self.ann_list[:] = []  # clear the annotation list
        # Update the plot
        # You may also use: "self.sc.set_offsets(np.c_[self.x, self.y])", however this makes changing the colors harder

        # If the colormap is not empty (i.e. something was passed as an argument), change the colors as necessary
        self.clear_plot()  # necessary if you use ax.scatter to redraw the point
        if len(self.colormap) > 0:
            # Split up the array into multiple ones, where each one corresponds to a single color in the color map
            unique_colors = set(self.colormap)
            for next_color in unique_colors:
                # Get the indices of the current color
                next_positions = [i for i, j in enumerate(self.colormap) if j == next_color]
                # Select only the points of the next color
                next_x = [self.x[i] for i in next_positions]
                next_y = [self.y[i] for i in next_positions]
                # Plot these points using their corresponding color
                self.ax.scatter(next_x, next_y, c=next_color, alpha=alpha)
        else:
            # Else: use the default color
            self.ax.scatter(self.x, self.y, c=self.def_color, alpha=alpha)
        self.set_limits()  # Necessary because ax.scatter keeps autoscaling, even with autoscale=False

        if annotate:
            # Add new annotations
            for i, txt in enumerate(self.ids):
                self.ann_list.append(self.ax.annotate(str(txt), (self.x[i], self.y[i])))

        # Pause for a very short time to let the graphics redraw
        # plt.pause(0.0000001)
        plt.gcf().canvas.draw()

    def plot_points_tuple(self, points, append=False, alpha=1, annotate=True):
        """
        Performs a plot_points operation, however this takes a list of 4-tuples (x,y,id,c) instead of four arrays that
        represent the points.

        This function just unpacks the array
        :param points: This list of points as a list of 4-tuples (x,y,id,c)
        :param append: Whether to append the points or redraw. See plot_points documentation.
        :param alpha: Alpha value of the plotted points (opacity)
        :param annotate: If True, every point will be annotated with object ids. If False, no such annotation will be
        done
        """
        x_pos, y_pos, ids, color_list = [], [], [], []
        for p in points:
            x_pos.append(p[0])
            y_pos.append(p[1])
            ids.append(p[2])
            color_list.append(p[3])

        # pass this data to the plot_points function where it will be plotted
        self.plot_points(x_pos, y_pos, ids, color_list, append=append, alpha=alpha, annotate=annotate)

    def plot_box_array(self, boxes, color='b', append=False, alpha=1, annotate=True):
        """
        THIS FUNCTION IS OUTDATED.
        THIS IS BASED ON THE OLD plot_points OPERATION.
        To plot an array of boxes, please use scatter_box_array.
        The old plot_points object uses local lists to store all plotted points, which leads to a lack of flexibility
        in advanced parameters such as alpha.

        Performs a plot_points operation based on data extracted from an array of TrackedOrientedBoxes.
        :param boxes: Data in the format of an array of TrackedOrientedBoxes
        :param color: Color to use for the plot (since this is not provided in the TrackedOrientedBox array)
        :param append: Whether to append the points or redraw. See plot_points documentation.
        :param alpha: Alpha value of the plotted points (opacity)
        :param annotate: If True, every point will be annotated with object ids. If False, no such annotation will be
        done
        """
        points = []  # Array where all data points will be stored
        for tracked_box in boxes:
            # Extract relevant information for the next box
            object_id = tracked_box.object_id
            oriented_box = tracked_box.box
            x = oriented_box.center_x
            y = oriented_box.center_y
            t = (x, y, object_id, color)
            points.append(t)  # Append this information to the list of points

        # pass this data on to the plot_points_tuple function where it will be processed further
        self.plot_points_tuple(points=points, append=append, alpha=alpha, annotate=annotate)

    def scatter_box_array(self, boxes, color='b', append=False, alpha=1, annotate=True):
        """
        New and updated function that should be used to scatter plot an array of boxes.

        :param boxes: Data in the format of an array of TrackedOrientedBoxes
        :param color: Color to use for the plot (since this is not provided in the TrackedOrientedBox array)
        :param append: Whether to append the points or redraw the entire plot.
        :param alpha: Alpha value of the plotted points (opacity)
        :param annotate: If True, every point will be annotated with object ids. If False, no such annotation will be
        done
        """
        x = []
        y = []
        ids = []
        for box in boxes:
            x.append(box.box.center_x)
            y.append(box.box.center_y)
            ids.append(box.object_id)

        # Remove annotations
        # Only do this if you don't append to the data, if you do append keep all annotations for the boxes
        # because: annotate = False: ignore everything about annotations
        if (not append) and annotate:
            for i, a in enumerate(self.ann_list):
                try:
                    a.remove()
                except ValueError:
                    # In case some multi-threading has already cleared all the data, don't attempt to remove it
                    pass
            self.ann_list[:] = []  # clear the annotation list

        if not append:
            plt.cla()

        self.ax.scatter(x, y, color=color, alpha=alpha)

        if annotate:
            # Add new annotations
            for i, txt in enumerate(ids):
                self.ann_list.append(self.ax.annotate(str(txt), (x[i], y[i])))

        plt.xlim(self.neg_limit, self.limit)
        plt.ylim(self.neg_limit_y, self.limit_y)

        plt.gcf().canvas.draw()

    def plot_box_array_rectangles(self, boxes, color='b', append=False):
        """
        Performs a plotting operation for rectangles (i.e. bounding boxes) that were passes as TrackedOrientedBoxes
        :param boxes: Array of TrackedOrientedBoxes containing the information to plot
        :param color: Color to use for the plot (since this is not provided in the TrackedOrientedBox array)
        :param append: Whether to append the points or redraw. See plot_points documentation.
        """
        rectangles = []  # Array where all data points will be stored
        for tracked_box in boxes:
            # Extract relevant information for the next box
            object_id = tracked_box.object_id
            oriented_box = tracked_box.box
            x = oriented_box.center_x
            y = oriented_box.center_y
            l = oriented_box.length
            w = oriented_box.width
            t = (x, y, l, w, object_id, color)
            rectangles.append(t)  # Append this information to the list of bounding boxes

        self.plot_bounding_boxes(boxes=rectangles, append=append)
        # pass extracted data to the relevant function to plot it

    def plot_bounding_boxes(self, boxes, append=False):
        """
        Plots a single bounding box (i.e. a rectangle specified by position and x+y coordinates
        :param boxes: Array of Tuples (x, y, length, width, id, plot_color) for the bounding box
        :param append: If True, the current plot will not be erased on plotting. If False, this will redraw for only
        the array of information in boxes
        """
        # Check if the relevant data should be appended or not

        if not append:
            # Should not append, clear all boxes
            for rect in self.rectangle_refs:
                try:
                    rect.remove()
                except ValueError:
                    # wasn't in list, possibly already removed (multi threading?)
                    pass

        self.bounding_boxes = boxes

        # Remove annotations
        # Only do this if you don't append to the data, if you do append keep all annotations for the boxes
        if not append:
            for i, a in enumerate(self.ann_list):
                try:
                    a.remove()
                except ValueError:
                    # In case some multi-threading has already cleared all the data, don't attempt to remove it
                    pass
            self.ann_list[:] = []  # clear the annotation list

        # Plot all bounding boxes
        for box in self.center_to_bottom_left(boxes):
            x, y, l, w , object_id, color = box  # Extract current relevant information
            # Width/height vs box.{length/width} may need double checking, depending on the rotation
            # (rotation of box size is currently not implemented)
            vis_box = patches.Rectangle((x, y), width=l, height=w, angle=0.0,
                                        linewidth=1, edgecolor=color, facecolor=color, alpha=0.3, fill=True)
            self.rectangle_refs.append(vis_box)
            self.ax.add_patch(vis_box)  # Add the rectangle to the plot
            self.ann_list.append(self.ax.annotate(str(object_id), (x, y)))  # Annotate the rectangle

        plt.gcf().canvas.draw()

    @staticmethod
    def center_to_bottom_left(boxes):
        """
        Converts a set of rectangles with x/y as center coords to a set of boxes with bottom-left coordinates.
        :param boxes: The original boxes as tuples (x, y, l, w, object_id, color) in an array
        :return: The boxes moved by half of their length/width to fit the usual rectangle definition.
        """
        adjusted = []
        for box in boxes:
            x, y, l, w, object_id, color = box  # Extract current relevant information
            x_new = x-w/2
            y_new = y-l/2
            box_new = (x_new, y_new, l, w, object_id, color)
            adjusted.append(box_new)
        return adjusted

    def add_point(self, x_new, y_new):
        """
        Add a point to the display, without any annotation.
        Since this clutters the plot, it should only be used for testing purposes.
        :param x_new: The x coordinate of the new point
        :param y_new: The y coordinate of the new point
        """
        self.x.append(x_new)  # Expand list of points by the next point
        self.y.append(y_new)  # see above
        self.sc.set_offsets(np.c_[self.x, self.y])
        plt.pause(0.00001)

    def clear_plot(self):
        """
        Clear the plot
        """
        self.ax.clear()
        self.annotation_method()
        # plt.grid()  # Enable showing the background grid

    def default_annotation(self):
        """
        Sets up default annotation for the plot
        """
        self.ax.set_xlabel("X Coordinate of the Object")
        self.ax.set_ylabel("Y Coordinate of the Object")
        self.ax.set_title("Visualization of tracked objects")

    def set_limits(self):
        """
        Set the x and y limits on the axis to the set values.
        Necessary if some function calls mess with the desired values.
        """
        plt.xlim(self.neg_limit, self.limit)
        plt.ylim(self.neg_limit_y, self.limit_y)

    def get_ax(self):
        """
        Getter function to get the ax object, if further manipulation of the visuals
        should be done outside of this class
        :return: The axis object of this TrackVisuals Objects
        """
        return self.ax
