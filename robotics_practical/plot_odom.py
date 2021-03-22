""" this file can be used to plot various odometry messages from a given experiment"""

import argparse
import csv
import glob
import itertools
import math

import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import matplotlib.patches as mpatches


font = {'family' : 'normal',
    'weight' : 'normal',
    'size'   : 14}
matplotlib.rc('font', **font)

parser = argparse.ArgumentParser()

parser.add_argument("--input", help="dir containing csv files", required=True)
parser.add_argument("--no_gt", help="dont plot ground truth", action="store_true")
args = parser.parse_args()


def read_csv_from_dir(input_dir):
    csvs = list(glob.iglob(input_dir + "*.csv"))
    data = {}
    for csv_file in csvs:
        csv_name = csv_file.split("/")[-1][1:-4]  # extracts name from filepath so that it looks cleaner
        csv_name.replace("updated_shifted", "")
        data[csv_name] = []
        with open(csv_file, newline='') as csv_in:
            data[csv_name] = np.loadtxt(csv_in, delimiter=",")
    return data


def show_paths(data, args):
    """ plots several odom paths as scatterplot"""
    fig, ax = plt.subplots(figsize=(10, 10))

    plt.title("Odometry from sensors and filter")
    for sensor in data:
        if len(data[sensor]) == 0:
            print("skipping", sensor, " as data is empty")
        else:
            x = data[sensor][:, 0]
            y = data[sensor][:, 1]
            # plt.scatter(x,y, s=1)
            #print(sensor)
            dotsize = 10
            if sensor=="ground_truth_gps_utm_odom":
                dotsize=20
            ax.scatter(x, y, label=sensor,
                   alpha=0.9, s=dotsize)
 
    if not args.no_gt: 
        # plot ground truth rectangle
        boxfig = matplotlib.patches.Rectangle((0,0), 10, 7.3, angle=19.0-90, fill=False)
        plt.xlim(-5, 15)
        plt.ylim(-15, 5)
        plt.gca().add_patch(boxfig)
    handles, labels = ax.get_legend_handles_labels()
    gt = mpatches.Patch(color='black', label='Ground Truth', linewidth=3, linestyle='solid')
    if not args.no_gt: 
        handles.append(gt)
    lgnd = plt.legend(handles=handles, loc="lower left", numpoints=1, fontsize=10)

    #change the marker size manually for both lines
    for h in lgnd.legendHandles:
        h._sizes = [30]
    plt.show()

def find_nearest(array,value):
    """finds value in array closest to the given data point"""
    idx = np.searchsorted(array, value, side="left")
    if idx > 0 and (idx == len(array)\
        or math.fabs(value - array[idx-1]) < math.fabs(value - array[idx])):
        return idx-1
    else:
        return idx


def calc_interpolation(time, time_other, distance):
    """interpolates list of points in distance at points in time"""
    interpolations = []
    counter = 0
    for t in time_other:
        i = find_nearest(time, t)
        
        # edge cases
        if i >= len(distance)-1:
            interpolations.append(distance[-1])
            counter += 1
            continue
        elif t < time[0]:
            interpolations.append(distance[0])
            continue

        # linear interpolation between two points
        d1 = distance[i]
        d2 = distance[i+1]
        if time[i+1]-time[i] < 0.00001:
            m = 0
        else:
            m = (d2-d1)/(time[i+1] - time[i])
        d_interpolated = d1 + (t - time[i]) * m
        interpolations.append(d_interpolated)
    if False:
        plt.plot(time_other, interpolations)
        plt.plot(time, distance)
        plt.show()
    return interpolations


def calc_diff(data, sensor1, sensor2, interpolate=True):
    """ plots distance in meters from two sensors odometry.
    Works even when sensors use different frequencies"""

    gt = data[sensor1] # not actually ground truth but lets pretend
    x_gt = gt[:, 0]
    y_gt = gt[:, 1]
    time_gt = gt[:, 7]

    x = data[sensor2][:, 0]
    y = data[sensor2][:, 1]
    time = data[sensor2][:, 7]

    dist = []
    gt_time_index = []
    if not interpolate:
        for i in range(len(time)):
            i_gt = find_nearest(time_gt, time[i])
            gt_time_index.append(i_gt)
    else:
       gt_time_index = list(range(len(time)))
       x_gt = calc_interpolation(time_gt, time, x_gt)
       y_gt = calc_interpolation(time_gt, time, y_gt)

    for i in range(len(time)):
        latest_dist = np.sqrt((x_gt[gt_time_index[i]] - x[i])**2 + (y_gt[gt_time_index[i]] - y[i])**2 )
        dist.append(latest_dist)
        
    return dist, time


def show_diff(data, interpolate):
    """ show error between ground truth and various sensors"""
    sensors = list(data.keys())
    sensors = [s for s in sensors if s != 'ground_truth_gps_utm_odom']  # remove ground truth
    """
    pairs = list(itertools.combinations(sensors, r=2))

    for pair in pairs:
        dist, time = calc_diff(data, pair[0], pair[1], interpolate)
        plt.plot(time, dist, label=str(pair))
    """

    for sensor in sensors:
        dist, time = calc_diff(data, sensor, 'ground_truth_gps_utm_odom', interpolate)
        plt.plot(time, dist, label=str(sensor))

    plt.title("Distance in meters from all pairs of sensor")
    plt.legend()
    plt.show()


data = read_csv_from_dir(args.input)
show_paths(data, args)
#show_diff(data, False)
