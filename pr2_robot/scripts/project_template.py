#!/usr/bin/env python
# -*- coding: utf-8 -*-

# To run
#  
#   $ roslaunch pr2_robot pick_place_project.launch
#
# In another terminal
#
#   $ rosrun pr2_robot project_template.py

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *
import sensor_msgs.point_cloud2 as pc2

import rospy
import tf
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import PickPlace
from rospy_message_converter import message_converter
import yaml
import pcl


def statistical_outlier_filter(cloud, mean_k = 50, threshold = 1.0):
    """
    statistical outlier filter

    params:
        cloud: pcl.PCLPointCloud2
        mean_k:
        threshold:
    returns: 
        pcl.PCLPointCloud2
    
    ref. http://pointclouds.org/documentation/tutorials/statistical_outlier.php
    """

    # Much like the previous filters, we start by creating a filter object: 
    outlier_filter = cloud.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(mean_k)

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(threshold)

    # Finally call the filter function for magic
    return outlier_filter.filter()


def voxel_filter(cloud, leaf_size = 0.01):
    """
    Voxel Grid filter for input point cloud(Downsampling filter)

    params:
        cloud: pcl.PCLPointCloud2
        leaf_size: The size of voxel grid. 
                   Large number is more downsampling.
    returns:
        filtered cloud: pcl.PCLPointCloud2
    """

    vox = cloud.make_voxel_grid_filter()
    # voxel size (also known as `leaf`)
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
    return vox.filter()


def pass_through_filter(cloud, filter_axis='z', axis_limit=(0.6, 1.1)):
    """
    Cut off filter

    Params:
        cloud: pcl.PCLPointCloud2
        filter_axis: 'z'|'y'|'x' axis 
        axis_limit: range

    Returns:
        filtered cloud: pcl.PCLPointCloud2
    """

    # Create a PassThrough filter object.
    passthrough = cloud.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_limit[0], axis_limit[1])

    return passthrough.filter()


def ransac_plane_segmentation(cloud, max_distance = 0.010):
    """
    Segmentation By RANSAC

    params:
        cloud: pcl.PCLPointCloud2
        max_distance: The float of threshold.

    returns:
        inliers: PointIndices 
    """

    # RANSAC plane segmentation
    seg = cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Set threshold
    # 1..0.1 : xx
    # 0.01.. : OK!
    seg.set_distance_threshold(max_distance)

    # Get inlier points
    inliers, _ = seg.segment()
    return  inliers


def euclidean_clustering(cloud, tolerance=0.001, cluster_range=(10,250)):
    """
    Euclidean Clustering using PCL with k-d tree

    returns:
        cluster_indices, white_cloud
    """

    # Obtain the Point Cloud with only XYZ, and convert k-d tree for PCL.
    white_cloud = XYZRGB_to_XYZ(cloud)
    kdtree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(tolerance)
    ec.set_MinClusterSize(cluster_range[0])
    ec.set_MaxClusterSize(cluster_range[1])
    
    # Search the k-d tree for clusters
    # k-d tree でクラスタリングする
    ec.set_SearchMethod(kdtree)

    # Extract indices for each of the discovered clusters
    # クラスタリングした点群の index を取り出す
    indices = ec.Extract()
    return indices, white_cloud


def create_colored_cluster_cloud(cluster_indices, white_cloud):
    """
    Group Point Cloud by color
    
    params:
        cluster_indices: [PointIndices]
        white_cloud: pcl.PointCloud_PointXYZ

    returns: pcl.PointCloud_PointXYZRGB
        XYZ -> XYZRGB
    """
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for _, indice in enumerate(indices):
            color_cluster_point_list.append([
                white_cloud[indice][0], white_cloud[indice][1], white_cloud[indice][2],
                rgb_to_float(cluster_color[j])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    return cluster_cloud


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
    rospy.loginfo(' Begin pcl_callback...')

    # Exercise-2 TODOs:

    # DONE: Convert ROS msg to PCL data
    # Using PointXYZRGB
    cloud = ros_to_pcl(pcl_msg)

    cloud_filtered = cloud

    # DONE: Statistical Outlier Filtering
    cloud_filtered = statistical_outlier_filter(cloud_filtered, mean_k=5, threshold=0.0001)

    # DONE: Voxel Grid Downsampling
    cloud_filtered = voxel_filter(cloud_filtered, leaf_size=0.01)

    # DONE: PassThrough Filter
    # Cut off the table top by z-axis.
    cloud_filtered = pass_through_filter(cloud_filtered, 
        filter_axis='z', axis_limit=(0.6, 1.1))
    # Cut off the near side of the table by x-axis.
    cloud_filtered = pass_through_filter(cloud_filtered,
        filter_axis='x', axis_limit=(0.33, 1.39))
    # Cut off the side of the table by y-axis.
    cloud_filtered = pass_through_filter(cloud_filtered,
        filter_axis='y', axis_limit=(-0.46, 0.46))

    # DONE: RANSAC Plane Segmentation
    inliers = ransac_plane_segmentation(cloud_filtered)
    # rospy.loginfo('type inliers after ransac_plane_segmentation %s ' % type(inliers))

    # DONE: Extract inliers and outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    # rospy.loginfo(' cloud_objects: %d' % (cloud_objects.size))

    # DONE: Euclidean Clustering
    cluster_indices, white_cloud = euclidean_clustering(cloud_objects,
        tolerance=0.02, cluster_range=(100,15000))
    # rospy.loginfo(' cluster_indices: %d, white_cloud: %d ' % (len(cluster_indices), white_cloud.size))

    # DONE: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_cloud = create_colored_cluster_cloud(cluster_indices, white_cloud)
    # rospy.loginfo(' colored cluster: %d' % (cluster_cloud.size))

    # DONE: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # DONE: Publish ROS messages
    rospy.loginfo(' Publish objects cloud')
    pcl_objects_pub.publish(ros_cloud_objects)

    rospy.loginfo(' Publish table cloud')
    pcl_table_pub.publish(ros_cloud_table)

    rospy.loginfo(' Publish colored cluster cloud')
    pcl_cluster_pub.publish(ros_cluster_cloud)

    rospy.loginfo('DONE the range of Exercise-2')


# Exercise-3 TODOs:
    detected_objects_labels = []
    detected_objects = []

    # Classify the clusters! (loop through each detected cluster one at a time)
    for index, pts_list in enumerate(cluster_indices):
        # DONE: Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)

        ros_cluster = pcl_to_ros(pcl_cluster)

        # DONE: Compute the associated feature vector
        #  Obtain the vector of object color histograms
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        #  Obtain the vector of object shape histograms 
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        #  Make the feature for prediction
        feature = np.concatenate((chists, nhists))

        # DONE: Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # DONE: Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # DONE: Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # DONE: Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)


    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass


def compute_centroid(cloud):
    """ Obtain cetoroid of Point Cloud """
    points = ros_to_pcl(cloud)
    np_points = np.mean(points, axis=0)[:3]
    return [ np.asscalar(x) for x in np_points]


# function to load parameters and request PickPlace service
def pr2_mover(detected_objects):
    rospy.loginfo('pr2_mover starting...')

    # DONE: Initialize variables
    scene_num = 1
    test_scene_num = Int32(data=scene_num)
    yaml_list = []

    # label: string -> DetectedObject
    detected_by_label = dict((x.label, x) for x in detected_objects)
    rospy.loginfo('  detected_label: %s' % ([ x.label for x in detected_objects ]))

    # DONE: Get/Read parameters
    # DONE: Parse parameters into individual variables
    # Obtain the original object list
    object_list_param = rospy.get_param('/object_list')
    rospy.loginfo('  object_list in config: %s' % (object_list_param))

    dropbox_param = rospy.get_param('/dropbox')
    rospy.loginfo('  dropbox in config: %s' % (dropbox_param))

    # group: string -> dropbox
    dropbox_by_group = dict((x['group'], x) for x in dropbox_param)


    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # DONE: Loop through the pick list
    rospy.loginfo('  Loop through the pick list')
    for i, obj in enumerate(object_list_param):

        rospy.loginfo('%d/%d: %s ...' % (i+1, len(object_list_param), obj['name']))

        # DONE: Get the PointCloud for a given object and obtain it's centroid
        # If not detected then ignore object.
        if not detected_by_label.has_key(obj['name']):
            rospy.loginfo('    %s NOT detected!' % (obj['name']))
            continue

        rospy.loginfo('    %s detected.' % (obj['name']))

        detected = detected_by_label[obj['name']]
        dropbox = dropbox_by_group[obj['group']]

        # Make some variables
        object_name = String(data=obj['name'])
        which_arm = String(data=dropbox['name'])

        # DONE: Create 'place_pose' for the object
        # place_pose = Pose(position=dropbox['position'])
        place_pose = Pose()
        place_pose.position.x = dropbox['position'][0]
        place_pose.position.y = dropbox['position'][1]
        place_pose.position.z = dropbox['position'][2]

        # DONE: Assign the arm to be used for pick_place
        centroid = compute_centroid(detected.cloud)
        pick_place = Pose()
        pick_place.position.x = centroid[0]
        pick_place.position.y = centroid[1]
        pick_place.position.z = centroid[2]

        rospy.loginfo('    pick_place.position %s' % pick_place.position)

        # DONE: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        rospy.loginfo('  making yaml item...')
        yaml_dict = make_yaml_dict(test_scene_num, which_arm, object_name, pick_place, place_pose)
        yaml_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        rospy.loginfo('    waiting pick_place_routine service...')
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # DONE: Insert your message variables to be sent as a service request
            rospy.loginfo('    request pick_place_routine.')
            resp = pick_place_routine(test_scene_num, object_name, which_arm, pick_place, place_pose)
            rospy.loginfo('    pick_place_routine done.')

            rospy.loginfo("    Response: %s" % resp.success)

        except rospy.ServiceException, e:
            rospy.loginfo("    Service call failed: %s" % e)


    # DONE: Output your request parameters into output yaml file
    if 0 < len(yaml_list): 
        rospy.loginfo('  Writing yaml...')
        send_to_yaml('output_%s.yaml' % scene_num, yaml_list)
        rospy.loginfo('  Writing yaml DONE')
    else:
        rospy.loginfo('  No detected(yaml not saved)')

    rospy.loginfo('pr2_mover DONE')


if __name__ == '__main__':

    rospy.loginfo('Initializing pr2-robot...')
    # DONE: ROS node initialization
    rospy.init_node('pr2-robot', anonymous=True)

    # DONE: Create Subscribers
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2, pcl_callback, queue_size=1)
    rospy.loginfo('Created subscriber')


    # DONE: Create Publishers

    # For debugging publisher in Rviz like Exercise-1
    # The Point Cloud of Objects
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    rospy.loginfo('Created Publisher for objects')
    
    # The Point Cloud of the table Only
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)
    rospy.loginfo('Created Publisher for table')

    # Clustered Point Cloud Objects
    pcl_cluster_pub = rospy.Publisher('/pcl_cluster', PointCloud2, queue_size=1)
    rospy.loginfo('Created Publisher for objects cluster')


    # DONE: here you need to create two publishers
    # Call them object_markers_pub and detected_objects_pub
    # Have them publish to "/object_markers" and "/detected_objects" with 
    # Message Types "Marker" and "DetectedObjectsArray" , respectively
    # 
    # Object maker for Rviz
    object_markers_pub = rospy.Publisher('/object_markers', Marker, queue_size=1)
    rospy.loginfo('Created Publisher for Markers')

    # The Result of object recognition 
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray, queue_size=1)
    rospy.loginfo('Created Publisher for DetectedObjectsArray')


    # DONE: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']


    # Initialize color_list
    get_color_list.color_list = []

    rospy.loginfo('Finished initializing')

    # DONE: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
