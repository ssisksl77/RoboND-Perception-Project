#!/usr/bin/env python

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

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


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

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)    
    # TODO: Statistical Outlier Filtering
    vox = pcl_data.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
    #NEW Outlier Removal Filter
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)  
    x = 1.0 
    outlier_filter.set_std_dev_mul_thresh(x)
    cloud_filter = outlier_filter.filter()
    # TODO: Voxel Grid Downsampling
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    axis_min = 0.6
    axis_max = 1.1
    # TODO: PassThrough Filter
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.038
    seg.set_distance_threshold(max_distance)
    # TODO: Extract inliers and outliers
    inliers, coefficients = seg.segment()
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    extracted_outliers= cloud_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(extracted_outliers)
    ec = white_cloud.make_EuclideanClusterExtraction()
    tree = white_cloud.make_kdtree()
    ec.set_ClusterTolerance(0.025)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(30000)
    cluster_indices = ec.Extract()
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                            rgb_to_float(cluster_color[j])])
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(extracted_outliers)
    ros_cloud_table = pcl_to_ros(extracted_inliers)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_cloud.publish(ros_cluster_cloud)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = extracted_outliers.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)
        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=False)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))  # where is clf
        label = encoder.inverse_transform(prediction)[0]
        # detected_objects.append(label)
        detected_objects_labels.append([feature, label])
        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_marker_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster_cloud
        detected_objects.append(do)
        # print('detected_object:: label={}'.format(label))
        
    # rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)
    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    ret_list = []
    # TODO: Get/Read parameters
    #  retrieve the list from the parameter server and parse
    object_list_param = rospy.get_param('/object_list')
    
    # TODO: Parse parameters into individual variables
    for i, object_param in enumerate(object_list_param):
        object_name = object_list_param[i]['name']
        object_group = object_list_param[i]['group']
    # TODO: Loop through the pick list
        for object_i, object_val in enumerate(object_list):
            if object_name == object_list[object_i]:
                continue
            # test_scene_num : Int32(), valid value: 1,2,3
            test_num = 3
            test_scene_num = Int32()
            test_scene_num.data = test_num  # 1, 2 or 3
             # object_name : std_msgs/String
            object_name = String()
            object_name.data = object_name
            # TODO: Assign the arm to be used for pick_place
            # arm_name : std_msgs/String, valid value: left, right
            arm_name = String()
            if object_group == 'green':
                arm_name.data = 'right'
            else:
                arm_name.data = 'left'
            # We need one more loop
            labels = []
            centroids = []
            for obj in object_list:
                labels.append(obj.label)
                point_arr = ros_to_pcl(obj.cloud).to_array()
                x = np.mean(point_arr, axis=0)[:3]
                # recast float type
                centroids = [np.asscalar(y) for y in x]
            # Assign Calculated pose of recognized object's CENTROID
            pick_pose = Pose()
            pick_pose.position.x = centroids[0]
            pick_pose.position.y = centroids[1]
            pick_pose.position.z = centroids[2]

            # find dropbox position match!
            box_pose = [0,0,0]  # x,y,z
            drop_box_param = rospy.get_param('/dropbox')
            for box in drop_box_param:
                if object_group == box['group']: # match! get the position
                    box_pos = box['position']
                    break

            # Assign Object Placement Post
            place_pose = Pose()
            place_pose.position.x = box_pose[0]
            place_pose.position.y = box_pose[1]
            place_pose.position.z = box_pose[2]

            # Finally, make_yaml_dict
            # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
            yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
            ret_list.append(yaml_dict)
            print('finished ', arm_name)
            break

    print('send to yaml')
    # print(ret_list)
    print('length of ret_list:', len(ret_list))
    # save yaml
    send_to_yaml('output_{}.yaml'.format(test_num), ret_list)
    print('write yaml')



if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    # TODO: Create Subscribers
    #pcl_sub = rospy.Subscriber('/senser_stick/point_cloud', pc2.PointCloud, pcl_callback, queue_size=1)
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2, pcl_callback, queue_size=1)
    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)
    pcl_cluster_cloud = rospy.Publisher('/pcl_cluster', PointCloud2, queue_size=1)

    object_marker_pub = rospy.Publisher('/object_markers', Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray, queue_size=1)
    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
