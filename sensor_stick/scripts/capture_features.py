#!/usr/bin/env python
# -*- coding: utf-8 -*-

# To run for make training data
#  
#   $ roslaunch sensor_stick training.launch
#
# In another terminal
#
#   $ rosrun sensor_stick capture_features.py


import numpy as np
import pickle
import rospy

from sensor_stick.pcl_helper import *
from sensor_stick.training_helper import spawn_model
from sensor_stick.training_helper import delete_model
from sensor_stick.training_helper import initial_setup
from sensor_stick.training_helper import capture_sample
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.srv import GetNormals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2


def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


if __name__ == '__main__':
    rospy.init_node('capture_node')

    models = [\
        'biscuits',
        'book',
        'eraser',
        'glue',
        'snacks',
        'soap',
        'soap2',
        'soda_can',
        'sticky_notes',
       ]
    
    # defaut: max_sample_num = 5
    max_sample_num = 1000

    # Disable gravity and delete the ground plane
    initial_setup()
    labeled_features = []

    rospy.loginfo('Begin to capture...')
    for idx_model, model_name in enumerate(models):
        rospy.loginfo('  Spawn model: %s(%d/%d)' % 
                (model_name, idx_model+1, len(models)))
        spawn_model(model_name)


        for i in range(max_sample_num):
            rospy.loginfo('    Sampling...: %s(%d/%d) %d/%d' %
                    (model_name, idx_model+1, len(models), i+1, max_sample_num))

            # make five attempts to get a valid a point cloud then give up
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:
                rospy.loginfo('      try count: %d' % try_count)
                sample_cloud = capture_sample()
                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()

                # Check for invalid clouds.
                if sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected')
                    try_count += 1
                else:
                    sample_was_good = True

            # Extract histogram features
            rospy.loginfo('    Extract histogram features')
            chists = compute_color_histograms(sample_cloud, using_hsv=True)
            normals = get_normals(sample_cloud)
            nhists = compute_normal_histograms(normals)
            feature = np.concatenate((chists, nhists))
            labeled_features.append([feature, model_name])

            rospy.loginfo('    Sampling DONE')

        delete_model()


    rospy.loginfo(' dump labeled features')
    pickle.dump(labeled_features, open('training_set.sav', 'wb'))


    rospy.loginfo('DONE')

