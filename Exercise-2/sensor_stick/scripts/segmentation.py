#!/usr/bin/env python

# Import modules
from pcl_helper import *

# Voxel Grid filter
def voxel_downsample(cloud):
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    return vox.filter()

# PassThrough filter
def passthrough_filter(cloud):
    passthrough = cloud.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.7
    axis_max = 1.1 
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()


# RANSAC plane segmentation
def ransac_segment(cloud):
    seg = cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()
    objects = cloud.extract(inliers, negative=True)
    table = cloud.extract(inliers, negative=False)
    return objects, table


# Extract outliers
def statistical_outliers_extract(cloud):
    fil = cloud.make_statistical_outlier_filter()
    fil.set_mean_k(50)
    fil.set_std_dev_mul_thresh(1.0)
    return fil.filter()


def kmean_get_clusters(cloud):
    xyz_cloud = XYZRGB_to_XYZ(cloud)
    tree = xyz_cloud.make_kdtree()
    ec = xyz_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(2000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # Assign colors to points from each cluster
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append(
                                    [xyz_cloud[indice][0],
                                     xyz_cloud[indice][1],
                                     xyz_cloud[indice][2],
                                     rgb_to_float(cluster_color[j])])
    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    return cluster_cloud


# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data
    pcl_cloud = ros_to_pcl(pcl_msg)

    cloud = voxel_downsample(pcl_cloud)
    cloud = passthrough_filter(cloud)

    # segment objects/table
    cloud_objects, cloud_table = ransac_segment(cloud)
    cloud_objects = statistical_outliers_extract(cloud_objects)
    cloud_table = statistical_outliers_extract(cloud_table)

    # Clusterize and assign colors
    cluster_cloud = kmean_get_clusters(cloud_objects)

    # Generate and publish ROS messages
    pcl_objects_pub.publish(pcl_to_ros(cloud_objects))
    pcl_table_pub.publish(pcl_to_ros(cloud_table))
    pcl_cluster_pub.publish(pcl_to_ros(cluster_cloud))


if __name__ == '__main__':

    rospy.init_node('clustering', anonymous=True)

    # subscribe to point cloud
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    # publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []

    # main loop
    while not rospy.is_shutdown():
        rospy.spin()
