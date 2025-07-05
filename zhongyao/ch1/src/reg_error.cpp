#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/shared_ptr.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/correspondence.h>
#include <pcl/filters/uniform_sampling.h>

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);    
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*input ,*cloud);

    pcl::PCDReader reader;
    reader.read("/home/mx/mxst/src/ch1/PCD/stairs_pass.pcd", *target);

    boost::shared_ptr<pcl::ModelCoefficients> coefficients(new pcl::ModelCoefficients);
    boost::shared_ptr<pcl::PointIndices> inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_non_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(boost::make_shared<std::vector<int>>(inliers->indices));
    extract.setNegative(true);
    extract.filter(*cloud_non_ground);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_non_ground);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.01);
    sor.filter(*cloud_filtered);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_non_ground);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.4, 0.4);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_non_ground);

    pass.setInputCloud(cloud_non_ground);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.4, 4);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_non_ground);

    pcl::PointCloud<pcl::Normal>::Ptr source_normal(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr target_normal(new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normal_estimate;
    normal_estimate.setRadiusSearch(0.03);
    normal_estimate.setInputCloud(cloud_non_ground);
    normal_estimate.compute(*source_normal);
    normal_estimate.setInputCloud(target);
    normal_estimate.compute(*target_normal);

    pcl::PointCloud<pcl::SHOT352>::Ptr source_descriptors(new pcl::PointCloud<pcl::SHOT352>);
    pcl::PointCloud<pcl::SHOT352>::Ptr target_descriptors(new pcl::PointCloud<pcl::SHOT352>);
    pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot_estimation;
    shot_estimation.setRadiusSearch(0.02);
    shot_estimation.setInputCloud(cloud_non_ground);
    shot_estimation.setInputNormals(source_normal);
    shot_estimation.compute(*source_descriptors);
    shot_estimation.setInputCloud(cloud_non_ground);
    shot_estimation.setInputNormals(target_normal);
    shot_estimation.compute(*target_descriptors);

    pcl::CorrespondencesPtr correspondence(new pcl::Correspondences);
    pcl::KdTreeFLANN<pcl::SHOT352> ktree;
    ktree.setInputCloud(target_descriptors);
    for (size_t i = 0; i < source_descriptors->size(); ++i) {
        std::vector<int> neighbors(1);
        std::vector<float> squared_distances(1);
        if (std::isfinite(source_descriptors->at(i).descriptor[0])) {
            int found_neighbors = ktree.nearestKSearch(source_descriptors->at(i), 1, neighbors, squared_distances);
            if (found_neighbors == 1 && squared_distances[0] < 0.25f) {
                pcl::Correspondence correspondenc(static_cast<int>(i), neighbors[0], squared_distances[0]);
                correspondence->push_back(correspondenc);
            }
        }
    }

    pcl::GeometricConsistencyGrouping<pcl::PointXYZ,pcl::PointXYZ> gc_cluster;
    gc_cluster.setGCSize(0.01);
    gc_cluster.setGCThreshold(5.0);
    gc_cluster.setInputCloud(cloud);
    gc_cluster.setSceneCloud(target);
    gc_cluster.setModelSceneCorrespondences(correspondence);

    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>> transformations;
    gc_cluster.recognize(transformations);

    ROS_INFO("变换矩阵 x:%f      y:%f      z:%f   ",transformations[0](0,3),transformations[1](1,3),transformations[2](2,3));

}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"recog");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("/livox/lidar",1,cloudCallback);
    ros::spin();
    return 0;
}