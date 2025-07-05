#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub_filtered;
ros::Timer timer;


void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*input,*cloud);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(60);//参考点数目
    sor.setStddevMulThresh(0.05);//误差忍耐
    sor.filter(*cloud_filtered);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); //设置模型类型
    seg.setMethodType(pcl::SAC_RANSAC);    //设置随机采样一致性方法类型
    seg.setDistanceThreshold(0.01); //点到估计模型最大值
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers,*coefficients); //储存分割结果

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true); //设置非地面部分

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_ground(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*cloud_non_ground);

    // 直通滤波器
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_non_ground);

    // x方向滤波
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.4, 0.4);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_non_ground);

    // y方向滤波
    pass.setInputCloud(cloud_non_ground);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 4.0);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_non_ground);

    cloud_non_ground->erase(
    std::remove_if(cloud_non_ground->points.begin(), cloud_non_ground->points.end(),
                    [](const pcl::PointXYZ &p) {
                        return (p.x == 0.0 && p.y == 0.0 && p.z == 0.0);
                    }),
    cloud_non_ground->points.end());


    pcl::PointXYZ min_y_point = cloud_non_ground->points[0];
    for (size_t i = 1; i < cloud_non_ground->points.size(); ++i)
    {
        if (cloud_non_ground->points[i].y < min_y_point.y )
        {
            min_y_point = cloud_non_ground->points[i];
        }
    }

    //转换为ros消息并发布滤波后的点云
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_non_ground,output);
    output.header = input->header;
    pub_filtered.publish(output);

}

void timerCallback(const ros::TimerEvent&)
{
    ROS_INFO("Point with the minimum y value: [%f, %f, %f]", min_y_point.x, min_y_point.y, min_y_point.z);
}

int main (int argc,char **argv)
{
    ros::init(argc,argv,"pointcloud_non_plane");//初始节点
    ros::NodeHandle nh;

    //订阅主题为/livox/lidar
    ros::Subscriber sub = nh.subscribe("/livox/lidar",1,cloudCallback);
    
    //发布滤波后的点云数据，主题为cloud_non_ground
    pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("cloud_non_ground",1);

    timer = nh.createTimer(ros::Duration(5.0), timerCallback);

    ros::spin();

    return 0;
}