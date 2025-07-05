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

ros::Publisher pub_filtered;
ros::Publisher pub_min_y_point;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 从ROS消息转换为PCL点云
    pcl::fromROSMsg(*input, *cloud);

    // 直通滤波器
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);

    // x方向滤波
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-7, 8);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_filtered);

    // z方向滤波
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.3, 0.6);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_filtered);

    // y方向滤波
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-5, 10);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_filtered);

    // 统计滤波器
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.1);
    sor.filter(*cloud_filtered);

    // 找到y值最小的点
    pcl::PointXYZ min_y_point = cloud_filtered->points[0];
    for (size_t i = 1; i < cloud_filtered->points.size(); ++i)
    {
        if (cloud_filtered->points[i].y < min_y_point.y)
        {
            min_y_point = cloud_filtered->points[i];
        }
    }

    ROS_INFO("Point with the minimum y value: [%f, %f, %f]", min_y_point.x, min_y_point.y, min_y_point.z);

    // 转换回ROS消息并发布滤波后的点云
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header = input->header;
    pub_filtered.publish(output);

    // 发布y值最小点的位置
    geometry_msgs::Point min_y_point_msg;
    min_y_point_msg.x = min_y_point.x;
    min_y_point_msg.y = min_y_point.y;
    min_y_point_msg.z = min_y_point.z;
    pub_min_y_point.publish(min_y_point_msg);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "pointcloud_filter");
    ros::NodeHandle nh;

    // 创建订阅者，订阅输入的点云数据主题（假设主题名为 "input"）
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 1, cloudCallback);

    // 创建发布者，发布滤波后的点云数据（主题名为 "filtered_output"）
    pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("filtered_output", 1);

    // 创建发布者，发布y值最小点的位置（主题名为 "min_y_point"）
    pub_min_y_point = nh.advertise<geometry_msgs::Point>("min_y_point", 1);

    // 进入ROS事件循环
    ros::spin();

    return 0;
}
