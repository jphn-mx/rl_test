#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudPublisher
{
public:
    PointCloudPublisher()
    {
        // 订阅点云话题
        sub = nh.subscribe("/scan1", 1, &PointCloudPublisher::pointCloudCallback, this);

        // 发布转换后的点云话题
        pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar", 1);
    }

    void pointCloudCallback(const sensor_msgs::PointCloudConstPtr &cloud_msg)
    {
        // 将PointCloud转换为PointCloud2
        sensor_msgs::PointCloud2 cloud2_msg;
        sensor_msgs::convertPointCloudToPointCloud2(*cloud_msg, cloud2_msg);

        // 转换PointCloud2为PCL点云
        pcl::PointCloud<pcl::PointXYZ> input_cloud;
        pcl::fromROSMsg(cloud2_msg, input_cloud);

        // 移除0,0,0处的点
        input_cloud.erase(
            std::remove_if(input_cloud.points.begin(), input_cloud.points.end(),
                           [](const pcl::PointXYZ &p) {
                               return (p.x == 0.0 && p.y == 0.0 && p.z == 0.0);
                           }),
            input_cloud.points.end());

        // 转换PCL点云为ROS消息
        sensor_msgs::PointCloud2 output_cloud_msg;
        pcl::toROSMsg(input_cloud, output_cloud_msg);
        output_cloud_msg.header.frame_id = "laser_livox"; // 根据实际情况设置坐标系

        // 发布转换后的点云消息
        pub.publish(output_cloud_msg);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_publisher_node");
    PointCloudPublisher point_cloud_publisher;

    ROS_INFO("PointCloud Publisher Node is running.");

    ros::spin();

    return 0;
}
