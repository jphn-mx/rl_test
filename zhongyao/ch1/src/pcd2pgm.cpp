#include <ros/ros.h>

#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/passthrough.h>                 //直通滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波器头文件
#include <pcl/filters/voxel_grid.h>                  //体素滤波器头文件
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>//滤波
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>//分割

ros::Publisher pub1;
ros::Publisher pub2;

std::string map_topic_name;

nav_msgs::OccupancyGrid map_topic_msg;
//最小和最大高度
double thre_z_min = -2.0;
double thre_z_max = 2.0;
double map_resolution = 0.05;
double thre_radius = 0.1;

pcl::PointCloud<pcl::PointXYZ>::Ptr
    pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);

//地面分割后的pcd
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_ground(new pcl::PointCloud<pcl::PointXYZ>);

//转换为栅格地图数据并发布
//void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map,
 //                   nav_msgs::OccupancyGrid &msg);

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_filters");
  ros::NodeHandle nh;

  map_resolution = 0.05;
  map_topic_name = std::string("map");

//pcd
  ros::Subscriber sub = nh.subscribe("/livox/lidar", 1, cloudCallback);

  //pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_non_ground", 1);

  pub1 = nh.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1);

  pub2 = nh.advertise<sensor_msgs::PointCloud2>("cloud_non_ground",1);
  //转换为栅格地图数据并发布

  //SetMapTopicMsg(cloud_after_PassThrough, map_topic_msg);
  ros::spin();
  return 0;
}
/*
//转换为栅格地图数据并发布
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map,
                    nav_msgs::OccupancyGrid &msg) 
{
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = ros::Time::now();
  msg.info.resolution = map_resolution;

  double x_min, x_max, y_min, y_max;
  double z_max_grey_rate = 0.05;
  double z_min_grey_rate = 0.95;
  //? ? ??
  double k_line =
      (z_max_grey_rate - z_min_grey_rate) / (thre_z_max - thre_z_min);
  double b_line =
      (thre_z_max * z_min_grey_rate - thre_z_min * z_max_grey_rate) /
      (thre_z_max - thre_z_min);

  if (cloud_map->points.empty()) 
  {
    ROS_WARN("pcd is empty!\n");
    return;
  }

  for (int i = 0; i < cloud_map->points.size() - 1; i++) {
    if (i == 0) {
      x_min = x_max = cloud_map->points[i].x;
      y_min = y_max = cloud_map->points[i].y;
    }

    double x = cloud_map->points[i].x;
    double y = cloud_map->points[i].y;

    if (x < x_min)
      x_min = x;
    if (x > x_max)
      x_max = x;

    if (y < y_min)
      y_min = y;
    if (y > y_max)
      y_max = y;
  }
  // origin的确定
  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  //设置栅格地图大小
  msg.info.width = int((x_max - x_min) / map_resolution);
  msg.info.height = int((y_max - y_min) / map_resolution);
  //实际地图中某点坐标为(x,y)，对应栅格地图中坐标为[x*map.info.width+y]
  msg.data.resize(msg.info.width * msg.info.height);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  ROS_INFO("data size = %d\n", msg.data.size());

  for (int iter = 0; iter < cloud_map->points.size(); iter++) {
    int i = int((cloud_map->points[iter].x - x_min) / map_resolution);
    if (i < 0 || i >= msg.info.width)
      continue;

    int j = int((cloud_map->points[iter].y - y_min) / map_resolution);
    if (j < 0 || j >= msg.info.height - 1)
      continue;
    // 栅格地图的占有概率[0,100]，这里设置为占据
    msg.data[i + j * msg.info.width] = 100;
    //    msg.data[i + j * msg.info.width] = int(255 * (cloud->points[iter].z *
    //    k_line + b_line)) % 255;
  }
}
*/


void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *cloud);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.01);
    sor.filter(*cloud_filtered);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.07);
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_non_ground);

    sor.setInputCloud(cloud_non_ground);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.01);
    sor.filter(*cloud_non_ground);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_non_ground, output);
    output.header = input->header;
    pub2.publish(output);

    // Populate map_topic_msg here
    map_topic_msg.header.seq = 0;
    map_topic_msg.header.stamp = ros::Time::now();
    map_topic_msg.header.frame_id = "map";

    map_topic_msg.info.map_load_time = ros::Time::now();
    map_topic_msg.info.resolution = map_resolution;

    double x_min, x_max, y_min, y_max;

    // Calculate x_min, x_max, y_min, y_max from cloud_non_ground

    // Set origin
    map_topic_msg.info.origin.position.x = x_min;
    map_topic_msg.info.origin.position.y = y_min;
    map_topic_msg.info.origin.position.z = 0.0;
    map_topic_msg.info.origin.orientation.x = 0.0;
    map_topic_msg.info.origin.orientation.y = 0.0;
    map_topic_msg.info.origin.orientation.z = 0.0;
    map_topic_msg.info.origin.orientation.w = 1.0;

    // Calculate width and height
    map_topic_msg.info.width = int((x_max - x_min) / map_resolution);
    map_topic_msg.info.height = int((y_max - y_min) / map_resolution);

    // Populate map data
    map_topic_msg.data.resize(map_topic_msg.info.width * map_topic_msg.info.height);
    map_topic_msg.data.assign(map_topic_msg.info.width * map_topic_msg.info.height, 0);

    for (size_t iter = 0; iter < cloud_non_ground->points.size(); ++iter) {
        int i = int((cloud_non_ground->points[iter].x - x_min) / map_resolution);
        if (i < 0 || i >= map_topic_msg.info.width)
            continue;

        int j = int((cloud_non_ground->points[iter].y - y_min) / map_resolution);
        if (j < 0 || j >= map_topic_msg.info.height - 1)
            continue;

        map_topic_msg.data[i + j * map_topic_msg.info.width] = 100;
    }

    pub1.publish(map_topic_msg);
}
