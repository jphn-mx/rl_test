#include <pcl/ModelCoefficients.h>           //采样一致性模型相关类头文件
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割类定义的头文件
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>


ros::Publisher pub_filtered;
ros::Timer timer;
pcl::PointXYZ min_x_point;
bool min_x_point_initialized = false;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 cloud_blob;
    pcl::fromROSMsg(*input,*cloud);
    //* the data should be available in cloud

    //过滤杂点
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50); // 参考点数目
    sor.setStddevMulThresh(0.01); // 误差忍耐
    sor.filter(*cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); // 设置模型类型
    seg.setMethodType(pcl::SAC_RANSAC);    // 设置随机采样一致性方法类型
    seg.setDistanceThreshold(0.1); // 平面与障碍距离
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients); // 储存分割结果

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // 设置非地面部分

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_ground(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*cloud_non_ground);

    // 直通滤波器（截断）
    pcl::PassThrough<pcl::PointXYZ> pass;
    // x方向滤波
    pass.setInputCloud(cloud_non_ground);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.4, 0.4);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_non_ground);

    // y方向滤波
    pass.setInputCloud(cloud_non_ground);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.4, 4.0);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_non_ground);

    if (!cloud_non_ground->points.empty())
    {
    // 法线估计
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud (cloud_non_ground);
      n.setInputCloud (cloud_non_ground);
      n.setSearchMethod (tree);
      n.setKSearch (20);
      n.compute (*normals);

    // Concatenate the XYZ and normal fields*
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
      pcl::concatenateFields (*cloud_non_ground , *normals, *cloud_with_normals);
    //*cloud_with_normals = cloud + normals

    // 创建搜索 tree*
      pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
      tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
      pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
      pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
      gp3.setSearchRadius (0.025);

    // Set typical values for the parameters
      gp3.setMu (2.5);
      gp3.setMaximumNearestNeighbors (100);
      gp3.setMaximumSurfaceAngle(M_PI/4); // 45 度
      gp3.setMinimumAngle(M_PI/18); // 10 度
      gp3.setMaximumAngle(2*M_PI/3); // 120 度
      gp3.setNormalConsistency(false);

    // 表面结果
      gp3.setInputCloud (cloud_with_normals);
      gp3.setSearchMethod (tree2);
      gp3.reconstruct (triangles);

      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(triangles.cloud, *output_cloud);

      pass.setInputCloud(output_cloud);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(-0.05,0.05);
      pass.setFilterLimitsNegative(false);
      pass.filter(*output_cloud);

      pass.setFilterFieldName("z");
      pass.setFilterLimits(-0.05,0.05);
      pass.setFilterLimitsNegative(false);
      pass.filter(*output_cloud);

      output_cloud->erase(
          std::remove_if(output_cloud->points.begin(), output_cloud->points.end(),
                        [](const pcl::PointXYZ &p) {
                            return (p.x == 0.0 && p.y == 0.0 && p.z == 0.0);
                        }),
          output_cloud->points.end());

      // 找到 x 值最小的点
      if (!output_cloud->points.empty())
      {
          min_x_point = output_cloud->points[0];
          for (size_t i = 1; i < output_cloud->points.size(); ++i)
          {
              if (output_cloud->points[i].y < min_x_point.y)
              {
                  min_x_point = output_cloud->points[i];
              }
          }
          min_x_point_initialized = true; // 表示min_x_point已初始化
      }

      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*output_cloud, output);
      output.header = input->header;
      pub_filtered.publish(output);
    }
  else
  {
    min_x_point_initialized = false; // 表示min_x_point已初始化
  }

}


void timerCallback(const ros::TimerEvent&)
{
    if (min_x_point_initialized) {
        ROS_INFO("Point with the minimum x value: [%f, %f, %f]", min_x_point.x, min_x_point.y, min_x_point.z);
    } else {
        ROS_WARN("min_x_point is not initialized yet.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tri_model"); // 初始节点
    ros::NodeHandle nh;

    // 订阅主题为 /livox/lidar
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 1, cloudCallback);
    
    // 发布滤波后的点云数据，主题为tri_model
    pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("tri_model", 1);

    // 设置发布频率
    timer = nh.createTimer(ros::Duration(0.5), timerCallback);

    ros::spin();

    return 0;
}
