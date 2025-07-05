#include <pcl/ModelCoefficients.h>           //采样一致性模型相关类头文件
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>          //滤波相关类头文件
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割类定义的头文件
#include <pcl/surface/concave_hull.h>                 //创建凹多边形类定义头文件
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher pub_filtered;
pcl::PointXYZ min_x_point;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), 
                                    cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), 
                                    cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*input,*cloud);
/*
    for (size_t i=1;i<cloud->points.size();++i)
    {
        cloud->points[i].x = -cloud->points[i].x;
        cloud->points[i].z = -cloud->points[i].z;
    }
*/
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.4, 0.4);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_filtered);

    // y方向滤波
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.5,4.0);//0,4.0
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_filtered);


    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); // 设置模型类型
    seg.setMethodType(pcl::SAC_RANSAC);    // 设置随机采样一致性方法类型
    seg.setDistanceThreshold(0.2); // 点到估计模型最大值
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients); // 储存分割结果

    pcl::ProjectInliers<pcl::PointXYZ> proj;//点云投影滤波模型
    proj.setModelType (pcl::SACMODEL_PLANE); //设置投影模型
    proj.setIndices (inliers);             
    proj.setInputCloud (cloud_filtered);
    proj.setModelCoefficients (coefficients);      //将估计得到的平面coefficients参数设置为投影平面模型系数
    proj.filter (*cloud_projected);            //得到投影后的点云

    // 存储提取多边形上的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;        //创建多边形提取对象
    chull.setInputCloud (cloud_projected);       //设置输入点云为提取后点云
    chull.setAlpha (0.1);
    chull.reconstruct (*cloud_hull);   //创建提取创建凹多边形

    // 创建一个新的指针并指向临时的点云
    cloud_hull->erase(
        std::remove_if(cloud_hull->points.begin(), cloud_hull->points.end(),
                       [](const pcl::PointXYZ &p) {
                           return (p.x == 0.0 && p.y == 0.0 && p.z == 0.0);
                       }),
        cloud_hull->points.end());

    // 找到 x 值最小的点
    if (!cloud_hull->points.empty()) {
        min_x_point = cloud_hull->points[0];
        for (size_t i = 1; i < cloud_hull->points.size(); ++i)
        {
            if (cloud_hull->points[i].x < min_x_point.x)
            {
                min_x_point = cloud_hull->points[i];
            }
        }
    }

    printf("min x :%f \n",min_x_point.x);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_hull, output);
    output.header = input->header;
    pub_filtered.publish(output);


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_2d_plane"); // 初始节点
    ros::NodeHandle nh;

    // 订阅主题为 /livox/lidar
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 1, cloudCallback);
    
    // 发布滤波后的点云数据，主题为 cloud_non_ground
    pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("cloud_2d_plane", 1);

    ros::spin();

    return 0;
}
