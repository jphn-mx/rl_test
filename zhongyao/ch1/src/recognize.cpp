#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/registration/icp.h>
#include <boost/shared_ptr.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
// recognize
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
ros::Publisher pub_filtered;
ros::Timer timer;
double obstacle_type = 0.0;
double obstacle_score = 0.0;


void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    //ROS_INFO("Callback triggered");
    
    
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target1(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target2(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target3(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target4(new pcl::PointCloud<pcl::PointXYZ>);
    /*
    pcl::PolygonMesh mesh;
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::loadPolygonFilePLY("/home/mx/mxst/src/ch1/PCD/stairs.ply", mesh);									//PCL利用VTK的IO接口，可以直接读取stl,ply,obj等格式的三维点云数据,传给PolygonMesh对象
	pcl::io::mesh2vtk(mesh, polydata);												//将PolygonMesh对象转化为vtkPolyData对象
	pcl::io::vtkPolyDataToPointCloud(polydata, *target1);
	pcl::io::savePCDFileASCII("bunny.pcd", *target1);									//存储为pcb文件
 
	char strfilepath[256] = "bunny.pcd";
	if (-1 == pcl::io::loadPCDFile(strfilepath, *target1)) 
    {							//读取pcb格式文件
		cout << "error input!" << endl;
	}
    */
    //ROS_INFO("PCD file read successfully");

    pcl::PCDReader read;
    read.read("/home/mx/mxst/src/ch1/PCD/stairs_pass.pcd", *target1);

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *cloud);
    //ROS_INFO("Point cloud converted from ROS message");

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.01);
    sor.filter(*cloud_filtered);
    //ROS_INFO("Statistical outlier removal applied");

    boost::shared_ptr<pcl::ModelCoefficients> coefficients(new pcl::ModelCoefficients);
    boost::shared_ptr<pcl::PointIndices> inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    //ROS_INFO("SACSegmentation applied");

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(boost::make_shared<std::vector<int>>(inliers->indices));
    extract.setNegative(true);

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_non_ground(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*cloud_non_ground);
    //ROS_INFO("Ground plane extracted");

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_non_ground);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.9, 0.9);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_non_ground);
    //ROS_INFO("PassThrough filter applied on y-axis");

    pass.setInputCloud(cloud_non_ground);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 7.0);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_non_ground);
    //ROS_INFO("PassThrough filter applied on x-axis");

    int iterations = 1;
    int tar;
    obstacle_type = 0;

    pcl::IterativeClosestPoint<PointT, PointT> icp;

    icp.setMaximumIterations(iterations);
    icp.setInputSource(cloud_non_ground);
    icp.setInputTarget(target1);
    icp.setMaxCorrespondenceDistance(10);
    //icp.setRANSACIterations(10);
    //icp.setRANSACOutlierRejectionThreshold(0.1);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    obstacle_score = icp.getFitnessScore();
    if (icp.hasConverged())
    {
        obstacle_type = 1.0;
    }
    else
    {
        obstacle_type = 0.0;
    }       
    
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_non_ground, output);
    output.header = input->header;
    pub_filtered.publish(output);
    //ROS_INFO("Point cloud published");
}

void resultinfo(const ros::TimerEvent&)
{
    ROS_INFO("Detected obstacle: %f,score:%f", obstacle_type,obstacle_score);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "recog");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/livox/lidar", 1, cloudCallback);
    pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("non_plane", 1);

    timer = nh.createTimer(ros::Duration(1.0), resultinfo);

    ros::spin();

    return 0;
}

