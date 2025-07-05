#include <pcl/io/pcd_io.h>// 文件、设备读写
#include <pcl/point_cloud.h>//基础pcl点云类型
#include <pcl/correspondence.h>//分组算法 对应表示两个实体之间的匹配（例如，点，描述符等）。
// 特征
#include <pcl/features/normal_3d_omp.h>//法向量特征
#include <pcl/features/shot_omp.h> //描述子 shot描述子 0～1
// https://blog.csdn.net/bengyanluo1542/article/details/76061928?locationNum=9&fps=1
// (Signature of Histograms of OrienTations)方向直方图特征
#include <pcl/features/board.h>
// 滤波
#include <pcl/filters/uniform_sampling.h>//均匀采样 滤波
// 识别
#include <pcl/recognition/cg/hough_3d.h>//hough算子
#include <pcl/recognition/cg/geometric_consistency.h> //几何一致性
// kdtree
#include <pcl/kdtree/kdtree_flann.h>// kdtree 快速近邻搜索
#include <pcl/kdtree/impl/kdtree_flann.hpp>
// 转换
#include <pcl/common/transforms.h>//点云转换 转换矩阵
// 命令行参数
#include <pcl/console/parse.h>//命令行参数解析

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
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZ PointType;//PointXYZRGBA数据结构 点类型 位置和颜色
typedef pcl::Normal NormalType;//法线类型
typedef pcl::ReferenceFrame RFType;//参考帧
typedef pcl::SHOT352 DescriptorType;//SHOT特征的数据结构（32*11=352）
ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
//======== 【2】新建必要的 指针变量===================
  pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());//模型点云
  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());//模型点云的关键点 点云
  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());//场景点云
  pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());//场景点云的 关键点 点云
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());//模型点云的 法线向量
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());//场景点云的 法线向量
  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());//模型点云 特征点的 特征描述子
  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());//场景点云 特征点的 特征描述子
    pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
    
  pcl::fromROSMsg(*input, *model);
    //ROS_INFO("Point cloud converted from ROS message");
    pcl::PCDReader read;
    read.read("/home/mx/mxst/src/ch1/PCD/stairs_pass.pcd", *scene);
/*
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(model);
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
    pass.filter(*model);
*/
pcl::NormalEstimationOMP<PointType, NormalType> norm_est;//多核 计算法线模型 OpenMP
//  pcl::NormalEstimation<PointType, NormalType> norm_est;//多核 计算法线模型 OpenMP
  norm_est.setKSearch (10);//最近10个点 协方差矩阵PCA分解 计算 法线向量
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  //norm_est.setSearchMethod (tree);// 多核模式 不需要设置 搜索算法
  norm_est.setInputCloud (model);//模型点云
  norm_est.compute (*model_normals);//模型点云的法线向量

  norm_est.setInputCloud (scene);//场景点云
  norm_est.compute (*scene_normals);//场景点云的法线向量

  //=======【6】下采样滤波使用均匀采样（可以试试体素格子下采样）得到关键点=========

  pcl::UniformSampling<PointType> uniform_sampling;//下采样滤波模型
  uniform_sampling.setInputCloud (model);//模型点云
  uniform_sampling.setRadiusSearch (0.01);//模型点云搜索半径
  uniform_sampling.filter (*model_keypoints);//下采样得到的关键点

  uniform_sampling.setInputCloud (scene);//场景点云
  uniform_sampling.setRadiusSearch (0.03);//点云搜索半径
  uniform_sampling.filter (*scene_keypoints);//下采样得到的关键点

  //========【7】为keypoints关键点计算SHOT描述子Descriptor===========
  //
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;//shot描述子
  descr_est.setRadiusSearch (0.02);

  descr_est.setInputCloud (model_keypoints);
  descr_est.setInputNormals (model_normals);
  descr_est.setSearchSurface (model);
  descr_est.compute (*model_descriptors);//模型点云描述子

  descr_est.setInputCloud (scene_keypoints);
  descr_est.setInputNormals (scene_normals);
  descr_est.setSearchSurface (scene);
  descr_est.compute (*scene_descriptors);//场景点云描述子

  //
  //========【8】按存储方法KDTree匹配两个点云（描述子向量匹配）点云分组得到匹配的组====== 
  //
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());//最佳匹配点对组

  pcl::KdTreeFLANN<DescriptorType> match_search;//匹配搜索
  match_search.setInputCloud (model_descriptors);//模型点云描述子
  // 在 场景点云中 为 模型点云的每一个关键点 匹配一个 描述子最相似的 点
  for (size_t i = 0; i < scene_descriptors->size (); ++i)//遍历场景点云
  {
    std::vector<int> neigh_indices (1);//索引
    std::vector<float> neigh_sqr_dists (1);//描述子距离
    if (!pcl_isfinite (scene_descriptors->at(i).descriptor[0])) //跳过NAN点
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //在模型点云中 找 距离 场景点云点i shot描述子距离 <0.25 的点  
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      //   neigh_indices[0] 为模型点云中 和 场景点云 点   scene_descriptors->at (i) 最佳的匹配 距离为 neigh_sqr_dists[0]  
      model_scene_corrs->push_back (corr);
    }
  }

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;//变换矩阵 旋转矩阵与平移矩阵
// 对eigen中的固定大小的类使用STL容器的时候，如果直接使用就会出错 需要使用 Eigen::aligned_allocator 对齐技术
  std::vector<pcl::Correspondences> clustered_corrs;//匹配点 相互连线的索引
// clustered_corrs[i][j].index_query 模型点 索引
// clustered_corrs[i][j].index_match 场景点 索引
  //
    //=========计算参考帧的Hough（也就是关键点）=========
    //
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());//模型参考帧
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());//场景参考帧
    //======估计模型参考帧（特征估计的方法（点云，法线，参考帧）
    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (0.015f); //设置搜索半径

    rf_est.setInputCloud (model_keypoints);//模型关键点
    rf_est.setInputNormals (model_normals);//法线向量
    rf_est.setSearchSurface (model);//模型点云
    rf_est.compute (*model_rf);//计算模型参考帧

    rf_est.setInputCloud (scene_keypoints);//场景关键点
    rf_est.setInputNormals (scene_normals);//法线向量
    rf_est.setSearchSurface (scene);//场景点云
    rf_est.compute (*scene_rf);//场景参考帧

    //  聚类 聚类的方法 Clustering
   //对输入点与的聚类，以区分不同的实例的场景中的模型
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (0.01f);//霍夫空间设置每个bin的大小
    clusterer.setHoughThreshold (5.0f);//阈值
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_keypoints);//模型点云 关键点
    clusterer.setInputRf (model_rf);//模型点云参考帧
    clusterer.setSceneCloud (scene_keypoints);//场景点云关键点
    clusterer.setSceneRf (scene_rf);//场景点云参考帧
    clusterer.setModelSceneCorrespondences (model_scene_corrs);//对于组关系

    //clusterer.cluster (clustered_corrs);//辨认出聚类的对象
    clusterer.recognize (rototranslations, clustered_corrs);


    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

    // 打印 相对于输入模型的旋转矩阵与平移矩阵  rotation matrix and translation vector
    // [R t]
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);//旋转矩阵
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);//平移向量

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
    } 

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*model,output);
  output.header=input->header;
  pub.publish(output);

}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"reg");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/livox/lidar",1,cloudCallback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("reg",1);
  
  ros::spin();

  return 0;
}