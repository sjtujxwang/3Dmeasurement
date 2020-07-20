#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <math.h>
#define PI 3.14159265358979
int
main (int argc, char** argv)
{
  long double plane_coefficients[100][5]={};//dingyi weidu bu neng  shi[100][3]
  //存储模型系数
  long double in_point[100][3]={};
  //存储模型内点
  long double distance1_0=0;
  long double distance0_1=0;
  long double distance=0;
  //存储两平面之间的距离
  long double theta=0;
  //存储两平面之间的夹角
  long double fenzi=0;
  long double fenmu=0;
  long double jueduizhi=0;
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  // 申明滤波前后的点云格式
  pcl::PCDReader reader;
  reader.read(argv[1], *cloud_blob);
  //reader.read ("./ok.pcd", *cloud_blob);
  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
  //统计点云滤波个数
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;//体素栅格下采样对象
  sor.setInputCloud (cloud_blob);//原始点云
  sor.setLeafSize (0.3f, 0.3f, 0.3f);//设置采样体素大小
  sor.filter (*cloud_filtered_blob);//保存采样后的点云
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);//将采样后的点云转化为标准格式
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
  // 采样后的点云数据大小
  pcl::PCDWriter writer; //pcd读写
  writer.write<pcl::PointXYZ> ("distance_downsampled.pcd", *cloud_filtered, false);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());//创建模型系数
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // 创建存储内点的点索引集合对象inliers
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // 创建分割对象
  seg.setOptimizeCoefficients (true);
  // 可选配置，设置模型系数需要优化
  seg.setModelType (pcl::SACMODEL_PLANE);//设置模型类型 
  seg.setMethodType (pcl::SAC_RANSAC);//设置随机采样一致性方法类型
  seg.setMaxIterations (1000);//设置最大的迭代次数
  seg.setDistanceThreshold (0.1);


  // 判断是否为模型内点的距离阈值
  pcl::ExtractIndices<pcl::PointXYZ> extract;//创建点云提取对象
  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.6 * nr_points)//0.5->0.6为了处理点云包含的多个模型，在一个循环中执行该过程并在每次模型被提取后，保存剩余的点进行迭代
  {
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    // 提取内点
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
    std::stringstream ss;
    ss << "plane_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
    plane_coefficients[i][0]=coefficients->values[0];
    plane_coefficients[i][1]=coefficients->values[1];
    plane_coefficients[i][2]=coefficients->values[2];
    plane_coefficients[i][3]=coefficients->values[3];
    //创建分割目标
    in_point[i][0]=cloud_p->points[100].x;
    in_point[i][1]=cloud_p->points[100].y;
    in_point[i][2]=cloud_p->points[100].z; 
    std::cerr<<"plane "<<i<<" point:"<<"("<<in_point[i][0]<<", "<<in_point[i][1]<<", "<<in_point[i][2]<<")"<<std::endl;
    //输出模型内点
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);//cloud_f->cloud_filtered
   
    std::cerr<<"plane "<<i<<" equation :"<<plane_coefficients[i][0]<<"x+"<<plane_coefficients[i][1]<<"y+"<<plane_coefficients[i][2]<<"z+"<<plane_coefficients[i][3]<<"=0"<<std::endl;
    //std::cerr<<"plane "<<i<<"model coefficients:"<<coefficients->values[0]<<" "<<coefficients->values[1]<<" "<<coefficients->values[2]<<" "<<coefficients->values[3]<<std::endl;
    //输出平面模型的参数ax+by+cz+d=0，分别对应a、b、c、d，法向量即为（a，b，c）
    i++;
  } 
    
    writer.write<pcl::PointXYZ> ("after_plane.pcd", *cloud_filtered, false);
    distance=fabs(plane_coefficients[0][0]*in_point[1][0]+plane_coefficients[0][1]*in_point[1][1]+plane_coefficients[0][2]*in_point[1][2]+plane_coefficients[0][3])/sqrt(pow(plane_coefficients[0][0],2)+pow(plane_coefficients[0][1],2)+pow(plane_coefficients[0][2],2));
    distance1_0=fabs(plane_coefficients[0][0]*in_point[1][0]+plane_coefficients[0][1]*in_point[1][1]+plane_coefficients[0][2]*in_point[1][2]+plane_coefficients[0][3])/(sqrt(plane_coefficients[0][0]*plane_coefficients[0][0]+plane_coefficients[0][1]*plane_coefficients[0][1]+plane_coefficients[0][2]*plane_coefficients[0][2]));
   
    //计算两平面之间的距离
    theta=acos((plane_coefficients[0][0]*plane_coefficients[1][0]+plane_coefficients[0][1]*plane_coefficients[1][1]+plane_coefficients[0][2]*plane_coefficients[1][2])/(sqrt(pow(plane_coefficients[0][0],2)+pow(plane_coefficients[0][1],2)+pow(plane_coefficients[0][2],2))*sqrt(pow(plane_coefficients[1][0],2)+pow(plane_coefficients[1][1],2)+pow(plane_coefficients[1][2],2))))*(180/PI);
    //计算两平面之间的夹角
    std::cerr<<"distance between two planes is: "<<distance<<std::endl;
    std::cerr<<"distance between two planes is: "<<distance1_0<<std::endl;

    std::cerr<<"angle between two planes is: "<<theta<<std::endl;
  return (0);
}
