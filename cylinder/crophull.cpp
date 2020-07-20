#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	reader.read(argv[1],*cloud);
        
    //creat a 2D cloudpoint for luozhu
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        boundingbox_ptr->push_back(pcl::PointXYZ(-7, 21, -10));//6->0
	boundingbox_ptr->push_back(pcl::PointXYZ(-27, 16, 15));
	boundingbox_ptr->push_back(pcl::PointXYZ(-25, -8, 15));
	boundingbox_ptr->push_back(pcl::PointXYZ(-5, -8, -9));
	
	//creat a 2D cloudpoint for luokon
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_kon (new pcl::PointCloud<pcl::PointXYZ>);

	boundingbox_kon->push_back(pcl::PointXYZ(17, -16, 46));//
	boundingbox_kon->push_back(pcl::PointXYZ(12, 15, 48));
	boundingbox_kon->push_back(pcl::PointXYZ(33, -12, 26));
	boundingbox_kon->push_back(pcl::PointXYZ(27, 15, 31));
	
	pcl::ConvexHull<pcl::PointXYZ> hull;//creat an object of 2D convex hull from 2D cloudpoints
	hull.setInputCloud(boundingbox_ptr);//set the input cloud of convex hull
	hull.setDimension(2);//set the dimension of crophull is 2D
	std::vector<pcl::Vertices> polygons;//creat a pcl::Vertices vector to save vertexs of boundingbox
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZ>);//creat a pointcloud to descripe the shape of crophull
	hull.reconstruct(*surface_hull, polygons);//to calculate 2D convex hull results

	pcl::ConvexHull<pcl::PointXYZ> hull1;//creat an object of 2D convex hull from 2D cloudpoints
	hull1.setInputCloud(boundingbox_kon);//set the input cloud of convex hull
	hull1.setDimension(2);
	std::vector<pcl::Vertices> polygons1;//creat a pcl::Vertices vector to save vertexs of boundingbox
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull1 (new pcl::PointCloud<pcl::PointXYZ>);//creat a pointcloud to descripe the shape of crophull
	hull1.reconstruct(*surface_hull1, polygons1);//to calculate 2D convex hull results-cloudpoints & vector

	pcl::PointCloud<pcl::PointXYZ>::Ptr luozhu (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::CropHull<pcl::PointXYZ> bb_filter;//creat an object of crophull
	bb_filter.setDim(2);
	bb_filter.setInputCloud(cloud);
	bb_filter.setHullIndices(polygons);//set the input convex hull vector
	bb_filter.setHullCloud(surface_hull);//set the input convex hull cloud
	bb_filter.filter(*luozhu);
	std::cout << "luozhu pointclouds has  " << luozhu->size() << "points " << std::endl;
    writer.write<pcl::PointXYZ> ("luozhu.pcd", *luozhu, false);

	pcl::PointCloud<pcl::PointXYZ>::Ptr luokon (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::CropHull<pcl::PointXYZ> bb_filter1;//creat an object of crophull
	bb_filter1.setDim(2);
	bb_filter1.setInputCloud(cloud);
	bb_filter1.setHullIndices(polygons1);//set the input convex hull vector
	bb_filter1.setHullCloud(surface_hull1);//set the input convex hull cloud
	bb_filter1.filter(*luokon);
	std::cout << "luokon pointclouds has  " << luokon->size() << "points " << std::endl;
    writer.write<pcl::PointXYZ> ("luokon.pcd", *luokon, false);

	//visualize
	boost::shared_ptr<pcl::visualization::PCLVisualizer> for_visualizer_v (new pcl::visualization::PCLVisualizer ("crophull display"));
	for_visualizer_v->setBackgroundColor(0, 0, 1);

	int v1(0);
	for_visualizer_v->createViewPort (0.0, 0.0, 0.25, 1, v1);
	for_visualizer_v->setBackgroundColor (255, 255, 255, v1);
	for_visualizer_v->addPointCloud (cloud,"cloud",v1);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"cloud");
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud");
	for_visualizer_v->addPolygon<pcl::PointXYZ>(surface_hull,0,.069*255,0.2*255,"backview_hull_polyline",v1);
	for_visualizer_v->addPolygon<pcl::PointXYZ>(surface_hull1,0,.069*255,0.2*255,"backview_hull_polyline1",v1);

	int v2(0);
	for_visualizer_v->createViewPort (0.25, 0.0, 0.5, 1, v2);	
	for_visualizer_v->setBackgroundColor (255, 255, 255, v2);
	for_visualizer_v->addPointCloud (surface_hull,"surface_hull",v2);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"surface_hull");
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,8,"surface_hull");
	for_visualizer_v->addPolygon<pcl::PointXYZ>(surface_hull,0,.069*255,0.2*255,"backview_hull_polyline",v2);
   
	int v3(0);
	for_visualizer_v->createViewPort (0.5, 0.0, 0.75, 1, v3);
	for_visualizer_v->setBackgroundColor (255, 255, 255, v3);
	for_visualizer_v->addPointCloud (luozhu,"luozhu",v3);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"luozhu");
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"luozhu");

	int v4(0);
	for_visualizer_v->createViewPort (0.75, 0.0, 1.0, 1, v4);
	for_visualizer_v->setBackgroundColor (255, 255, 255, v4);
	for_visualizer_v->addPointCloud (luokon,"luokon",v4);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"luokon");
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"luokon");

	while (!for_visualizer_v->wasStopped())
	{
		for_visualizer_v->spinOnce(1000);
	}
	system("pause");
}
