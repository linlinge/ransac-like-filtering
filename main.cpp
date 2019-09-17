#include "global.h"
#include "ransac_like_based_filtering.h"
#include "Entropy.h"

int main(int argc, char** argv)
{
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	// load file
    if (pcl::io::loadPLYFile<PointType>(argv[1], *cloud) == -1){
   //  if (pcl::io::loadPLYFile<PointType>("/home/llg/dataset/8_Couple.ply", *cloud) == -1){
		PCL_ERROR("Couldn't read file\n");
		return (-1);
    }	  
	
	Entropy(cloud);
	
	// Normal Estimation 
	//pcl::NormalEstimation<PointType, pcl::Normal> ne;
	pcl::NormalEstimationOMP<PointType, pcl::Normal> ne;	
	ne.setInputCloud (cloud);
	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType> ());
	ne.setSearchMethod (tree);
	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);
	// Compute the features
	ne.compute(*cloud_normals);
	
	#pragma omp parallel for
	for(int i=0;i<cloud->points.size();i++){
		cloud->points[i].r=67;
		cloud->points[i].g=81;
		cloud->points[i].b=237;
	}
		
	
	//ransac_based_filtering(cloud,cloud_normals);	
	

  
	//显示设置
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    //设置背景色
    viewer->setBackgroundColor (1.0, 1.0, 1.0);

  //set multi-color for point cloud
	pcl::visualization::PointCloudColorHandlerRGBField<PointType> multi_color(cloud);

	//add the demostration point cloud data
	viewer->addPointCloud<PointType> (cloud, multi_color, "sample cloud");


    //设置点显示大小
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

    //添加需要显示的点云法向。cloud为原始点云模型，normal为法向信息，10表示需要显示法向的点云间隔，即每10个点显示一次法向，５表示法向长度。
   // viewer->addPointCloudNormals<PointType, pcl::Normal> (cloud, cloud_normals, 1, 0.02, "normals");

	
	
    //--------------------
    while (!viewer->wasStopped ()){
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }
	
	return (0);
}
