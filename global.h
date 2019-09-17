#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <omp.h>
#include <Eigen/Dense> 
#include "V3.hpp"
using namespace std;
typedef pcl::PointXYZRGBA PointType;