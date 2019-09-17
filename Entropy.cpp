#include "Entropy.h"
#include<cmath>

float Entropy(pcl::PointCloud<PointType>::Ptr cloud)
{
	// 创建存储点云重心的对象
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
	
	vector<V3> ort;
	int count[8]={0};
	
	
	for(int i=0;i<cloud->points.size();i++){
		V3 vtmp(cloud->points[i].x-centroid[0],cloud->points[i].y-centroid[1],cloud->points[i].z-centroid[2]);
		if(vtmp.x>0 && vtmp.y>0 && vtmp.z>0){
			count[0]=count[0]+1;
		}
		else if(vtmp.x<0 && vtmp.y>0 && vtmp.z>0){
			count[1]=count[1]+1;
		}
		else if(vtmp.x<0 && vtmp.y<0 && vtmp.z>0){
			count[2]=count[2]+1;
		}
		else if(vtmp.x>0 && vtmp.y<0 && vtmp.z>0){
			count[3]=count[3]+1;
		}
		else if(vtmp.x>0 && vtmp.y>0 && vtmp.z<0){
			count[4]=count[4]+1;
		}
		else if(vtmp.x<0 && vtmp.y>0 && vtmp.z<0){
			count[5]=count[5]+1;
		}
		else if(vtmp.x<0 && vtmp.y<0 && vtmp.z<0){
			count[6]=count[6]+1;
		}
		else if(vtmp.x>0 && vtmp.y<0 && vtmp.z<0){
			count[7]=count[7]+1;
		}
		
		ort.push_back(vtmp);
	}
	
	cout<<ort.size()<<endl;
	cout<<count[0]<<" "<<count[1]<<" "<<count[2]<<" "<<count[3]<<" "<<count[4]<<" "<<count[5]<<" "<<count[6]<<" "<<count[7]<<endl;
	
	
	float ety=0;	
	for(int i=0;i<8;i++)
	{
		float p=1.0*count[i]/ort.size();		
		ety+=-p*log(p);
	}
	
	cout<<ety<<endl;
	return ety;
	
}