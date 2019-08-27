#include "ransac_like_based_filtering.h"
#include <cstdlib>

void ransac_based_filtering(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normls)
{
	int K=20;
	vector<int> pointIdxNKNSearch(K);
	vector<float> pointNKNSquaredDistance(K);
	pcl::search::KdTree<PointType> kdtree(new pcl::search::KdTree<PointType>());
	kdtree.setInputCloud(cloud);	
	vector<int> outlier_idx_set;
	
	#pragma omp parallel for
	for(int i=0;i<cloud->points.size();i++){
		if (kdtree.nearestKSearch(cloud->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){ // Get all K-nearest Neighbours
						
			vector<int> outlier_idx;
			vector<int> outlier_idx_global;
			vector<int> outlier_idx_local;
			vector<int> inlier_idx_local;
			int min_outlier_count=INT_MAX;
			
			// ransac processing progress			
			for(int j=0;j<40;j++)
			{
				int index01,index02,index03;
				
				index01=index02=index03=INT_MAX;
				index01= pointIdxNKNSearch[rand()%K];
				do{ index02= pointIdxNKNSearch[rand()%K]; }while(index02==index01);
				do{ index03= pointIdxNKNSearch[rand()%K]; }while(index03==index01 || index03==index02);
				
				outlier_idx_local.clear();
				inlier_idx_local.clear();
				
				V3 sample01(cloud->points[index01].x,cloud->points[index01].y,cloud->points[index01].z);
				V3 sample02(cloud->points[index02].x,cloud->points[index02].y,cloud->points[index02].z);
				V3 sample03(cloud->points[index03].x,cloud->points[index03].y,cloud->points[index03].z);				
				Plane plane1(sample01,sample02,sample03);				
				
				float max_dist=0;
				vector<float> dist_set;
				for(int a=0;a<K;a++)
				{
					int itmp=pointIdxNKNSearch[a];
					if(itmp!=index01 && itmp!=index02 && itmp!=index03){
						float dist=plane1.Distance(V3(cloud->points[itmp]));
						dist_set.push_back(dist);
						// cout<<"dist: "<<dist<<endl;
						
						max_dist = max_dist > dist ? max_dist:dist;						
						
						/* 
						if(dist>0.01){
							// record outlier indices
							outlier_idx_local.push_back(itmp);
						} 
						*/
					}
				}
				
				for(int a=0;a<K;a++)
				{
					if(max_dist> 5* pointNKNSquaredDistance[0] && dist_set[a]>0.99*max_dist)
					{
						outlier_idx_local.push_back(pointIdxNKNSearch[a]);
					}
				}				
				
				// update global outlier				
				if(min_outlier_count > outlier_idx_local.size()){
					// update the record of number of outlier 
					min_outlier_count=outlier_idx_local.size();
					// update global outlier
					outlier_idx_global.clear();
					outlier_idx_global.insert(outlier_idx_global.end(),outlier_idx_local.begin(),outlier_idx_local.end());
				}
			}

			// gather all outliers 
			outlier_idx_set.insert(outlier_idx_set.end(),outlier_idx_global.begin(),outlier_idx_global.end());			
		}
		else{
			cout<<"Failed to find k-nearest neighbour!"<<endl;
		}
	}

	// remove outlier
	cout<<"unique size before:"<<outlier_idx_set.size()<<endl;
	sort(outlier_idx_set.begin(),outlier_idx_set.end());
	outlier_idx_set.erase(unique(outlier_idx_set.begin(),outlier_idx_set.end()),outlier_idx_set.end());
	cout<<"unique size after:"<<outlier_idx_set.size()<<endl;
	
	
	
	#pragma omp parallel for
	for(int i=outlier_idx_set.size()-1;i>=0;i--)
	{
		// cloud->erase(cloud->begin()+outlier_idx_set[i]);
		cloud->points[outlier_idx_set[i]].r=255;
		cloud->points[outlier_idx_set[i]].g=0;
		cloud->points[outlier_idx_set[i]].b=0;
	}
	
	
}