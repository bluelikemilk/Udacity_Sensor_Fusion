/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <vector>
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));


	// Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    

    
	
	// TODO: Fill in this function
	// For max iterations 
	while(maxIterations--){
		// Randomly sample subset and fit plane
		// select 3 random points
		std::unordered_set<int> curInliers;
		while(curInliers.size()<3) curInliers.insert(rand() % cloud->size());
		// take all starting points
		auto iter = curInliers.begin();
		pcl::PointXYZ p1 = cloud->points[*(iter++)];
		float x1=p1.x, y1=p1.y, z1=p1.z;
		pcl::PointXYZ p2 = cloud->points[*(iter++)];
		float x2=p2.x, y2=p2.y, z2=p2.z;
		pcl::PointXYZ p3 = cloud->points[*(iter++)];
		float x3=p3.x, y3=p3.y, z3=p3.z;

		// fit the plane using p1-p3
		// v1 = < x2 - x1, y2 - y1, z2 - z1 >, v2 = < x3 - x1, y3 - y1, z3 - z1 >
		// Find normal vector to the plane by taking cross product of v1 \times v2:
		// v1 \times v2 = <(y2-y1)(z3-z1)-(z2-z1)(y3-y1),
		// (z2-z1)(x3-x1)-(x2-x1)(z3-z1),
		// (x2-x1)(y3-y1)-(y2-y1)(x3-x1)> = <i, j, k>
		double i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		double j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		double k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		// A = i, B = j, C = k, D = -( ix1 + jy1 + kz1 )
		double A=i, B=j, C=k, D=-(i*x1 + j*y1 + k*z1);

		// Measure distance between every point and fitted line
		for(int idx=0; idx<cloud->size(); idx++){
			// if already in inlier list, skip
			if(curInliers.count(idx)) continue;

			//Distance d = d = |Ax+By+C*z+D|/sqrt(A^2+B^2+C^2).
			pcl::PointXYZ p = cloud->points[idx];
			double d = fabs(A*p.x + B*p.y + C*p.z + D)/sqrt(A*A + B*B + C*C); // fabs always returns a float, abs may return int based in input type
			// If distance is smaller than threshold count it as inlier
			if(d<=distanceTol)	curInliers.insert(idx);
		}
		// Return indicies of inliers from fitted line with most inliers
		if(curInliers.size()>inliersResult.size()){
			inliersResult = curInliers;
		} 
	}
	
	// calculate running time
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "My implementation of plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
