// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT> ());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point:indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    return cloudRegion;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3D(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	while (maxIterations--)
	{
		// Randomly pick two points

		std:: unordered_set<int> inliers;
		while(inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));
		 
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
        itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

		float a = (y2-y1)*(z3-z1) - (z2 -z1)*(y3-y1);
		float b = (z2-z1)*(x3-x1) - (x2 -x1)*(z3-z1);
		float c = (x2-x1)*(y3-y1) - (y2 -y1)*(x3-x1);
        float d = -(a*x1 + b*y1 + c*z1);
		for(int index = 0; index< cloud->points.size(); index++)
		{
			if(inliers.count(index)>0)
				continue;
			pcl::PointXYZI point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
            float z4 = point.z;

			float dist = fabs(a*x4+b*y4+c*z4 + d)/sqrt(a*a + b*b + c*c); // fabs is the absolute value of the floating value...
			if(dist <= distanceTol)
				inliers.insert(index);

		}
		if(inliers.size()>inliersResult.size())
		{
			inliersResult = inliers;
		}

	}
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout<<"Ransac took "<< elapsedTime.count() << "milliseconds"<<std::endl; 
	
	return inliersResult;

}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false); // setNegative to false will extract the inliers. If you setNegative to true then it extracts all but the inliers.
    // Get the points associated with the planar surface
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>());
    extract.filter(*cloud_plane);
    std::cout<<"PointCloud representing the planar component: "<<cloud_plane->size()<<"data points."<<std::endl;
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obs(new pcl::PointCloud<pcl::PointXYZI>());
    extract.filter(*cloud_obs);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obs, cloud_plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	// pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // // TODO:: Fill in this function to find inliers for the cloud.
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    
    // // Create Segmentation object
    // pcl::SACSegmentation<pcl::PointXYZI> seg; //object of a library class that will perform segmentation. We have specified RANSAC as the algorithm to do Segmentation.
    // seg.setOptimizeCoefficients (true);
    // // Mandatory
    // seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setDistanceThreshold(distanceThreshold);
    // seg.setMaxIterations(maxIterations);
    // seg.setInputCloud(cloud);
    // seg.segment(*inliers, *coefficients);
    // // call RANSAC3D function here
    
	std::unordered_set<int> inliers = Ransac3D(cloud, maxIterations, distanceThreshold);
	pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZI point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    
    
    
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (clusterTolerance); // 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    
    // creating point cloud for each cluster from indices.
    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    // {
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    //     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    //         cloud_cluster->push_back ((*cloud)[*pit]); //*
    //     cloud_cluster->width = cloud_cluster->size ();
    //     cloud_cluster->height = 1;
    //     cloud_cluster->is_dense = true;
    //     clusters.push_back (cloud_cluster);
    // }

    KdTree* mytree = new KdTree;
    //initialize kd tree
    initializeKdTree(cloud, mytree);
    //std::cout << "initialized tree "<< std::endl;
    std::vector<std::vector<int>> clustersIndicies = euclideanCluster(cloud, mytree, clusterTolerance);
    std::cout << "clusters found: "<<clustersIndicies.size()<< std::endl;
    for(std::vector<int> cluster : clustersIndicies)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(pcl::PointXYZI(cloud->points[indice]));
        clusterCloud->width = clusterCloud->size ();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;
        clusters.push_back (clusterCloud);
    }
  	


    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool> processed, KdTree* tree, float distanceTol)
{
	processed[indice] = true;
	cluster.push_back(indice);
    //Create a vector of poin using x, y and z value from cloud->points[indice].x, cloud->points[indice].y, cloud->points[indice].z
	std::vector<float> point;
    point.push_back(cloud->points[indice].x);
    point.push_back(cloud->points[indice].y);
    point.push_back(cloud->points[indice].z);

    //std::vector<int> nearest = tree->search(cloud->points[indice],distanceTol);
    std::vector<int> nearest = tree->search(point,distanceTol);
	for(int id:nearest)
	{
        //std::cout << "nearest id : " <<id << "is processed: " << processed[id] << std::endl;
		if(!processed[id])
			clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
        else
        {
           // std::cout<< "exit for nearest ID:"<<id<<std::endl;
        }
            
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(cloud->points.size(), false);  
	int i = 0;
    //std::cout << cloud->points.size() << std::endl;
	while(i < cloud->points.size())
	{
        //std::cout << i << std::endl;
		if(processed[i])
		{
			i++;
			continue;
		}
		std::vector<int> cluster;
		clusterHelper(i, cloud, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;
        
	}
	return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::initializeKdTree(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree)
{
    
  
    for (int i=0; i<cloud->points.size(); i++) 
    {
        std::vector<float> point;
        point.push_back(cloud->points[i].x);
        point.push_back(cloud->points[i].y);
        point.push_back(cloud->points[i].z);
        //Create a vector of poin using x, y and z value from cloud->points[i].x, cloud->points[i].y, cloud->points[i].z
        //tree->insert(cloud->points[i],i);
        tree->insert(point,i);
    }
      
}