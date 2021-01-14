/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "./render/render.h"
#include<math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point2;
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node ,uint depth, pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if (*node == NULL)
		{
			*node = new Node(point,id);
		}		
		else
		{
			uint cd =depth % 3;
			
			float treePoint;
			float insertPoint;
			
			if (cd == 0) 
			 {
				insertPoint = point.x; 
				treePoint = (*node)->point.x;
			 }
			else if (cd == 1) 
			 { 
				insertPoint = point.y;
				treePoint = (*node)->point.y;
			 }
			else if (cd == 2)
			{
				insertPoint = point.z;
				treePoint = (*node)->point.z;
			}
			if (insertPoint < treePoint)
				insertHelper(&((*node)->left),depth +1, point, id);
			else
				insertHelper(&((*node)->right),depth +1, point, id);
			// if(point[cd] < ((*node)->point[cd]))
			// 	insertHelper(&((*node)->left), depth+1, point, id);
			// else
			// 	insertHelper(&((*node)->right), depth+1, point, id);
		}
		
	
	}

	void insert(pcl::PointXYZI point, int id)
	{
		insertHelper(&root,0,point,id);
	}

	// return a list of point ids in the tree that are within distance of target
	void searchHelper(pcl::PointXYZI target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		//Change to target.x target.y, target.z
		// bounding box
		float x1 = target.x + distanceTol;
		float x2 = target.x - distanceTol;
		float y1 = target.y + distanceTol;
		float y2 = target.y - distanceTol;
        float z1 = target.z + distanceTol;
		float z2 = target.z - distanceTol;
        
		
		if(node!=NULL)
		{
			// check if the point is inside or outside the box. 
			float point_x1 = ((node)->point.x); 
			float point_y1 = ((node)->point.y);
            float point_z1 = ((node)->point.z); 
			if ((point_x1 >= x2) && (point_x1 <= x1) && (point_y1 >= y2) && (point_y1 <= y1)  && (point_z1 >= z2) && (point_z1 <= z1))
			{
				// calculate distance threshold.
				float d =  pow(pow((point_x1 - target.x),2) + pow((point_y1 - target.y),2) + pow((point_z1 - target.z),2),0.5);
				if (d < distanceTol)
				{
					ids.push_back((node)->id);	
				} 
			}
			float targetPoint, treePoint;

			if (depth%3 == 0) 
			 {
				targetPoint = target.x; 
				treePoint = (*node).point.x;
			 }
			else if (depth%3 == 1) 
			 { 
				targetPoint = target.y;
				treePoint = (*node).point.y;
			 }
			else if (depth%3 == 2)
			{
				targetPoint = target.z;
				treePoint = (*node).point.z;
			}

			if((targetPoint-distanceTol)< treePoint)
				searchHelper(target,node->left,depth+1,distanceTol,ids);			
			if((targetPoint+distanceTol)> treePoint)
				searchHelper(target,node->right,depth+1,distanceTol,ids);
		}	
		
	}
	
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}

};


















