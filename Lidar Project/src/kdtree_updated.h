/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "./render/render.h"
#include<math.h>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node ,uint depth, std::vector<float> point, int id)
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

			if(point[cd] < ((*node)->point[cd]))
				insertHelper(&((*node)->left), depth+1, point, id);
			else
				insertHelper(&((*node)->right), depth+1, point, id);
		}
		
	
	}

	void insert(std:: vector<float> point, int id)
	{
		insertHelper(&root,0,point,id);
	}

	// return a list of point ids in the tree that are within distance of target
	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		// bounding box
		float x1 = target[0] + distanceTol;
		float x2 = target[0] - distanceTol;
		float y1 = target[1] + distanceTol;
		float y2 = target[1] - distanceTol;
        float z1 = target[2] + distanceTol;
		float z2 = target[2] - distanceTol;
		
		if(node!=NULL)
		{
			// check if the point is inside or outside the box. 
			float point_x1 = ((node)->point[0]); 
			float point_y1 = ((node)->point[1]);
            float point_z1 = ((node)->point[2]); 
			if ((point_x1 >= x2) && (point_x1 <= x1) && (point_y1 >= y2) && (point_y1 <= y1) && (point_z1 >= z2) && (point_z1 <= z1))
			{
				// calculate distance threshold.
				float d =  pow(pow((point_x1 - target[0]),2) + pow((point_y1 - target[1]),2) + pow((point_z1 - target[2]),2),0.5);
				if (d < distanceTol)
				{
					ids.push_back((node)->id);	
				} 
			}
			if((target[depth%3]-distanceTol)< node->point[depth%3])
				searchHelper(target,node->left,depth+1,distanceTol,ids);			
			if((target[depth%3]+distanceTol)>node->point[depth%3])
				searchHelper(target,node->right,depth+1,distanceTol,ids);
		}	
		
	}
	
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}

};




