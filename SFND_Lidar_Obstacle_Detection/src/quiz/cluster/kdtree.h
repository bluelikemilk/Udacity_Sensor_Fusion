/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

	void insertHelper(Node** root, int depth, std::vector<float> point, int id){
		// the function should create a new node and place correctly with in the root 
		if(*root==NULL){ // if the tree is empty/reach the leaf, insert the root node
	        *root = new Node(point, id);
			return;
		} else { // else, find the proper child tree and do recursion
			int idx = depth%2; // determine which axis to compare with
			// determine which child tree to go
			if(point[idx]<(*root)->point[idx]) insertHelper(&((*root)->left), depth+1, point, id); // move to left tree
			else insertHelper(&((*root)->right), depth+1, point, id); // move to right tree
		}
	}

	void insert(std::vector<float> point, int id){
		// TODO: Fill in this function to insert a new point into the tree
		// recursively call helper function to find the proper position
		insertHelper(&root, 0, point, id); // pass the address of the root(root itself is a pointer, here pass the adress of the pointer)
		return;
	}

	
	void searchHelper(Node* root, std::vector<int> &ids, int depth, std::vector<float> target, float distanceTol){
		if(root == NULL) return;

		if(root->point[0]>=(target[0] - distanceTol) && root->point[0]<=(target[0] + distanceTol) && root->point[1]>=(target[1] - distanceTol) && root->point[1]<=(target[1] + distanceTol)){
			float distance = sqrt((root->point[0]-target[0])*(root->point[0]-target[0]) + (root->point[1]-target[1])*(root->point[1]-target[1]));
			if(distance <= distanceTol) ids.push_back(root->id);
		}

		// check across boundary by comparing current point with the min/max boundary around target
		// do not compare target with current node, compare the boundary of target with current node
		if((target[depth%2]-distanceTol)<root->point[depth%2]) searchHelper(root->left, ids, depth+1, target, distanceTol);
		if((target[depth%2]+distanceTol)>root->point[depth%2]) searchHelper(root->right, ids, depth+1, target, distanceTol);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, ids, 0, target, distanceTol);
		return ids;
	}
	

};




