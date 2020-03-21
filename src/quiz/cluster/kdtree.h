/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <queue>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(nullptr), right(nullptr)
	{
	}
};

struct KdTree
{
	Node *root;
	int dimensions;

	KdTree()
		: root(nullptr), dimensions(2)
	{
	}

	void insert(std::vector<float> point, int id)
	{
		if (point.size() < dimensions)
		{
			throw std::runtime_error("given point has fewer than required dimensiosn");
		}

		Node *new_node = new Node(point, id);
		int component = 0;
		Node **currNode = &root;

		// Increment compponent
		while (*currNode != nullptr)
		{
			if (new_node->point[component] < (*currNode)->point[component])
			{				
				currNode = &(*currNode)->left;
			}
			else
			{				
				currNode = &(*currNode)->right;
			}
			component = (component + 1) % dimensions;
		}		
		*currNode = new_node;
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::queue<std::pair<Node *, int>> searchList;
		searchList.push(std::make_pair(root, 0));
		std::vector<int> ids;
		float distsq = distanceTol * distanceTol;

		std::cout << "Query Point: " << target[0] << ", " << target[1] << std::endl;

		while (searchList.size() > 0)
		{
			auto next = searchList.front();
			auto component = next.second;
			auto currNode = next.first;
			searchList.pop();

			std::vector<float> dist(dimensions);
			for (auto di = 0; di < dimensions; di++)
			{
				dist[di] = currNode->point[di] - target[di];
			}

			std::cout << "Compare Point: " << currNode->point[0] << ", " << currNode->point[1] << std::endl;

			// Case 1: Point is square
			if (std::abs(dist[0]) < distanceTol && std::abs(dist[1]) < distanceTol)
			{
				// Finer check
				if (dist[0] * dist[0] + dist[1] * dist[1] < distsq)
				{
					ids.push_back(currNode->id);				
				}
			}

			// Add the side the node is on
			Node *oppSide = nullptr;
			if (target[component] < currNode->point[component])
			{
				oppSide = currNode->right;
				if (currNode->left != nullptr)
				{

					searchList.push(std::make_pair(currNode->left,
												   (component + 1) % dimensions));
				}
			}
			else
			{
				oppSide = currNode->left;
				if (currNode->right != nullptr)
					searchList.push(std::make_pair(currNode->right,
												   (component + 1) % dimensions));
			}

			// Add the other side if required
			if (dist[component] < distanceTol)
			{
				if (oppSide != nullptr)
				{
					searchList.push(std::make_pair(oppSide,
												   (component + 1) % dimensions));
				}
			}
		}
		return ids;
	}
	
};
