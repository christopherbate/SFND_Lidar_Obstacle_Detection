/* \author Aaron Brown */
// Revised by Christopher Bate for SFND Project
#pragma once
#include <queue>

#include "types.h"

// Structure to represent node of kd tree
template <typename PointT>
struct Node
{
	PointT point;
	int id;
	int component;
	Node<PointT> *left;
	Node<PointT> *right;
	Node(PointT const& pt, int setId)
		: point(pt), id(setId), left(nullptr), right(nullptr)
	{
	}
};

template <typename PointT>
struct KdTree
{
	Node<PointT> *root;
	int dimensions;
	std::vector<bool> _processed;

	KdTree()
		: root(nullptr), dimensions(2)
	{
	}

	void setInputCloud(CloudPtr<PointT> const& input)
	{
		for (auto idx = 0; idx < input->size(); idx++)
		{
			this->insert(input->at(idx), idx);
		}
		_processed = std::vector<bool>(input->size(), false);
	}

	void insert(PointT const& point, int const id)
	{
		int component = 0;
		Node<PointT> **currNode = &root;
		Node<PointT> *new_node = new Node<PointT>(point, id);

		// Increment compponent
		while (*currNode != nullptr)
		{
			bool comp = false;
			switch (component)
			{
			case 0:
				comp = new_node->point.x < (*currNode)->point.x;
				break;
			case 1:
				comp = new_node->point.y < (*currNode)->point.y;
				break;
			case 2:
				comp = new_node->point.z < (*currNode)->point.z;
				break;
			default:
				throw std::runtime_error("component out of bounds");
			}
			if (comp)
			{
				currNode = &(*currNode)->left;
			}
			else
			{
				currNode = &(*currNode)->right;
			}
			component = (component + 1) % dimensions;
		}
		new_node->component = component;
		*currNode = new_node;
	}

	void proximity(int idx, const CloudPtr<PointT> &cloud, std::vector<int> &clusterIdx, float tol)
	{
		_processed[idx] = true;
		clusterIdx.push_back(idx);
		auto found = this->search(cloud->at(idx), tol);
		for (auto nidx : found)
		{
			if (!_processed[nidx])
			{
				proximity(nidx, cloud, clusterIdx, tol);
			}
		}
	}

	CloudPtrVec<PointT> euclideanCluster(const CloudPtr<PointT> &cloud, float tol, int minSize, int maxSize)
	{
		CloudPtrVec<PointT> clusters;
		for (auto idx = 0; idx < cloud->size(); idx++)
		{
			if (!_processed[idx])
			{
				CloudPtr<PointT> cluster(new pcl::PointCloud<PointT>);
				std::vector<int> clusterIdx;
				this->proximity(idx, cloud, clusterIdx, tol);

				if(clusterIdx.size() >= minSize && clusterIdx.size() <= maxSize){
					for(auto pidx : clusterIdx){
						cluster->push_back(cloud->at(pidx));
					}					
				}
				clusters.push_back(cluster);
			}
		}
		return clusters;
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::queue<Node<PointT> *> searchList;
		searchList.push(root);
		std::vector<int> ids;
		float const sqTol = distanceTol * distanceTol;

		while (searchList.size() > 0)
		{
			auto currNode = searchList.front();
			searchList.pop();

			std::vector<float> dist(dimensions);
			dist[0] = std::abs(currNode->point.x - target.x);
			dist[1] = std::abs(currNode->point.y - target.y);
			dist[2] = std::abs(currNode->point.z - target.z);
			if (dist[0] < distanceTol && dist[1] < distanceTol && dist[2] < distanceTol)
			{
				float trueDistSq = (dist[0] * dist[0]) + (dist[1] * dist[1]) + (dist[2] * dist[2]);
				if (trueDistSq < sqTol)
				{
					ids.push_back(currNode->id);
				}
			}

			// Perform comparison on the relevant dimension.
			bool left = false;
			switch (currNode->component)
			{
			case 0:
				left = target.x < currNode->point.x;
				break;
			case 1:
				left = target.y < currNode->point.y;
				break;
			case 2:
				left = target.z < currNode->point.z;
				break;
			default:
				throw std::runtime_error("component out of bounds");
			}

			// Add the side the node is on
			Node<PointT> *oppSide = nullptr;
			if (left)
			{
				oppSide = currNode->right;
				if (currNode->left != nullptr)
					searchList.push(currNode->left);
			}
			else
			{
				oppSide = currNode->left;
				if (currNode->right != nullptr)
					searchList.push(currNode->right);
			}
			if (dist[currNode->component] < distanceTol && oppSide != nullptr)
				searchList.push(oppSide);
		}
		return ids;
	}
};
