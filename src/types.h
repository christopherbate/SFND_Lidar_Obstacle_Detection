/**
 * Declares common template types
 * to make dealing with the PCL templates easier
 */
#pragma once

#include <pcl/common/common.h>

template <typename PointType>
using CloudPtr = typename pcl::PointCloud<PointType>::Ptr;

template <typename PointType>
using CloudPtrVec = std::vector<CloudPtr<PointType>>;

template <typename PointType>
using Cloud = typename pcl::PointCloud<PointType>;

template <typename PointType>
using CloudPtrPair = std::pair<CloudPtr<PointType>, CloudPtr<PointType>>;
