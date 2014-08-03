// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Yuto Inagaki and JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef __OBJECT_RELATION_SOLVER_H__
#define __OBJECT_RELATION_SOLVER_H__

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h> 
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <string>
#include <vector>
#include <cmath>
#include "aginika_object_georelation/aginika_object_georelation.h"

using namespace std;
namespace aginika_object_georelation
{
  class ObjectRelationSolver{
  private:
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_cloud_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_cloud_;
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGBA>::Ptr source_voxel_octree_;
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGBA>::Ptr target_voxel_octree_;
    pcl::PointCloud<pcl::PointXYZRGBA>::VectorType source_voxel_centroids_;
    pcl::PointCloud<pcl::PointXYZRGBA>::VectorType target_voxel_centroids_;

    vector<boost::shared_ptr<ObjectRelation> > aginika_object_georelations_;
    vector<string> result_terms_;

    double resolution_;
  public:
    ObjectRelationSolver():source_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>),
                                               target_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>),
                                               source_voxel_octree_(new pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGBA>(0.1)),
                                               target_voxel_octree_(new pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGBA>(0.1)),
                                               result_terms_()
  {
    ROS_INFO("aginika_object_georelation_solver initialize");
  }
    void setSourceCloud(vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr >& source_cloud_vector);
    void setTargetCloud(vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr >& target_cloud_vector);
    void setSourceCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& source_cloud);
    void setTargetCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& target_cloud);
    void solve();
    void registerObjectRelation(const boost::shared_ptr<ObjectRelation> &obj_relation){aginika_object_georelations_.push_back(obj_relation);}
    vector<string> getResultTerms(){return result_terms_;};
  private:
    void createVoxelCentroids();
    void integrateClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud ,vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &source_cloud_vector);
    void calcResolution();
  };
}
#endif
