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

#ifndef __OBJECT_RELATION_H__
#define __OBJECT_RELATION_H__

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <string>

using namespace std;
using namespace Eigen;
namespace aginika_object_georelation
{
  class ObjectRelation{
  public:
    const string term_;
    Matrix3f world_matrix_;
    typedef boost::shared_ptr<ObjectRelation> Ptr;
    typedef boost::shared_ptr<const ObjectRelation> ConstPtr;

  public:
    ObjectRelation():term_("None"){world_matrix_=Matrix3f::Identity();};
    virtual bool solve(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &source,
                       pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target,
                       pcl::PointCloud<pcl::PointXYZRGBA>::VectorType &source_voxel_vector,
                       pcl::PointCloud<pcl::PointXYZRGBA>::VectorType &target_voxel_vector){return false;};
    virtual string getResultTerm(){return term_;};
    virtual void setWorldMatrix(Matrix3f &world_matrix){world_matrix_=world_matrix;};
    virtual ~ObjectRelation(){};
  };
}
#endif
