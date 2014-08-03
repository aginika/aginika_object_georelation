#include "aginika_object_georelation/aginika_object_georelation_solver.h"

using namespace std;
namespace aginika_object_georelation
{
  void ObjectRelationSolver::setSourceCloud(vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr >& source_cloud_vector)
  {
    integrateClouds(source_cloud_, source_cloud_vector);
  }
  void ObjectRelationSolver::setTargetCloud(vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr >& target_cloud_vector)
  {
    integrateClouds(target_cloud_, target_cloud_vector);
  }

  void ObjectRelationSolver::setSourceCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & source_cloud)
  {
    source_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>(*source_cloud));
  }

  void ObjectRelationSolver::setTargetCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& target_cloud)
  {
    target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>(*target_cloud));
  }


  void ObjectRelationSolver::solve()
  {
    //reset result_terms
    result_terms_.clear();

    //When source or target points are zero, revert
    if(source_cloud_->points.size() == 0 || target_cloud_->points.size() == 0)
      {
        ROS_ERROR("Source %ld target %ld are ZERO !!", source_cloud_->points.size(), target_cloud_->points.size());
        return;
      }

    //Create Voxels
    createVoxelCentroids();


    //Solce each aginika_object_georelation
    for(vector<boost::shared_ptr<ObjectRelation> >::iterator it=aginika_object_georelations_.begin(); it < aginika_object_georelations_.end(); it++)
      if((*it)->solve(source_cloud_, target_cloud_,source_voxel_centroids_, target_voxel_centroids_))
        result_terms_.push_back((*it)->getResultTerm());
  }

  void ObjectRelationSolver::calcResolution()
  {
    pcl::PointXYZRGBA min_pt, max_pt;

    pcl::getMinMax3D(*source_cloud_, min_pt, max_pt);
    double simple_estimated_source_volume = ((max_pt.x - min_pt.x)/1000) * ((max_pt.y - min_pt.y)/1000) * ((max_pt.z - min_pt.z)/1000);

    pcl::getMinMax3D(*target_cloud_, min_pt, max_pt);
    double simple_estimated_target_volume = ((max_pt.x - min_pt.x)/1000) * ((max_pt.y - min_pt.y)/1000) * ((max_pt.z - min_pt.z)/1000);
    double volume_rate = simple_estimated_target_volume / simple_estimated_source_volume;
    //    ROS_INFO("calc max segment end");

    //when source is as large as target
    if ( 0.8 < volume_rate && volume_rate < 1.2 ){
      resolution_ = cbrt(simple_estimated_source_volume) * 1000;
    }else if(volume_rate <= 0.8){
      resolution_ = cbrt(simple_estimated_target_volume) * 1000;
    }else{
      resolution_ = cbrt(simple_estimated_source_volume) * 1000;
    }
    //    ROS_INFO("resolution_ is set to %lf", resolution_);
  }

  void ObjectRelationSolver::createVoxelCentroids()
  {
    //set resolution
    calcResolution();

    //set voxel_octrees...
    source_voxel_octree_.reset(new pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGBA>(resolution_));
    source_voxel_octree_->setInputCloud(source_cloud_);
    source_voxel_octree_->addPointsFromInputCloud();
    target_voxel_octree_.reset(new pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGBA>(resolution_));
    target_voxel_octree_->setInputCloud(target_cloud_);
    target_voxel_octree_->addPointsFromInputCloud();

    source_voxel_octree_->getOccupiedVoxelCenters (source_voxel_centroids_);
    target_voxel_octree_->getOccupiedVoxelCenters (target_voxel_centroids_);

    //    ROS_INFO("source voxel leaf %ld", source_voxel_octree_->getLeafCount());
    //    ROS_INFO("target voxel leaf %ld", target_voxel_octree_->getLeafCount());

    //    ROS_INFO("voxel centroids source %ld", source_voxel_centroids_.size());
    //    ROS_INFO("voxel centroids source %ld", target_voxel_centroids_.size());


  }

  void ObjectRelationSolver::integrateClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                                             vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &source_cloud_vector)
  {
    for(vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr >::iterator it = source_cloud_vector.begin();it < source_cloud_vector.end();it++)
      *cloud += *(*it);
  }
}

