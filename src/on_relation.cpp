#include "aginika_object_georelation/on_relation.h"

using namespace std;
using namespace Eigen;
namespace aginika_object_georelation{
  OnRelation::OnRelation():term_("On"), MIN_LENGTH(0.1), resolution_(0.1){
    ROS_INFO("Initialize On Relation");
  };

  bool OnRelation::solve(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &source,
                           pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target,
                           pcl::PointCloud<pcl::PointXYZRGBA>::VectorType &source_voxel_vector,
                           pcl::PointCloud<pcl::PointXYZRGBA>::VectorType &target_voxel_vector)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr larger_cloud = source;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr smaller_cloud = target;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA> search_octree (resolution_);
    bool source_gt_target = (source->points.size() > target->points.size());
    if(source_gt_target){
      larger_cloud = target;
      smaller_cloud = source;
    }

    search_octree.setInputCloud (larger_cloud);
    search_octree.addPointsFromInputCloud ();

    int result_index;
    float sqr_distance;

    //search the nearest point
    for (int i = 0;i < smaller_cloud->points.size(); i++){
      search_octree.approxNearestSearch(smaller_cloud->points[i], result_index, sqr_distance);
      sqr_distance = sqrt(sqr_distance);
      if (sqr_distance < MIN_LENGTH){
        ROS_INFO("tmp sqr_distance %f", sqr_distance);
        return true;
      }
    }
    return false;


    ////Method 1
    // float min_length = std::numeric_limits<float>::max();
    // for(pcl::PointCloud<pcl::PointXYZRGBA>::VectorType::iterator target_it = target_voxel_vector.begin();
    //     target_it < target_voxel_vector.end();
    //     target_it++){
    //   for(pcl::PointCloud<pcl::PointXYZRGBA>::VectorType::iterator source_it = source_voxel_vector.begin();
    //       source_it < source_voxel_vector.end();
    //       source_it++){
    //     Vector3f v((*target_it).x - (*source_it).x, (*target_it).y - (*source_it).y, (*target_it).z - (*source_it).z);
    //     float tmp_length = v.norm();
    //     ROS_INFO("tmp_length : %f", tmp_length);
    //     if(min_length > tmp_length)
    //       min_length = tmp_length;
    //     if(min_length < MIN_LENGTH)
    //       return true;
    //   }
    // }
    // return false;
  }

  string OnRelation::getResultTerm(){
    return term_;
  }
  OnRelation::~OnRelation(){};
}
