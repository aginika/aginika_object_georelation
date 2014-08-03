#include "aginika_object_georelation/over_relation.h"

using namespace std;
using namespace Eigen;
namespace aginika_object_georelation{
  OverRelation::OverRelation():term_("Over"), threshold_(0.2){
    ROS_INFO("Initialize Over Relation");
  };

  bool OverRelation::solve(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &source,
                           pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target,
                           pcl::PointCloud<pcl::PointXYZRGBA>::VectorType &source_voxel_vector,
                           pcl::PointCloud<pcl::PointXYZRGBA>::VectorType &target_voxel_vector){

    Vector4f source_center;
    pcl::compute3DCentroid(*source, source_center);
    Vector4f target_center;
    pcl::compute3DCentroid(*target, target_center);
    Vector3f target_to_source((source_center[0]-target_center[0]), (source_center[1]-target_center[1]), (source_center[2]-target_center[2]));
    target_to_source = target_to_source.normalized();

    //check is it about above?
    // float dot_result = target_to_source.dot(Vector3f(1,0,0));
    // ROS_INFO("DOT_RESULT x: %f", dot_result);
    // dot_result = target_to_source.dot(Vector3f(0,1,0));
    // ROS_INFO("DOT_RESULT y: %f", dot_result);

    float dot_result = target_to_source.dot(Vector3f(0,-1,0));
    // dot_result = target_to_source.dot(Vector3f(0,0,1));
    if( dot_result <   1 - threshold_)
      return false;
    ROS_INFO("DOT_RESULT y: %f", dot_result);


    for(pcl::PointCloud<pcl::PointXYZRGBA>::VectorType::iterator target_it = target_voxel_vector.begin();
        target_it < target_voxel_vector.end();
        target_it++){
      for(pcl::PointCloud<pcl::PointXYZRGBA>::VectorType::iterator source_it = source_voxel_vector.begin();
          source_it < source_voxel_vector.end();
          source_it++){
        if((*target_it).y < (*source_it).y)
          return false;
      }
    }
    return true;
  }

  string OverRelation::getResultTerm(){
    return term_;
  }
  OverRelation::~OverRelation(){};
}
