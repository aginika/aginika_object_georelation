#include "aginika_object_georelation/in_relation.h"

using namespace std;
using namespace Eigen;
namespace aginika_object_georelation{
  InRelation::InRelation():term_("In"),threshold_(0.2){
    ROS_INFO("Initialize In Relation");
  };

  //If there are three kinds of vector, solve will return true
  //1.0degree
  //2.90degree
  //3.180degree
  bool InRelation::solve(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &source,
                         pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target,
                         pcl::PointCloud<pcl::PointXYZRGBA>::VectorType &source_voxel_vector,
                         pcl::PointCloud<pcl::PointXYZRGBA>::VectorType &target_voxel_vector){

    for(pcl::PointCloud<pcl::PointXYZRGBA>::VectorType::iterator target_it = target_voxel_vector.begin();
        target_it < target_voxel_vector.end();
        target_it++){
      for(pcl::PointCloud<pcl::PointXYZRGBA>::VectorType::iterator source_it = source_voxel_vector.begin();
          source_it < source_voxel_vector.end();
          source_it++){
        Vector3f v1((*target_it).x - (*source_it).x, (*target_it).y - (*source_it).y, (*target_it).z - (*source_it).z);
        v1 = v1.normalized();
        bool right_angle = false;
        bool straight_angle = false;

        for(pcl::PointCloud<pcl::PointXYZRGBA>::VectorType::iterator source_it2 = source_voxel_vector.begin();
            source_it2 < source_voxel_vector.end();
            source_it2++){

          Vector3f v2((*target_it).x - (*source_it2).x, (*target_it).y - (*source_it2).y, (*target_it).z - (*source_it2).z);
          v2 = v2.normalized();
          float result = v1.dot(v2);


          if (result < -1 + threshold_){
            straight_angle = true;
            //            ROS_INFO("straight dot value : %f", result);
          }
          else if( 0 + threshold_ > result  && result > 0 - threshold_){
            right_angle = true;
            //            ROS_INFO("right dot value : %f", result);
          }

          if (right_angle && straight_angle)
            return true;
        }
      }
    }
    return false;
  }

  string InRelation::getResultTerm(){
    return term_;
  }
  InRelation::~InRelation(){};
}
