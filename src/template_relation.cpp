#include "aginika_object_georelation/template_relation.h"

using namespace std;
using namespace Eigen;
namespace aginika_object_georelation{
  TemplateRelation::TemplateRelation():term_("Template"){
    ROS_INFO("Initialize Template Relation");
  };

  bool TemplateRelation::solve(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &source,
                           pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &target,
                           pcl::PointCloud<pcl::PointXYZRGBA>::VectorType &source_voxel_vector,
                           pcl::PointCloud<pcl::PointXYZRGBA>::VectorType &target_voxel_vector){
    return false;
  }

  string TemplateRelation::getResultTerm(){
    return term_;
  }
  TemplateRelation::~TemplateRelation(){};
}
