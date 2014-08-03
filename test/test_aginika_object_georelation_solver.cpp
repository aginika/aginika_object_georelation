#include <ros/ros.h>
#include <aginika_object_georelation/aginika_object_georelation_solver.h>
#include <aginika_object_georelation/in_relation.h>
#include <aginika_object_georelation/over_relation.h>
#include <boost/shared_ptr.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace aginika_object_georelation;
int main(int argc, char* argv[]){

  ObjectRelationSolver a;
  boost::shared_ptr<InRelation> in(new InRelation());
  boost::shared_ptr<OverRelation> over(new OverRelation());

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZRGBA>);
  vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > source_vector;
  vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > target_vector;

  if(argc < 2){
    ROS_ERROR("Set the pcd filename");
    exit(-1);
  }

  if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (std::string(argv[1]), *cloud_a) == -1){
    std::cout << " Couldn't open "<< std::endl;
    return (-1);
  }
  ROS_INFO("add source_cloud");
  source_vector.push_back(cloud_a);

  if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (std::string(argv[1]), *cloud_b) == -1){
    std::cout << " Couldn't open "<< std::endl;
    return (-1);
  }
  target_vector.push_back(cloud_b);
  ROS_INFO("add target_cloud");

  a.registerObjectRelation(boost::dynamic_pointer_cast<ObjectRelation>(in));
  a.registerObjectRelation(boost::dynamic_pointer_cast<ObjectRelation>(over));
  a.setSourceCloud(target_vector);
  a.setTargetCloud(source_vector);

  ROS_INFO("setup done");
  a.solve();
  ROS_INFO("solverd");
  vector<string> resultsTerms = a.getResultTerms();
  for(vector<string>::iterator it=resultsTerms.begin(); it < resultsTerms.end(); it++)
    std::cout << *it << "  "<< endl;
}
