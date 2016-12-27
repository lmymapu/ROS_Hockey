
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "world_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(50.0);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(0.5, 1.5, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/world"));
    rate.sleep();
  }
  return 0;
};
