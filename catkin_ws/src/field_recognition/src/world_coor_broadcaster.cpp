
#include <ros/ros.h>
#include "field_recognition/convert_frames.h"
#include <tf/transform_broadcaster.h>

double offset_x(0), offset_y(0);
double rotate_theta(0);
bool isTrafoAvailable(false);

bool convert(field_recognition::convert_frames::Request &req, field_recognition::convert_frames::Response &resp){
    offset_x = req.offset_x;
    offset_y = req.offset_y;
    rotate_theta = acos(req.xAxis_x / sqrt(pow(req.xAxis_x,2) + pow(req.xAxis_y,2)));
    if(req.xAxis_y < 0) rotate_theta = -rotate_theta;
    resp.theta_rad = rotate_theta;
    resp.theta_degree = rotate_theta * 180/M_PI;
    isTrafoAvailable = true;
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "world_coor_broadcaster");
    ros::NodeHandle node;
    ros::ServiceServer get_trafo = node.advertiseService("convertFrames", convert);
    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(50.0);
    //wait until trafo is incoming
    while(ros::ok()){
        ros::spinOnce();
        if(isTrafoAvailable) break;
        rate.sleep();
    }
    tf::Quaternion qt(tf::Vector3(0,0,1), rotate_theta);
    transform.setRotation(qt);
    transform.setOrigin(tf::Vector3(offset_x, offset_y, 0.0));

    //broadcast world coordinate
    while (node.ok()){
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/world"));
        rate.sleep();
    }
    return 0;
};
