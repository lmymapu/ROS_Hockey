#include "worldFrameBroadcast.h"

worldFrameBroadcast::worldFrameBroadcast():offset_x(0),offset_y(0),rotate_theta(0),isTrafoAvailable(false)
{
    get_trafo = world_node.advertiseService("convertFrames", &worldFrameBroadcast::convert, this);
}

bool worldFrameBroadcast::convert(field_recognition::convert_frames::Request &req, field_recognition::convert_frames::Response &resp){
    offset_x = req.offset_x;
    offset_y = req.offset_y;
    rotate_theta = acos(req.xAxis_x / sqrt(pow(req.xAxis_x,2) + pow(req.xAxis_y,2)));
    if(req.xAxis_y < 0) rotate_theta = -rotate_theta;
    resp.theta_rad = rotate_theta;
    resp.theta_degree = rotate_theta * 180/M_PI;
    isTrafoAvailable = true;
    return true;
}

void worldFrameBroadcast::setWorldFrame(){
    tf::Quaternion qt(tf::Vector3(0,0,1), rotate_theta);
    transform.setRotation(qt);
    transform.setOrigin(tf::Vector3(offset_x, offset_y, 0.0));
}

void worldFrameBroadcast::sendWorldFrame(){
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/world"));
}
