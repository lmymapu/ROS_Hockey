#ifndef WORLDFRAMEBROADCAST_H
#define WORLDFRAMEBROADCAST_H
#include <ros/ros.h>
#include "field_recognition/convert_frames.h"
#include <tf/transform_broadcaster.h>

class worldFrameBroadcast
{
public:
    worldFrameBroadcast();
    bool convert(field_recognition::convert_frames::Request &req, field_recognition::convert_frames::Response &resp);
    void sendWorldFrame();
    void setWorldFrame();
    bool isTrafoAvailable;

private:
    ros::NodeHandle world_node;
    ros::ServiceServer get_trafo;
    tf::TransformBroadcaster br;
    tf::Transform transform;

    double offset_x, offset_y;
    double rotate_theta;

};

#endif // WORLDFRAMEBROADCAST_H
