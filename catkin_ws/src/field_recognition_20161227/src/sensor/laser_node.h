#ifndef LASER_NODE_H
#define LASER_NODE_H
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "laserobject.h"
#include <tf/transform_listener.h>
#include <ros/callback_queue.h>

class laser_node
{
public:
    laser_node();
    laserObject* laserObj;
    void getLaserData();
    void processLaserData(tf::StampedTransform trafo);
    radialCoordinate getClosestObjRadialPose_nblk(double angleMin, double angleMax);
    radialCoordinate getClosestObjRadialPose_blk(double angleMin, double angleMax);
    cartesianCoordinate getClosestObjCartPose_nblk(double angleMin, double angleMax);
    cartesianCoordinate getClosestObjCartPose_blk(double angleMin, double angleMax);
    bool isLaserDataAvailable;

private:
    ros::NodeHandle node;
    ros::Subscriber laserSub;
    ros::CallbackQueue laser_queue;
    tf::StampedTransform trafo_Odom2Base;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif // LASER_NODE_H
