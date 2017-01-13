#ifndef LASEROBJECT_H
#define LASEROBJECT_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include "globalConfig.h"
#include "ObjectInMap.h"
#include <mutex>
#define LASER_IN_SIMULATOR

using namespace std;


class laserObject
{
public:
    laserObject();
    laserObject(const laserObject &);
    laserObject(sensor_msgs::LaserScan::ConstPtr);
#ifdef MULTI_FIFO_DATAREQ
    ros::CallbackQueue Laser_queue;
#endif
#ifdef MULTI_FIFO_TH_DATAREQ
    ros::CallbackQueue Laser_queue;
#endif

    vector<radialCoordinate> objectsInRadialCoor;
    vector<cartesianCoordinate> objectsInRobotCartCoor;
    vector<cartesianCoordinate> objectsInOdomCartCoor;
    void showObjectsPose();
    bool findClosestObjectRadialPose(double angleMin, double angleMax, radialCoordinate &closestDist);
    bool findClosestObjectRadialPose_blk(double angleMin, double angleMax, radialCoordinate &closestDist);
    bool findLeftMostObjectRadialPose(double angleMin, double angleMax, radialCoordinate &leftPose);
    bool findLeftMostObjectRadialPose_blk(double angleMin, double angleMax, radialCoordinate &leftPose);
    bool findRightMostObjectRadialPose(double angleMin, double angleMax, radialCoordinate &rightPose);
    bool findRightMostObjectRadialPose_blk(double angleMin, double angleMax, radialCoordinate &rightPose);

    void transformOdomCartCoor(tf::StampedTransform trafo_Odom2Base);
    void setDetectRange(double dl){laserMaxDetectRange=dl;}
    bool isLaserDataAvailable;
private:
    ros::NodeHandle node;
    ros::Subscriber laserSub;
    ros::Rate sample_rate;
    mutex laserMutex;
    double laserMaxDetectRange;
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

};

#endif // LASEROBJECT_H
