#ifndef LASERSCAN_H
#define LASERSCAN_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include "globalConfig.h"
#include "laserObject.h"
#include "ObjectInMap.h"
#include <mutex>
#define LASER_IN_SIMULATOR

using namespace std;


class laserScanner
{
public:
    laserScanner();
    laserScanner(const laserScanner &);
#ifdef MULTI_FIFO_DATAREQ
    ros::CallbackQueue Laser_queue;
#endif
#ifdef MULTI_FIFO_TH_DATAREQ
    ros::CallbackQueue Laser_queue;
#endif
    vector<laserObject> lObjects;
    vector<radialCoordinate> objectsInRadialCoor;

    void showObjectsPose();
    bool findClosestObjectRadialPose(double angleMin, double angleMax, radialCoordinate &closestDist);
    bool findClosestObjectRadialPose(double angleMin, double angleMax, laserObject &closestDist);
    bool findClosestObjectRadialPose_blk(double angleMin, double angleMax, radialCoordinate &closestDist);
    bool findClosestObjectRadialPose_blk(double angleMin, double angleMax, laserObject &closestDist);

    bool findLeftMostObjectRadialPose(double angleMin, double angleMax, radialCoordinate &leftPose);
    bool findLeftMostObjectRadialPose(double angleMin, double angleMax, laserObject &leftPose);
    bool findLeftMostObjectRadialPose_blk(double angleMin, double angleMax, radialCoordinate &leftPose);
    bool findLeftMostObjectRadialPose_blk(double angleMin, double angleMax, laserObject &leftPose);

    bool findRightMostObjectRadialPose(double angleMin, double angleMax, radialCoordinate &rightPose);
    bool findRightMostObjectRadialPose(double angleMin, double angleMax, laserObject &rightPose);
    bool findRightMostObjectRadialPose_blk(double angleMin, double angleMax, radialCoordinate &rightPose);
    bool findRightMostObjectRadialPose_blk(double angleMin, double angleMax, laserObject &rightPose);

    void setDetectRange(double dl){laserMaxDetectRange=dl;}

    void setLaserDataDirty(){isLaserDataAvailable = false;}
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
