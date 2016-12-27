#ifndef LASEROBJECT_H
#define LASEROBJECT_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include "globalConfig.h"
#include "ObjectInMap.h"
#define LASER_IN_SIMULATOR

using namespace std;


class laserObject
{
public:
    laserObject();
    laserObject(const laserObject &);
    laserObject(sensor_msgs::LaserScan::ConstPtr);

    vector<radialCoordinate> objectsInRadialCoor;
    vector<cartesianCoordinate> objectsInRobotCartCoor;
    vector<cartesianCoordinate> objectsInOdomCartCoor;
    void showObjectsPose();
    radialCoordinate findClosestObjectRadialPose(double angleMin, double angleMax);
    cartesianCoordinate findClosestObjectCartPose(double angleMin, double angleMax);
    radialCoordinate findClosestObjectRadialPose_blk(double angleMin, double angleMax);
    cartesianCoordinate findClosestObjectCartPose_blk(double angleMin, double angleMax);
    void transformOdomCartCoor(tf::StampedTransform trafo_Odom2Base);

    bool isLaserDataAvailable;
private:
    ros::NodeHandle node;
    ros::Subscriber laserSub;
    ros::Rate sample_rate;
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

public:
    static const double LEAST_INTENSITY = 0.1;
    static const double DETECT_THRESHOLD_DISTDIFF = 0.1;
    static const unsigned int MAX_NUMOF_OBJECTS = 25;
    static const double MAX_DETECT_RANGE = 1.0;
#ifndef SIMULATION_MODE
    static const double ANGLE_OFFSET=M_PI;
#else
    static const double ANGLE_OFFSET=0;
#endif
};

#endif // LASEROBJECT_H
