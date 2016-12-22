#ifndef AUTONOMDRIVE_H
#define AUTONOMDRIVE_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


/*this struct will store the road condition ahead of vehicle, the scan angles are evenly partitioned into several slices*/
struct distStatistic{
    double maxSliceDist;                        //max average distance in slices
    double minSliceDist;                        //min average distance in slices
    double frontMinDist;                        //min distance at front area
    double angleFrontMinDist;                   //angle at which the distance at front area is minimum
    double frontAvgDist;                        //average distance at front area
    double leftAvgDist;                         //average distance at leftmost angle slice
    double rightAvgDist;                        //average distance at rightmost angle slice
    double angleMaxSliceDist;                   //angle of the slice with max average distance
    double angleMinSliceDist;                   //angle of the slice with min average distance
    const static int NUM_OF_ANGLESLICE = 10;    //number of slices
    double distMap[NUM_OF_ANGLESLICE];          //array which stores the average distance of slices.
};

class AutonomDrive
{
public:
    const static double FORWARD_SPEED_MPS = 0.4;
    const static double ROTATE_SPEED_RPS = 0.8;
    const static double STEER_AMPLIFY_FACTOR = 1.8;
    const static double MIN_SCAN_ANGLE_RAD = -80.0 * M_PI/180.0;
    const static double MAX_SCAN_ANGLE_RAD = 80.0 * M_PI/180.0;
    const static double MIN_PROXIMITY_DISTANCE_M = 0.3;
    const static double SAFE_DISTANCE_M = 0.9;
    const static double DANGER_DISTANCE_M = 0.6;

    AutonomDrive();
    void startMoving();

private:
    ros::NodeHandle node;
    ros::Publisher commandPub;
    ros::Subscriber laserSub;
    bool isTurningAround, isLeftRotating, isRightRotating, isMoving;
    enum driveState{
        SAFE,
        NEED_STEER,
        CRITICAL_LEFT,
        CRITICAL_RIGHT,
        DEADEND,
    } drvStat;



    void takeAction();
    void NextStateController();
    void moveForward();
    void steer();
    void rotateLeft();
    void rotateRight();
    void turnaround();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    distStatistic dist_state;
};



#endif // AUTONOMDRIVE_H
