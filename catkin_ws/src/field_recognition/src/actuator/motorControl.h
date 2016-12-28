#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H
#include "Camera.h"
#include "CamObject.h"
#include "laserobject.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <string>
#include "globalConfig.h"

#define MOTOR_IN_SIMULATOR

using namespace std;
class motorControl
{
public:
    motorControl();
    //motorControl(laserObject &lobj, turtlebotCamera &Cobj);
    void initMotor(laserObject *, turtlebotCamera*);

private:
    ros::NodeHandle motor_node;
    ros::Publisher velocityPub;

    tf::TransformListener listener;
    tf::StampedTransform trafo_Odom2Base;
    laserObject *laserProcessPtr;
    turtlebotCamera *cameraProcessPtr;
    //laserObject laserProcess;
    //turtlebotCamera cameraProcess;
    vector<CamObject> camObj;
    void getTrafo_Odom2Robot();
    ros::Rate sample_rate;

public:
    cartesianCoordinate getRobotOrientVector(tf::StampedTransform trafo);
    void rotateToOdomVector(double angularVel, cartesianCoordinate targetVec);
    void rotateUntilObjInMiddle(double angularVel, double angleMin, double angleMax);  //obj captured by laser
    void rotateUntilObjInMiddle(double angularVel, Color objcolor);     //obj captured by camera
    void moveToOdomPose(double vel, cartesianCoordinate targetPose);
    void moveToWorldPose(double vel, cartesianCoordinate targetPose);
    void moveToFirstObjRight(double linearVel, double angularVel, Color objcolor);
    void moveUntilMinDist(double vel, double dist, double detectAngleRange);
    void moveByDist(double vel, double dist);
    void testDataReceipt();

    const static double ANGLE_CONTROL_PRECISION = 0.05;
    const static double ANGLE_DETECT_PRECISION = 0.1;
    const static double POSITION_PRECISION = 0.05;
};

#endif // MOTORCONTROL_H
