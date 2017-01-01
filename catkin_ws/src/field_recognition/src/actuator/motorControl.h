#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H
#include "Camera.h"
#include "CamObject.h"
#include "laserobject.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <string>
#include "globalConfig.h"

struct CamLaserGuide{
    double camWindowBeg;
    double camWindowEnd;
    double laserWindowBeg;
    double laserWindowEnd;
    Color objColor;
    CamLaserGuide():camWindowBeg(0),camWindowEnd(1),laserWindowBeg(-M_PI),laserWindowEnd(M_PI),objColor(green){}
    CamLaserGuide(double CWB, double CWE, double LWB, double LWE, Color C):
        camWindowBeg(CWB),camWindowEnd(CWE),laserWindowBeg(LWB),laserWindowEnd(LWE),objColor(C){}
    void setValue(double CWB, double CWE, double LWB, double LWE, Color C){
        camWindowBeg=CWB; camWindowEnd=CWE; laserWindowBeg=LWB; laserWindowEnd=LWE, objColor=C;
    }
};

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
    void moveToLeftMostObj(double linearVel, double angularVel, double minDist, CamLaserGuide CLparam);
    void moveAndSearchLeftMostObj(double linearVel, double angularVel, double minDist, CamLaserGuide CLparam);
    void moveToRightMostObj(double linearVel, double angularVel, double minDist, CamLaserGuide CLparam);
    void moveAndSearchRightMostObj(double linearVel, double angularVel, double minDist, CamLaserGuide CLparam);
    void moveUntilMinDist(double vel, double dist, double detectAngleRange);
    void moveByDist(double vel, double dist);
    void testDataReceipt();

    const static double ANGLE_CONTROL_PRECISION = 0.05;
    const static double ANGLE_DETECT_PRECISION = 0.1;
    const static double POSITION_PRECISION = 0.05;
};

#endif // MOTORCONTROL_H
