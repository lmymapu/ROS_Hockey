#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H
#include "3DCamera.h"
#include "laserScanner.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <string>
#include <mutex>
#include "globalConfig.h"
#include "nav_msgs/Odometry.h"

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
    //motorControl(laserScanner &lobj, turtlebotCamera &Cobj);
    void initMotor(laserScanner *, turtlebotCamera3D*);
    void addLaserSensor(laserScanner *lp){laserProcessPtr = lp;}
    void addCam3DSensor(turtlebotCamera3D *cp3d){cam3DProcessPtr = cp3d;}

protected:
    ros::NodeHandle motor_node;
    ros::Publisher velocityPub;
    mutex mutexPose;
    tf::TransformListener listener;
    tf::StampedTransform trafo_Odom2Base;
    laserScanner *laserProcessPtr;
    turtlebotCamera3D *cam3DProcessPtr;
    ros::Subscriber poseSub;
    void getTrafo_Odom2Robot();

    ros::Rate sample_rate;
    double motorAngleCtrlPrec;
    double motorAngleLockPrec;
    double motorPoseCtrlPrec;
    double rotateAngleSum;

    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
    double theta,goalTheta;
public:
    cartesianCoordinate getRobotOrientVector(tf::StampedTransform trafo);
    void rotateToOdomVector(double angularVel, cartesianCoordinate targetVec);
    void rotateBeyondOdomVector(double angularVel, cartesianCoordinate targetVec);
    bool rotateUntilObjInMiddle(double angularVel, double angleMin, double angleMax);  //obj captured by laser
    bool rotateUntilObjInWindow(double angularVel, Color objcolor, CamLaserGuide CLparam);     //obj captured by camera
    bool rotateUntilNoObjInWindow(double angularVel, Color objcolor, CamLaserGuide CLparam);
    void moveToOdomPose(double vel, cartesianCoordinate targetPose);
    void moveBeyondOdomPose(double vel, cartesianCoordinate targetPose);
    void moveToWorldPose(double vel, cartesianCoordinate targetPose);
    void moveToLeftMostObj_Simple(double linearVel, double angularVel, double minDist, CamLaserGuide CLparam);
    void moveToLeftMostObj(double linearVel, double angularVel, double minDist, CamLaserGuide CLparam);
    void moveAndSearchLeftMostObj(double linearVel, double angularVel, double minDist, CamLaserGuide CLparam);
    void moveToRightMostObj_Simple(double linearVel, double angularVel, double minDist, CamLaserGuide CLparam);
    void moveToRightMostObj(double linearVel, double angularVel, double minDist, CamLaserGuide CLparam);
    void moveAndSearchRightMostObj(double linearVel, double angularVel, double minDist, CamLaserGuide CLparam);
    void moveUntilMinDist(double vel, double dist, double detectAngleRange);
    void moveByDist(double vel, double dist);
    void moveBeyondDist(double vel, double dist);
    void testDataReceipt();
    double getAggregateRotation(){return rotateAngleSum;}
    void resetAggregateRotation(){rotateAngleSum=0;}

    void setControlPrecision(double acp, double alp, double pcp){
        motorAngleCtrlPrec=acp; motorAngleLockPrec=alp; motorPoseCtrlPrec=pcp;
    }


};

#endif // MOTORCONTROL_H
