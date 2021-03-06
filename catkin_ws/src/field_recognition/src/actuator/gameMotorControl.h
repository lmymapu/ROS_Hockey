#ifndef GAMEMOTORCONTROL_H
#define GAMEMOTORCONTROL_H
#include "motorControl.h"
#include "3DCamera.h"
#include <vector>

using namespace std;
enum controlMessage{
    FOUND_A_PUCK,
    PUCK_OUT_OF_SIGHT,
    PUCK_CATCHED,
    PUCK_DETACHED,
    PUCK_LOST,
    GOAL,    
    ALL_PUCKS_IN_GATE,
    TIME_OUT,
};

class gameMotorControl:public motorControl
{
public:
    gameMotorControl();
    void testDataReceiptInGame();

    void initGameMotor(laserObject *lp, turtlebotCamera3D *cp){
        laserProcessPtr=lp; cam3DProcessPtr=cp;
    }
protected:
    vector<controlMessage> msg_buffer;
    tf::StampedTransform trafo_Odom2Cam,trafo_World2Cam;
    tf::StampedTransform trafo_Odom2Laser, trafo_World2Laser;
    cartesianCoordinate calculateObjWorldPose_fromCam(radialCoordinate robotPose);
    cartesianCoordinate calculateObjOdomPose_fromCam(radialCoordinate robotPose);
    cartesianCoordinate calculateObjWorldPose_fromLaser(radialCoordinate robotPose);
    cartesianCoordinate calculateObjOdomPose_fromLaser(radialCoordinate robotPose);
};

#endif // GAMEMOTORCONTROL_H
