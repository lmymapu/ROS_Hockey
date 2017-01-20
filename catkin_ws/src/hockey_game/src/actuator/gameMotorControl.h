#ifndef GAMEMOTORCONTROL_H
#define GAMEMOTORCONTROL_H
#include "motorControl.h"
#include "Map.h"
#include <vector>

#define PI 3.141592

using namespace std;

enum stateCatchPuck{
    DODGE,
    OBJ_FAR,
    OBJ_NEAR,
    OBJ_VERY_NEAR,
    OBJ_LOST,
    OBJ_CATCHED,
};

enum controlMessage{
    FOUND_NO_PUCK,
    FOUND_A_PUCK,
    PUCK_OUT_OF_SIGHT,
    PUCK_CATCHED,
    PUCK_CATCHED_ERROR,
    PUCK_DETACHED,
    PUCK_LOST,
    OBSTACLE_AHEAD,
    GOAL,    
    ALL_PUCKS_IN_GATE,
    TIME_OUT,
};

class gameMotorControl:public motorControl
{
public:
    gameMotorControl();
    void getMap(const Map& fd){hockeyField=fd;}
    void testDataReceiptInGame();

    void initGameMotor(laserScanner *lp, turtlebotCamera3D *cp){
        laserProcessPtr=lp; cam3DProcessPtr=cp;
    }
    void rotateToRandomVector(double angularVel);
    void moveMaxDist(double linearVel);
    bool rotateUntil3DObjInWindow(double angularVel, Color objcolor, CamLaserGuide CLparam, vector<controlMessage> &msg);
    bool rotateUntilNo3DObjInWindow(double angularVel, Color objcolor, CamLaserGuide CLparam, vector<controlMessage> &msg);
    bool moveAndCatchObj_simple(double linVel, double angVel, CamLaserGuide CLparam, vector<controlMessage> &msg);
    bool moveAndCatchObj(double linVel, double angVel, CamLaserGuide CLparam, cartesianCoordinate &objPose, vector<controlMessage> &msg);
    void rotateAndPointToWorldVec(double angVel, cartesianCoordinate target);
    void rotateBeyondWorldVector(double angularVel, cartesianCoordinate targetVec);
    bool rotateUntilNoObjAtBack(double angVel);
    bool movePuckToGoal(double linVel, double angVel, double objRadius, cartesianCoordinate targetPose, vector<controlMessage> &msg);

    double FORWARD_SPEED_MPS = 0.2; // Determines how fast the robot moves
    float TURN_PARAM = 0.2; // Determines how fast the robot turns
    void moveForward();
    bool hasReachedGoalDegree();
    void turn(float degree);
    void findPuck(Color objcolor);
    void allignToCenter(Color objcolor);
    void moveToObject();

protected:
    vector<controlMessage> msg_buffer;
    tf::StampedTransform trafo_World2Base;
    cartesianCoordinate robotWorldOrient;
    cartesianCoordinate robotWorldCoor;
    double robotWorldAngle;
    Map hockeyField;
    void getRobotWorldPose();
    bool isRobotAtBorder();
    bool isRobotOutofBorder();

};

#endif // GAMEMOTORCONTROL_H
