#ifndef GAMEMOTORCONTROL_H
#define GAMEMOTORCONTROL_H
#include "motorControl.h"
#include "Map.h"
#include <vector>

#define PI 3.141592

using namespace std;

enum stateCatchPuck{
    CATCH_DODGE,
    CATCH_OBJ_FAR,
    CATCH_OBJ_NEAR,
    CATCH_OBJ_VERY_NEAR,
    CATCH_OBJ_LOST,
    CATCH_OBJ_CATCHED,
};

enum stateShootPuck{
    SHOOT_STEER,
    SHOOT_ADVANCING,
    SHOOT_OBJ_LOST,
    SHOOT_OFF_FIELD,
    SHOOT_ARRIVED,
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
    void rotateBeyondAngel(double angularVel, double rad);
    bool rotateUntilNoObjAtBack(double angVel);
    bool movePuckToGoal_simple(double linVel, double angVel, double objRadius, cartesianCoordinate targetPose, vector<controlMessage> &msg);
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
    vector<radialCoordinate> distToBorder;
    double robotWorldAngle;
    Map hockeyField;
    void getRobotWorldPose();
    bool isRobotAtBorder();
    bool isRobotOutofBorder();
    bool getClosestDistToBorder();
    bool closeToBorder_front(radialCoordinate &dist);
    bool closeToBorder_back(radialCoordinate &dist);
    bool closeToBorder_right(radialCoordinate &dist);
    bool closeToBorder_left(radialCoordinate &dist);
};

#endif // GAMEMOTORCONTROL_H
