#ifndef FIELDRECOGAI_H
#define FIELDRECOGAI_H
#include "ros/ros.h"
#include "hockey_game/convert_frames.h"
#include "3DCamera.h"
#include "laserScanner.h"
#include "gameMotorControl.h"
#include "globalConfig.h"
#include "std_srvs/Empty.h"
#include "Map.h"
#include <tf/transform_listener.h>

enum fieldRecogState{
    start_pos0,
    move_01,
    halt_pos1,
    move_12,
    halt_pos2,
    move_23,
    halt_pos3,
    move_34,move_30,
    halt_pos4,
    move_45,move_40,
    halt_pos5,
    move_50,
    end_pos0,
    move_56,
    halt_pos6,
    move_60,
};

class fieldRecogAI
{
public:

    fieldRecogAI();
    fieldRecogAI(laserScanner *lp, turtlebotCamera3D *cp);
    void startFieldRecognition();
    void nextStateControl();
    void stateAction();
    void testDataReceipt();
    void activateWorldCoordinate();
    Map hockeyField;

private:
#ifdef MULTI_TH_DATAREQ
    ros::AsyncSpinner data_request;
#endif

    ros::NodeHandle AI_node;
    ros::ServiceClient send_trafo;
    hockey_game::convert_frames srv_conv;
    ros::ServiceClient start_laserDrv, stop_laserDrv;
    std_srvs::Empty srv_start, srv_stop;
    bool laserScanOn;

    gameMotorControl motorProcess;
    laserScanner* laserProcessPtr;
    turtlebotCamera3D* cameraProcessPtr;

    tf::TransformListener listener;
    tf::StampedTransform trafo_Odom2Base;
    tf::StampedTransform trafo_World2Odom;

    fieldRecogState stat;
    bool isRecogJobFinished;
    bool isMovingFinished;
    cartesianCoordinate xAxisInOdom, yAxisInOdom;

    void calculateAllPostsPose();
    void calculateAllGatesPose();
    void determineTeamPuckColor();
    void determineTeamGateColor();
};

#endif // FIELDRECOGAI_H
