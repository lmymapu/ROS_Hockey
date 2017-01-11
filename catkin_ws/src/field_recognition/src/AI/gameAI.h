#ifndef GAMEAI_H
#define GAMEAI_H
#include "ros/ros.h"
#include "3DCamera.h"
#include "laserobject.h"
#include "CamObject.h"
#include "gameMotorControl.h"
#include "globalConfig.h"
#include "Map.h"
#include <tf/transform_listener.h>
#include <vector>

using namespace std;
enum GameState{
    SEARCH_PUCK,
    CATCH_PUCK,
    SHOOT_PUCK,
    DETACH_PUCK,
    GAME_OVER,
};

class gameAI
{
public:
#ifdef MULTI_TH_DATAREQ
    gameAI():stat(SEARCH_PUCK),data_request(NUM_OF_DATAREQ_THREAD){}
#else
    gameAI():stat(SEARCH_PUCK){}
#endif
    gameAI(const Map &field);
    Map hockeyField;

    void startFighting();
    void stateTransition();
    void testDataReceipt();

private:
#ifdef MULTI_TH_DATAREQ
    ros::AsyncSpinner data_request;
#endif
    ros::NodeHandle AI_node;
    gameMotorControl motorProcess;
    laserObject laserProcess;
    turtlebotCamera3D cam3DProcess;

    tf::TransformListener listener;
    tf::StampedTransform trafo_Odom2Base;
    tf::StampedTransform trafo_World2Odom;

    GameState stat;

    vector<controlMessage> msg_buffer;
    bool existsMessage(const controlMessage msg);
};

#endif // GAMEAI_H
