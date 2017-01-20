#ifndef GAMEAI_H
#define GAMEAI_H
#include "ros/ros.h"
#include "3DCamera.h"
#include "laserScanner.h"
#include "gameMotorControl.h"
#include "globalConfig.h"
#include "Map.h"
#include "std_srvs/Empty.h"
#include <tf/transform_listener.h>
#include <vector>

using namespace std;
enum GameState{
    SEARCH_PUCK,
    CATCH_PUCK,
    CATCH_PUCK_DODGE,
    SHOOT_PUCK,
    SHOOT_PUCK_DODGE,
    DETACH_PUCK,
    GAME_OVER,
};

class gameAI
{
public:
#ifdef SINGLE_FIFO_TH_DATAREQ
    gameAI():stat(SEARCH_PUCK){
        start_laserDrv = AI_node.serviceClient<std_srvs::Empty>("start_motor");
        stop_laserDrv = AI_node.serviceClient<std_srvs::Empty>("stop_motor");
    }
    gameAI(const Map &plgnd):stat(SEARCH_PUCK)
    {
        start_laserDrv = AI_node.serviceClient<std_srvs::Empty>("start_motor");
        stop_laserDrv = AI_node.serviceClient<std_srvs::Empty>("stop_motor");
        hockeyField = plgnd;
    }
#endif
#ifdef MULTI_TH_DATAREQ
    gameAI():stat(SEARCH_PUCK),data_request(NUM_OF_DATAREQ_THREAD),laserScanOn(false){
        start_laserDrv = AI_node.serviceClient<std_srvs::Empty>("start_motor");
        stop_laserDrv = AI_node.serviceClient<std_srvs::Empty>("stop_motor");
    }
    gameAI(const Map &plgnd):stat(SEARCH_PUCK),data_request(NUM_OF_DATAREQ_THREAD),laserScanOn(false)
    {
        start_laserDrv = AI_node.serviceClient<std_srvs::Empty>("start_motor");
        stop_laserDrv = AI_node.serviceClient<std_srvs::Empty>("stop_motor");
        hockeyField = plgnd;
    }
    gameAI(const Map &plgnd, laserScanner *lp, turtlebotCamera3D *cp):stat(SEARCH_PUCK),data_request(NUM_OF_DATAREQ_THREAD),laserScanOn(false)
    {
        start_laserDrv = AI_node.serviceClient<std_srvs::Empty>("start_motor");
        stop_laserDrv = AI_node.serviceClient<std_srvs::Empty>("stop_motor");
        hockeyField = plgnd;
        laserProcessPtr = lp;
        cam3DProcessPtr = cp;
    }
#endif

    Map hockeyField;

    void startFighting();
    void stateTransition();
    void testDataReceipt();

private:
#ifdef MULTI_TH_DATAREQ
    ros::AsyncSpinner data_request;
#endif
    ros::NodeHandle AI_node;
    ros::ServiceClient start_laserDrv, stop_laserDrv;
    std_srvs::Empty srv_start, srv_stop;

    gameMotorControl motorProcess;
    laserScanner* laserProcessPtr;
    turtlebotCamera3D* cam3DProcessPtr;

    tf::TransformListener listener;
    tf::StampedTransform trafo_Odom2Base;
    tf::StampedTransform trafo_World2Odom;
    bool laserScanOn;

    GameState stat;

    vector<controlMessage> msg_buffer;
    bool existsMessage(const controlMessage msg);
};

#endif // GAMEAI_H
