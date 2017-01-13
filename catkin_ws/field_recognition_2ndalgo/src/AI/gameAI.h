#ifndef GAMEAI_H
#define GAMEAI_H
#include "ros/ros.h"
#include "3DCamera.h"
#include "laserobject.h"
#include "CamObject.h"
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
    SHOOT_PUCK,
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
#endif
#ifdef MULTI_FIFO_DATAREQ
    gameAI():stat(SEARCH_PUCK){
        start_laserDrv = AI_node.serviceClient<std_srvs::Empty>("start_motor");
        stop_laserDrv = AI_node.serviceClient<std_srvs::Empty>("stop_motor");
    }
#endif
#ifdef MULTI_TH_DATAREQ
    gameAI():stat(SEARCH_PUCK),data_request(NUM_OF_DATAREQ_THREAD){
        start_laserDrv = AI_node.serviceClient<std_srvs::Empty>("start_motor");
        stop_laserDrv = AI_node.serviceClient<std_srvs::Empty>("stop_motor");
    }
#endif

#ifdef MULTI_FIFO_TH_DATAREQ
    gameAI():stat(SEARCH_PUCK),laser_request(NUM_OF_THREAD_MULTIFIFO, &(laserProcess.Laser_queue)),
        cam3DImg_request(NUM_OF_THREAD_MULTIFIFO, &(cam3DProcess.Img_queue)),
        cam3DPtCloud_request(NUM_OF_THREAD_MULTIFIFO, &(cam3DProcess.PtCloud_queue)){
        start_laserDrv = AI_node.serviceClient<std_srvs::Empty>("start_motor");
        stop_laserDrv = AI_node.serviceClient<std_srvs::Empty>("stop_motor");
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
#ifdef MULTI_FIFO_TH_DATAREQ
    ros::AsyncSpinner laser_request, cam3DImg_request, cam3DPtCloud_request;
#endif
    ros::NodeHandle AI_node;
    ros::ServiceClient start_laserDrv, stop_laserDrv;
    std_srvs::Empty srv_start, srv_stop;

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
