#ifndef FIELDRECOGAI_H
#define FIELDRECOGAI_H
#include "ros/ros.h"
#include "field_recognition/convert_frames.h"
#include "Camera.h"
#include "laserobject.h"
#include "CamObject.h"
#include "motorControl.h"
#include "globalConfig.h"
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
    void startFieldRecognition();
    void nextStateControl();
    void stateAction();
    void testDataReceipt();
    Map hockeyField;

private:
#ifdef MULTI_TH_DATAREQ
    ros::AsyncSpinner data_request;
#endif
#ifdef MULTI_FIFO_TH_DATAREQ
    ros::AsyncSpinner laser_request, cam2D_request;
#endif
    ros::NodeHandle AI_node;
    ros::ServiceClient send_trafo;
    field_recognition::convert_frames srv_conv;

    motorControl motorProcess;
    laserObject laserProcess;
    turtlebotCamera cameraProcess;

    tf::TransformListener listener;
    tf::StampedTransform trafo_Odom2Base;
    tf::StampedTransform trafo_World2Odom;

    fieldRecogState stat;
    bool isRecogJobFinished;
    bool isMovingFinished;
    cartesianCoordinate xAxisInOdom, yAxisInOdom;

    cartesianCoordinate calculatePostOdomPose(cartesianCoordinate robotPose);
    cartesianCoordinate calculatePostOdomPose(radialCoordinate robotPose);
    void activateWorldCoordinate();
    void calculateAllPostsPose();
    void calculateAllGatesPose();
    void determineTeamPuckColor();
    void determineTeamGateColor();
};

#endif // FIELDRECOGAI_H
