#ifndef FIELDRECOGAI_H
#define FIELDRECOGAI_H
#include "ros/ros.h"
#include "Camera.h"
#include "laser_node.h"
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
    move_34,
    halt_pos4,
    move_45,
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
    ros::AsyncSpinner data_request;
    motorControl motorProcess;


    tf::TransformListener listener;
    tf::StampedTransform trafo_Odom2Base;
    laser_node laserProcess;
    turtlebotCamera cameraProcess;
    vector<CamObject> camObj;

    fieldRecogState stat;
    bool isRecogJobFinished;
    bool isMovingFinished;
    cartesianCoordinate xAxisInOdom, yAxisInOdom;

    cartesianCoordinate calculateObjOdomPose(cartesianCoordinate robotPose);
public:
    const static double MINDIST_OBJECT = 0.3;
    const static double NORMAL_LINEAR_VEL = 1;
    const static double FINECTRL_LINEAR_VEL = 0.2;
    const static double NORMAL_ANGULAR_VEL = 1;
    const static double FINECTRL_ANGULAR_VEL = 0.2;
};

#endif // FIELDRECOGAI_H
