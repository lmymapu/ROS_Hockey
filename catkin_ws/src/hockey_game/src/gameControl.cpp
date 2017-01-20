#include "motorControl.h"
#include "fieldRecogAI.h"
#include "gameAI.h"
#include "ros/ros.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "game_control");
#ifdef TEST_MODE
    laserScanner laserProcess;
    turtlebotCamera3D cam3DProcess;
    fieldRecogAI detectField(&laserProcess, &cam3DProcess);
    Map playground(1.2, 3, yellow);
    playground.blueGate.poseInWorld.setVal(1.5, 0.45);
    playground.blueGate.height = 0.3; playground.blueGate.width = 1;
    playground.yellowGate.poseInWorld.setVal(1.5, 3.15);
    playground.yellowGate.height = 0.3; playground.yellowGate.width = 1;
    playground.postObjs[0].poseInOdom.setVal(-0.45, 1.5);
    detectField.hockeyField = playground;
    detectField.activateWorldCoordinate();
    gameAI gameRoutine(playground,&laserProcess, &cam3DProcess);
    gameRoutine.startFighting();
#else
    laserScanner laserProcess;
    turtlebotCamera3D cam3DProcess;
    fieldRecogAI detectField(&laserProcess, &cam3DProcess);
    Map playground;
    detectField.startFieldRecognition();
    playground = detectField.hockeyField;
    playground.printMap();
    ROS_INFO("Map printed");
    gameAI gameRoutine(playground,&laserProcess, &cam3DProcess);
    gameRoutine.startFighting();
#endif
}
