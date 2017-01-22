#include "motorControl.h"
#include "fieldRecogAI.h"
#include "gameAI.h"
#include "ros/ros.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "game_control");
#ifdef TEST_MODE
    laserScanner laserProcess;
    turtlebotCamera3D cam3DProcess;
//    QApplication app(argc, argv);
//    Angelina angelina;
//    angelina.testconnect();
//    angelina.ReportReady();
    fieldRecogAI detectField(&laserProcess, &cam3DProcess);
    Map playground(1.2, 3, yell);
    playground.blGate.poseInWorld.setVal(1.5, 0.45);
    playground.blGate.height = 0.3; playground.blGate.width = 1;
    playground.yellGate.poseInWorld.setVal(1.5, 3.15);
    playground.yellGate.height = 0.3; playground.yellGate.width = 1;
    playground.postObjs[0].poseInOdom.setVal(-0.45, 1.5);
    detectField.hockeyField = playground;
    detectField.activateWorldCoordinate();
//    angelina.SendAlive();
//    QApplication::processEvents();
//    gameAI gameRoutine(playground,&laserProcess, &cam3DProcess);
//    gameRoutine.startFighting();
#else
    laserScanner laserProcess;
    turtlebotCamera3D cam3DProcess;
    //Angelina angelina;
    Map playground;

    fieldRecogAI detectField(&laserProcess, &cam3DProcess);
    detectField.startFieldRecognition();
    playground = detectField.hockeyField;
    playground.printMap();

    ROS_INFO("Map printed");
    gameAI gameRoutine(playground,&laserProcess, &cam3DProcess);
    gameRoutine.startFighting();
#endif
}
