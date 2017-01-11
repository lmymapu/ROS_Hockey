#include "motorControl.h"
#include "fieldRecogAI.h"
#include "gameAI.h"
#include "ros/ros.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "fieldRecog_rechAngle");
#ifdef TEST_MODE
    fieldRecogAI detectField;
    Map playground;
    detectField.startFieldRecognition();
#else
    fieldRecogAI detectField;
    Map playground;
    detectField.startFieldRecognition();
    playground = detectField.hockeyField;
    playground.printMap();
    ROS_INFO("Map printed");
#endif
}
