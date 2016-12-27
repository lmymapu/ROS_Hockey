#include "motorControl.h"
#include "fieldRecogAI.h"
#include "ros/ros.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "fieldRecog_rechAngle");
    fieldRecogAI detectField;
    detectField.startFieldRecognition();
}
