#include "AutonomDrive.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "autonomDrive");
    AutonomDrive autonom_drive;
    autonom_drive.startMoving();
    return 0;
}
