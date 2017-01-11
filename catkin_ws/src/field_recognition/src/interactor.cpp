
#include "worldFrameBroadcast.h"


int main(int argc, char** argv){
    ros::init(argc, argv, "interactor");
    worldFrameBroadcast worldFr;

    ros::Rate rate(20);
    //wait until trafo is incoming
    while(ros::ok()){
        ros::spinOnce();
        if(worldFr.isTrafoAvailable) break;
        rate.sleep();
    }
    worldFr.setWorldFrame();

    //broadcast world coordinate
    while (ros::ok()){
        worldFr.sendWorldFrame();
        rate.sleep();
    }
    return 0;
};
