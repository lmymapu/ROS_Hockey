
#include "worldFrameBroadcast.h"
#include "angelinaConnect.h"
#include <QtGui>


int main(int argc, char** argv){
    ros::init(argc, argv, "interactor");
    worldFrameBroadcast worldFr;
    double start = ros::Time::now().toSec();
    double stop = ros::Time::now().toSec();
    //Connect to angelina
    QApplication app(argc, argv);
    Angelina angelina;
    angelina.testconnect();
    angelina.ReportReady();

    //angelina.sendPosition();
    //angelina.tellAbRatio();
    //angelina.tellTeamColor();
    //angelina.ReportGoal();
    //angelina.ReportDone();
  

    ros::Rate rate(50);
    //wait until trafo is incoming
    while(ros::ok()){
        // Process events angelina connection


        ros::spinOnce();
        if(worldFr.isTrafoAvailable) break;
        rate.sleep();
    }
    worldFr.setWorldFrame();
    //broadcast world coordinate
    while (ros::ok()){
        //stop = ros::Time::now().toSec();
        worldFr.sendWorldFrame();
        rate.sleep();
    }
    return 0;
}
