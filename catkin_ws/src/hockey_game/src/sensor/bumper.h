#ifndef BUMPER_H
#define BUMPER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vector>
#include "globalConfig.h"
#include <mutex>
#include "BumperEvent.h"

#define BUMPER_IN_SIMULATOR

using namespace std;


class Bumper
{
public:
    Bumper();
    Bumper(const Bumper &);
#ifdef MULTI_FIFO_DATAREQ
    ros::CallbackQueue Bumper_queue;
#endif
#ifdef MULTI_FIFO_TH_DATAREQ
    ros::CallbackQueue Bumper_queue;
#endif
    int getBumperNr();
    int getBumperState();
    bool isBumperDataAvailable;
private:
    ros::NodeHandle node;
    ros::Subscriber bumperSub;
    mutex bumperMutex;
    void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
    
    /*    
     bumperNr:    
         LEFT = 0
         CENTER = 1
         RIGHT = 2
     bumperState
         RELEASED = 0
         PRESSED = 1
    */
    int bumperNr, bumperState;
};

#endif // BUMPER_H
