#include <boost/bind.hpp>
#include <iostream>
#include "bumper.h"

using namespace std;
#ifdef SINGLE_FIFO_TH_DATAREQ
Bumper::Bumper()//:sample_rate(10)
{
    isBumperDataAvailable=false;
    bumperSub = node.subscribe("mobile_base/events/bumper", 10, &Bumper::bumperCallback, this);
}
#endif

#ifdef MULTI_TH_DATAREQ
Bumper::Bumper()//:sample_rate(10)
{
    isBumperDataAvailable=false;
    bumperSub = node.subscribe("mobile_base/events/bumper", 10, &Bumper::bumperCallback, this);
}
#endif

int Bumper::getBumperNr()
{
    isBumperDataAvailable = false;
    return bumperNr;
}
int Bumper::getBumperState()
{
    isBumperDataAvailable = false;
    return bumperState;
}

void Bumper::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    ROS_INFO("Bumper Callback");
    ROS_INFO("Bumper: %d",msg->bumper);
    ROS_INFO("Bumper State: %d",msg->state);
    bumperNr=msg->bumper;
    bumperState=msg->state;
    isBumperDataAvailable = true;
}
