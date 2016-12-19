#include "Stopper.h"
#include "geometry_msgs/Twist.h"

Stopper::Stopper()
{
    keepMoving = true;
    commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel",10);
    laserSub = node.subscribe("base_scan", 1, &Stopper::scanCallback, this);
}

void Stopper::moveForward(){
    if(keepMoving){
        geometry_msgs::Twist msg;
        msg.linear.x = FORWARD_SPEED_MPS;
        commandPub.publish(msg);
    }else{
        return;
    }
}

void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan){
    int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

    float closestRange = scan->ranges[minIndex];
    for(int currIndex = minIndex; currIndex != maxIndex; ++currIndex){
        if(scan->ranges[currIndex] < closestRange){
            closestRange = scan->ranges[currIndex];
        }
    }

    ROS_INFO_STREAM("Closest distance to wall: " << closestRange);

    if(closestRange < MIN_PROXIMITY_RANGE_M){
        ROS_INFO("Stop!");
        keepMoving = false;
    }else{
        keepMoving = true;
    }
}

void Stopper::startMoving(){
    ros::Rate loop_rate(10);
    ROS_INFO("Start moving!");

    while(ros::ok()){
        moveForward();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
