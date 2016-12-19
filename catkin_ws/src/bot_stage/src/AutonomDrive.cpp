#include "AutonomDrive.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

AutonomDrive::AutonomDrive()
{
    commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel",1);
    laserSub = node.subscribe("base_scan", 1, &AutonomDrive::scanCallback, this);
    drvStat = SAFE;
    isTurningAround = false; isLeftRotating = false; isRightRotating = false; isMoving = false;
}

void AutonomDrive::takeAction(){
    switch(drvStat){
    case CRITICAL_LEFT:
        ROS_INFO("rotateLeft!");
        rotateLeft();
        break;
    case CRITICAL_RIGHT:
        ROS_INFO("rotateRight!");
        rotateRight();
        break;
    case NEED_STEER:
        ROS_INFO("steer!");
        steer();
        break;
    case SAFE:
        ROS_INFO("Forward!");
        moveForward();
        break;
    case DEADEND:
        ROS_INFO("goback!");
        turnaround();
    }
}

void AutonomDrive::NextStateController(){
    ROS_INFO("frontMinDist: %f, angle at max dist: %f, max slice dist: %f, leftdist: %f, rightdist: %f]",
             dist_state.frontMinDist, dist_state.angleMaxSliceDist, dist_state.maxSliceDist,
             dist_state.leftAvgDist, dist_state.rightAvgDist);
    switch(drvStat){
    case SAFE:
        if(dist_state.frontMinDist > DANGER_DISTANCE_M){        //if it is free of obstacles ahead, it's safe to drive forward
            drvStat = SAFE;
        }
        else{
            drvStat = NEED_STEER;                               //otherwise steer away
        }
        break;
    case NEED_STEER:
        if(dist_state.frontMinDist > SAFE_DISTANCE_M){          //if it is free of obstacles ahead, it's safe to drive forward
            drvStat = SAFE;
            break;
        }
        if(dist_state.frontMinDist>MIN_PROXIMITY_DISTANCE_M){   //if obstable in front is not critically close, steering is enough:)
            drvStat = NEED_STEER;
        }else{                                                                                                              //if vehicle is very close to an obstacle, it has to stop and rotate!
            if(dist_state.leftAvgDist < MIN_PROXIMITY_DISTANCE_M && dist_state.rightAvgDist < MIN_PROXIMITY_DISTANCE_M){    //if left and right side have no room as well, vehicle encounters dead end!
                drvStat = DEADEND;
            }else{
                if(dist_state.leftAvgDist < dist_state.rightAvgDist){                                                       //in case of no dead end, turn to the side with larger room.
                    drvStat = CRITICAL_RIGHT;
                }else{
                    drvStat = CRITICAL_LEFT;
                }
            }
        }
        break;
    case CRITICAL_LEFT:
        if(dist_state.frontMinDist < MIN_PROXIMITY_DISTANCE_M){         //keep rotating left, until it's clear again ahead
            drvStat = CRITICAL_LEFT;
        }else{
            drvStat = NEED_STEER;
        }
        break;
    case CRITICAL_RIGHT:
        if(dist_state.frontMinDist < MIN_PROXIMITY_DISTANCE_M){         //keep rotating right, until it's clear again ahead
            drvStat = CRITICAL_RIGHT;
        }else{
            drvStat = NEED_STEER;
        }
        break;
    case DEADEND:
        if(dist_state.frontMinDist < MIN_PROXIMITY_DISTANCE_M){         //keep turning around, until it's clear again ahead
            drvStat = DEADEND;
        }else{
            drvStat = NEED_STEER;
        }
    }
}

void AutonomDrive::rotateLeft(){
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = ROTATE_SPEED_RPS;
    commandPub.publish(msg);
}

void AutonomDrive::rotateRight(){
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = -ROTATE_SPEED_RPS;
    commandPub.publish(msg);
}

void AutonomDrive::steer(){
    geometry_msgs::Twist msg;
    double forward_speed;
    forward_speed = FORWARD_SPEED_MPS;
    msg.angular.z = STEER_AMPLIFY_FACTOR * dist_state.angleMaxSliceDist * ROTATE_SPEED_RPS / MAX_SCAN_ANGLE_RAD;    //always steer to the angle with maximal average distance of angle slice
    msg.linear.x = FORWARD_SPEED_MPS;
    commandPub.publish(msg);
}

void AutonomDrive::moveForward(){
    geometry_msgs::Twist msg;
    msg.linear.x = FORWARD_SPEED_MPS;
    msg.angular.z = 0;
    commandPub.publish(msg);
}

void AutonomDrive::turnaround(){
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = ROTATE_SPEED_RPS * 2;
    commandPub.publish(msg);
}

/*scanCallback: get current road condition */
void AutonomDrive::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan){
    double sliceAngle = (MAX_SCAN_ANGLE_RAD - MIN_SCAN_ANGLE_RAD) / (1.0*distStatistic::NUM_OF_ANGLESLICE);
    double startAngle = MIN_SCAN_ANGLE_RAD;
    double endAngle = MIN_SCAN_ANGLE_RAD + sliceAngle;
    int startIndex = ceil((startAngle - scan->angle_min) / scan->angle_increment );
    int endIndex = floor((endAngle - scan->angle_min ) / scan->angle_increment );

    dist_state.frontMinDist = scan->ranges[startIndex];
    dist_state.minSliceDist = scan->range_max;
    dist_state.maxSliceDist = 0;
    for(int sl=0; sl!=distStatistic::NUM_OF_ANGLESLICE; ++sl){
        double avg_slice =0, sum_slice=0;
        for(int currIndex = startIndex; currIndex <= endIndex;++currIndex){
            if(sl>=1 && sl<=distStatistic::NUM_OF_ANGLESLICE-2 && scan->ranges[currIndex] < dist_state.frontMinDist){
                dist_state.frontMinDist = scan->ranges[currIndex];
                dist_state.angleFrontMinDist = startAngle + currIndex * scan->angle_increment;
            }
            sum_slice += scan->ranges[currIndex];
        }
        avg_slice = sum_slice / (endIndex - startIndex +1);
        dist_state.distMap[sl] = avg_slice;
        if (avg_slice > dist_state.maxSliceDist){
            dist_state.maxSliceDist = avg_slice;
            dist_state.angleMaxSliceDist = startAngle + sliceAngle/2;
        }
        if (avg_slice < dist_state.minSliceDist){
            dist_state.minSliceDist = avg_slice;
            dist_state.angleMinSliceDist = startAngle + sliceAngle/2;
        }


        //move to next slice
        startAngle += sliceAngle;
        endAngle += sliceAngle;
        startIndex = ceil((startAngle - scan->angle_min) / scan->angle_increment);
        endIndex = floor((endAngle - scan->angle_min) / scan->angle_increment);
    }
    dist_state.leftAvgDist = dist_state.distMap[distStatistic::NUM_OF_ANGLESLICE - 1];
    dist_state.rightAvgDist = dist_state.distMap[0];

}


void AutonomDrive::startMoving(){
    ros::Rate loop_rate(50);
    ROS_INFO("Start moving!");

    while(ros::ok()){
        ROS_INFO("Next round: ");
        takeAction();                   //take action, depending on current state
        ros::spinOnce();                //gather information of road condition in front of vehicle
        NextStateController();          //calculate state in next loop
        loop_rate.sleep();
    }
}
