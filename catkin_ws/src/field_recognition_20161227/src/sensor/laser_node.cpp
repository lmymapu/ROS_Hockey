#include "laser_node.h"
//#define LASER_CHANNEL

laser_node::laser_node()
{
#ifdef LASER_CHANNEL
    node.setCallbackQueue(&laser_queue);
#endif
#ifndef SIMULATION_MODE
    laserSub = node.subscribe("/scan_laser", 10, &laser_node::scanCallback, this);
#else
    laserSub = node.subscribe("/base_scan_1", 10, &laser_node::scanCallback, this);
#endif
    isLaserDataAvailable = false;
}

void laser_node::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    laserObj = new laserObject(scan);
    isLaserDataAvailable = true;
}

void laser_node::getLaserData(){
    laser_queue.callAvailable(ros::WallDuration(0.2));
}

void laser_node::processLaserData(tf::StampedTransform trafo){

    laserObj->recogniseObjects();
    laserObj->transformOdomCartCoor(trafo);
    laserObj->showObjectsPose();
    delete laserObj;
}

radialCoordinate laser_node::getClosestObjRadialPose_nblk(double angleMin, double angleMax){

    radialCoordinate objPose;
    laserObj->recogniseObjects();
    objPose = laserObj->findClosestObjectRadialPose(angleMin, angleMax);
    delete laserObj;
    return objPose;
}

radialCoordinate laser_node::getClosestObjRadialPose_blk(double angleMin, double angleMax){
    while(!isLaserDataAvailable){}
    isLaserDataAvailable = false;
    return getClosestObjRadialPose_nblk(angleMin, angleMax);
}

cartesianCoordinate laser_node::getClosestObjCartPose_nblk(double angleMin, double angleMax){

    cartesianCoordinate objCartPose;
    laserObj->recogniseObjects();
    objCartPose = laserObj->findClosestObjectCartPose(angleMin, angleMax);
    delete laserObj;
    return objCartPose;
}

cartesianCoordinate laser_node::getClosestObjCartPose_blk(double angleMin, double angleMax){
    while(!isLaserDataAvailable){}
    isLaserDataAvailable = false;
    return getClosestObjCartPose_nblk(angleMin, angleMax);
}
