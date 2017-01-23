#include "laserObject.h"



void laserObject::getDiameter(){
    objDiameter = sqrt(pow(beginPoint.r,2) + pow(endPoint.r,2) - 2*beginPoint.r*endPoint.r*cos(endPoint.theta-beginPoint.theta));
}

cartesianCoordinate laserObject::getOdomPose(double objRadius){
    tf::TransformListener listener;
    tf::StampedTransform trafo_Odom2Laser;
    tf::Vector3 objPoseInRobot, objPoseInOdom;
    cartesianCoordinate odomPose;
    try{
        listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/odom", "/base_link", ros::Time(0), trafo_Odom2Laser);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }

    objPoseInRobot.setValue((closestPoint.r+objRadius) * cos(closestPoint.theta), (closestPoint.r+objRadius) * sin(closestPoint.theta), 0);
    objPoseInOdom = trafo_Odom2Laser * objPoseInRobot;
    odomPose.x = objPoseInOdom.getX();
    odomPose.y = objPoseInOdom.getY();
    return odomPose;
}

cartesianCoordinate laserObject::getWorldPose(double objRadius){
    tf::TransformListener listener;
    tf::StampedTransform trafo_World2Laser;
    tf::Vector3 objPoseInRobot, objPoseInWorld;
    cartesianCoordinate worldPose;
    try{
        listener.waitForTransform("/world", "/base_link", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/world", "/base_link", ros::Time(0), trafo_World2Laser);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }

    objPoseInRobot.setValue((closestPoint.r+objRadius) * cos(closestPoint.theta), (closestPoint.r+objRadius) * sin(closestPoint.theta), 0);
    objPoseInWorld = trafo_World2Laser * objPoseInRobot;
    worldPose.x = objPoseInWorld.getX();
    worldPose.y = objPoseInWorld.getY();
    return worldPose;
}

cartesianCoordinate laserObject::getCamPose(){
    tf::StampedTransform trafo_laser2Cam;
    tf::TransformListener listener;
    try{
        listener.waitForTransform("/camera_rgb_frame", "/base_link", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/camera_rgb_frame", "/base_link", ros::Time(0), trafo_laser2Cam);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
    tf::Vector3 objPoseInLaser, objPoseInCam;
    cartesianCoordinate camPose;
    objPoseInLaser.setValue(closestPoint.r * cos(closestPoint.theta), closestPoint.r * sin(closestPoint.theta), 0);
    objPoseInCam = trafo_laser2Cam * objPoseInLaser;
    camPose.x = objPoseInCam.getX();
    camPose.y = objPoseInCam.getY();
    return camPose;
}

bool laserObject::isObjInTrack(){
    /*
    return (beginPoint.r*sin(beginPoint.theta) < OBJ_ROBOT_SAFE_RADIUS && beginPoint.r*sin(beginPoint.theta) > -OBJ_ROBOT_SAFE_RADIUS &&
            endPoint.r*sin(endPoint.theta) < OBJ_ROBOT_SAFE_RADIUS && endPoint.r*sin(endPoint.theta) > -OBJ_ROBOT_SAFE_RADIUS);
            */
    return (closestPoint.r*sin(closestPoint.theta) < OBJ_ROBOT_SAFE_RADIUS && closestPoint.r*sin(closestPoint.theta) > -OBJ_ROBOT_SAFE_RADIUS);
}
bool laserObject::isObjInRegion(double ybeg, double yend){
    return (closestPoint.r*sin(closestPoint.theta) < yend && closestPoint.r*sin(closestPoint.theta) > ybeg);
}

void laserObject::getAllWorldPose(){
    tf::TransformListener listener;
    tf::StampedTransform trafo_World2Laser;
    tf::Vector3 objPoseInRobot, objPoseInWorld;
    cartesianCoordinate worldPose;
    try{
        listener.waitForTransform("/world", "/base_link", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/world", "/base_link", ros::Time(0), trafo_World2Laser);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
    objPoseInRobot.setValue(closestPoint.r * cos(closestPoint.theta), closestPoint.r * sin(closestPoint.theta), 0);
    objPoseInWorld = trafo_World2Laser * objPoseInRobot;
    closestPointWorld.x = objPoseInWorld.getX();
    closestPointWorld.y = objPoseInWorld.getY();

    objPoseInRobot.setValue(beginPoint.r * cos(beginPoint.theta), beginPoint.r * sin(beginPoint.theta), 0);
    objPoseInWorld = trafo_World2Laser * objPoseInRobot;
    beginPointWorld.x = objPoseInWorld.getX();
    beginPointWorld.y = objPoseInWorld.getY();

    objPoseInRobot.setValue(endPoint.r * cos(endPoint.theta), endPoint.r * sin(endPoint.theta), 0);
    objPoseInWorld = trafo_World2Laser * objPoseInRobot;
    endPointWorld.x = objPoseInWorld.getX();
    endPointWorld.y = objPoseInWorld.getY();
}
