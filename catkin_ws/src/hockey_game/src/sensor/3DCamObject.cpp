#include "3DCamObject.h"

CamObject3D::CamObject3D():mColor(otherColor), mType(otherObject), mEulerDistanceMedian(0),mEulerDistance(0),
    mLaserDistance(0),mNumPoints(0), mTotalNumPoints(0), mAreaInImage(0.0), mOnFloor(false)
{}

cartesianCoordinate CamObject3D::getOdomPose(double objRadius){
    tf::TransformListener listener;
    tf::StampedTransform trafo_Odom2Cam;
    tf::Vector3 objPoseInRobot, objPoseInOdom;
    cartesianCoordinate odomPose;
    try{
        listener.waitForTransform("/odom", "/camera_link", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/odom", "/camera_link", ros::Time(0), trafo_Odom2Cam);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }

    objPoseInRobot.setValue(m3DPositionMedian.z * (1 + objRadius/sqrt(pow(m3DPositionMedian.x, 2)+pow(m3DPositionMedian.z, 2))), -(m3DPositionMedian.x * (1 + objRadius/sqrt(pow(m3DPositionMedian.x, 2)+pow(m3DPositionMedian.z, 2)))), 0);
    objPoseInOdom = trafo_Odom2Cam * objPoseInRobot;
    odomPose.x = objPoseInOdom.getX();
    odomPose.y = objPoseInOdom.getY();
    return odomPose;
}

cartesianCoordinate CamObject3D::getWorldPose(double objRadius){
    tf::TransformListener listener;
    tf::StampedTransform trafo_World2Cam;
    tf::Vector3 objPoseInRobot, objPoseInWorld;
    cartesianCoordinate worldPose;
    try{
        listener.waitForTransform("/world", "/camera_link", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/world", "/camera_link", ros::Time(0), trafo_World2Cam);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }

    objPoseInRobot.setValue(m3DPositionMedian.z * (1 + objRadius/sqrt(pow(m3DPositionMedian.x, 2)+pow(m3DPositionMedian.z, 2))), -(m3DPositionMedian.x * (1 + objRadius/sqrt(pow(m3DPositionMedian.x, 2)+pow(m3DPositionMedian.z, 2)))), 0);
    objPoseInWorld = trafo_World2Cam * objPoseInRobot;
    worldPose.x = objPoseInWorld.getX();
    worldPose.y = objPoseInWorld.getY();
    return worldPose;
}
