#include "gameMotorControl.h"

gameMotorControl::gameMotorControl()
{

}

cartesianCoordinate gameMotorControl::calculateObjOdomPose_fromCam(radialCoordinate robotPose){
    try{
        listener.waitForTransform("/odom", "/camera_depth_frame", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/odom", "/camera_depth_frame", ros::Time(0), trafo_Odom2Cam);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
    tf::Vector3 objPoseInRobot, objPoseInOdom;
    cartesianCoordinate odomPose;
    objPoseInRobot.setValue((robotPose.r) * cos(robotPose.theta), (robotPose.r) * sin(robotPose.theta), 0);
    objPoseInOdom = trafo_Odom2Cam * objPoseInRobot;
    odomPose.x = objPoseInOdom.getX();
    odomPose.y = objPoseInOdom.getY();
    return odomPose;
}

cartesianCoordinate gameMotorControl::calculateObjWorldPose_fromCam(radialCoordinate robotPose){
    try{
        listener.waitForTransform("/world", "/camera_depth_frame", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/world", "/camera_depth_frame", ros::Time(0), trafo_World2Cam);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
    tf::Vector3 objPoseInRobot, objPoseInOdom;
    cartesianCoordinate odomPose;
    objPoseInRobot.setValue((robotPose.r) * cos(robotPose.theta), (robotPose.r) * sin(robotPose.theta), 0);
    objPoseInOdom = trafo_World2Cam * objPoseInRobot;
    odomPose.x = objPoseInOdom.getX();
    odomPose.y = objPoseInOdom.getY();
    return odomPose;
}

cartesianCoordinate gameMotorControl::calculateObjOdomPose_fromLaser(radialCoordinate robotPose){
    try{
        listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), trafo_Odom2Laser);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
    tf::Vector3 objPoseInRobot, objPoseInOdom;
    cartesianCoordinate odomPose;
    objPoseInRobot.setValue((robotPose.r) * cos(robotPose.theta), (robotPose.r) * sin(robotPose.theta), 0);
    objPoseInOdom = trafo_Odom2Laser * objPoseInRobot;
    odomPose.x = objPoseInOdom.getX();
    odomPose.y = objPoseInOdom.getY();
    return odomPose;
}

cartesianCoordinate gameMotorControl::calculateObjWorldPose_fromLaser(radialCoordinate robotPose){
    try{
        listener.waitForTransform("/world", "/base_footprint", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/world", "/base_footprint", ros::Time(0), trafo_World2Laser);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
    tf::Vector3 objPoseInRobot, objPoseInOdom;
    cartesianCoordinate odomPose;
    objPoseInRobot.setValue((robotPose.r) * cos(robotPose.theta), (robotPose.r) * sin(robotPose.theta), 0);
    objPoseInOdom = trafo_World2Laser * objPoseInRobot;
    odomPose.x = objPoseInOdom.getX();
    odomPose.y = objPoseInOdom.getY();
    return odomPose;
}

void gameMotorControl::testDataReceiptInGame(){
    while(ros::ok()){
        try{
            listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), trafo_Odom2Base);
        }catch(tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }
        ROS_INFO_STREAM("robot pose: "<<trafo_Odom2Base.getOrigin().getX() << "  "<<trafo_Odom2Base.getOrigin().getY());
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        cam3DProcessPtr->Img_queue.callOne();
        cam3DProcessPtr->PtCloud_queue.callOne();
        laserProcessPtr->Laser_queue.callOne();
#endif
        if(laserProcessPtr->isLaserDataAvailable){
            laserProcessPtr->showObjectsPose();
            laserProcessPtr->isLaserDataAvailable = false;
        }

        if(cam3DProcessPtr->isImgDataReady()){
            cam3DProcessPtr->detectObject(green);
            cam3DProcessPtr->printObjects();
            cam3DProcessPtr->setImgDataDirty();
        }
        sample_rate.sleep();
    }
}
