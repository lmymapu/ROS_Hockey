#include "motorControl.h"


motorControl::motorControl():sample_rate(10),laserProcessPtr(NULL),cameraProcessPtr(NULL)
{
#ifdef SIMULATION_MODE
    velocityPub = motor_node.advertise<geometry_msgs::Twist>("cmd_vel", 10, this);
#else
    velocityPub = motor_node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10, this);
#endif
}

//motorControl::motorControl(laserObject &lobj, turtlebotCamera &Cobj):sample_rate(10),laserProcess(lobj),cameraProcess(Cobj){
//#ifdef SIMULATION_MODE
//    velocityPub = motor_node.advertise<geometry_msgs::Twist>("cmd_vel", 10, this);
//#else
//    velocityPub = motor_node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10, this);
//#endif
//}

void motorControl::initMotor(laser_node *ln, turtlebotCamera *lc){
    laserProcessPtr = ln;
    cameraProcessPtr = lc;
}

void motorControl::getTrafo_Odom2Robot(){
    try{
        listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), trafo_Odom2Base);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
}

cartesianCoordinate motorControl::getRobotOrientVector(tf::StampedTransform trafo){
    cartesianCoordinate orient;
    orient.x = cos(trafo.getRotation().getAxis().getZ() * trafo.getRotation().getAngle());
    orient.y = sin(trafo.getRotation().getAxis().getZ() * trafo.getRotation().getAngle());
    return orient;
}

void motorControl::rotateToOdomVector(double angularVel, cartesianCoordinate targetVec){
    cartesianCoordinate orient;
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = angularVel;
    while(ros::ok()){
        getTrafo_Odom2Robot();
        orient = getRobotOrientVector(trafo_Odom2Base);
        if(acos(orient * targetVec) > 2 * ANGLE_CONTROL_PRECISION){
            vel_msg.angular.z = angularVel;
            velocityPub.publish(vel_msg);
        }else if(acos(orient * targetVec) > ANGLE_CONTROL_PRECISION){
            vel_msg.angular.z = 0.5 * angularVel;
            velocityPub.publish(vel_msg);
        }else{
            vel_msg.angular.z = 0;
            velocityPub.publish(vel_msg);
            break;
        }
    }
}

void motorControl::rotateUntilObjInMiddle(double angularVel, double angleRange){

}

void motorControl::rotateUntilObjInMiddle(double angularVel, Color objcolor){

}

void motorControl::moveToOdomPose(double vel, cartesianCoordinate targetPose){

}

void motorControl::moveToWorldPose(double vel, cartesianCoordinate targetPose){

}

void motorControl::moveUntilMinDist(double vel, double dist, double detectAngleRange){

}

void motorControl::moveByDist(double vel, double dist){

}

void motorControl::testDataReceipt(){
    while(ros::ok()){
        getTrafo_Odom2Robot();
        cartesianCoordinate orient = getRobotOrientVector(trafo_Odom2Base);
        ROS_INFO_STREAM("Robot orientation: " << orient.x << "\t" << orient.y);
        if(laserProcessPtr->isLaserDataAvailable){
            //laserProcessPtr->recogniseObjects();
            //laserProcessPtr->transformOdomCartCoor(trafo_Odom2Base);
            laserProcessPtr->processLaserData(trafo_Odom2Base);
            laserProcessPtr->isLaserDataAvailable = false;
        }

        if(!cameraProcessPtr->mImage.empty()){
            cameraProcessPtr->detectObject(green, camObj);
            ROS_INFO_STREAM("Number of detected objects: "<< camObj.size());
        }
        sample_rate.sleep();
    }
}
