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

void motorControl::initMotor(laserObject *ln, turtlebotCamera *lc){
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
    while(ros::ok()){
        getTrafo_Odom2Robot();
        orient = getRobotOrientVector(trafo_Odom2Base);
        if(acos(orient * targetVec) > 2 * ANGLE_CONTROL_PRECISION){
            vel_msg.angular.z = angularVel;
            velocityPub.publish(vel_msg);
        }else if(acos(orient * targetVec) > ANGLE_CONTROL_PRECISION){
            vel_msg.angular.z = 0.2 * angularVel;
            velocityPub.publish(vel_msg);
        }else{
            vel_msg.angular.z = 0;
            velocityPub.publish(vel_msg);
            break;
        }
        sample_rate.sleep();
    }
}

void motorControl::rotateUntilObjInMiddle(double angularVel, double angleMin, double angleMax){
    radialCoordinate objRadialPose;
    geometry_msgs::Twist vel_msg;
    while(ros::ok()){
        ros::spinOnce();
        if(laserProcessPtr->isLaserDataAvailable){
            laserProcessPtr->isLaserDataAvailable = false;
            if(!laserProcessPtr->findClosestObjectRadialPose(angleMin, angleMax, objRadialPose)){
                vel_msg.angular.z = angularVel;
                velocityPub.publish(vel_msg);
            }else{
                if(objRadialPose.theta < 0.5*ANGLE_DETECT_PRECISION && objRadialPose.theta > -0.5*ANGLE_DETECT_PRECISION){
                    vel_msg.angular.z = 0;
                    velocityPub.publish(vel_msg);
                    return;
                }else{
                    vel_msg.angular.z = abs(angularVel) * sin(objRadialPose.theta);
                    velocityPub.publish(vel_msg);
                }
            }
        }
        sample_rate.sleep();
    }
}

void motorControl::rotateUntilObjInMiddle(double angularVel, Color objcolor){
    geometry_msgs::Twist vel_msg;
    camObj.clear();
    while(ros::ok()){
        ros::spinOnce();
        if(!cameraProcessPtr->mImage.empty()){
            cameraProcessPtr->detectObject(objcolor, camObj);
        }
        sample_rate.sleep();
    }
}

void motorControl::moveToOdomPose(double vel, cartesianCoordinate targetPose){
    geometry_msgs::Twist vel_msg;
    tf::Vector3 targetPoseInRobot, targetPoseInOdom;
    targetPoseInOdom.setValue(targetPose.x, targetPose.y, 0);

    while(ros::ok()){
        getTrafo_Odom2Robot();
        targetPoseInRobot = trafo_Odom2Base.inverse() * targetPoseInOdom;
        if(targetPoseInRobot.length() > POSITION_PRECISION){
            vel_msg.angular.z = 8*vel * atan2(targetPoseInRobot.getY(), targetPoseInRobot.getX());
            vel_msg.linear.x = vel * sqrt(pow(targetPoseInRobot.getX(), 2) + pow(targetPoseInRobot.getY(), 2));
            velocityPub.publish(vel_msg);
        }else{
            vel_msg.angular.z = 0;
            vel_msg.linear.x = 0;
            velocityPub.publish(vel_msg);
            return;
        }
        sample_rate.sleep();
    }
}

void motorControl::moveToWorldPose(double vel, cartesianCoordinate targetPose){

}

void motorControl::moveUntilMinDist(double vel, double dist, double detectAngleRange){
    radialCoordinate objRadialPose;
    geometry_msgs::Twist vel_msg;
    while(ros::ok()){
        ros::spinOnce();
        if(laserProcessPtr->isLaserDataAvailable){
            laserProcessPtr->isLaserDataAvailable = false;
            if(!laserProcessPtr->findClosestObjectRadialPose(-detectAngleRange/2, detectAngleRange/2, objRadialPose)){
                vel_msg.linear.x = vel;
                velocityPub.publish(vel_msg);
            }else{
                if(objRadialPose.r > dist){
                    vel_msg.linear.x = 0.5*vel;
                    velocityPub.publish(vel_msg);
                }else{
                    vel_msg.linear.x = 0;
                    velocityPub.publish(vel_msg);
                    return;
                }
            }
        }
        sample_rate.sleep();
    }
}

void motorControl::moveByDist(double vel, double dist){
    geometry_msgs::Twist vel_msg;
    tf::Vector3 targetPoseInRobot, targetPoseInOdom;
    targetPoseInRobot.setValue(dist, 0, 0);
    getTrafo_Odom2Robot();
    targetPoseInOdom = trafo_Odom2Base * targetPoseInRobot;
    while(ros::ok()){
        getTrafo_Odom2Robot();
        targetPoseInRobot = trafo_Odom2Base.inverse() * targetPoseInOdom;
        if(targetPoseInRobot.length() > POSITION_PRECISION){
            vel_msg.angular.z = 8*vel * atan2(targetPoseInRobot.getY(), targetPoseInRobot.getX());
            vel_msg.linear.x = vel * sqrt(pow(targetPoseInRobot.getX(), 2) + pow(targetPoseInRobot.getY(), 2));
            velocityPub.publish(vel_msg);
        }else{
            vel_msg.angular.z = 0;
            vel_msg.linear.x = 0;
            velocityPub.publish(vel_msg);
            return;
        }
        sample_rate.sleep();
    }
}

void motorControl::testDataReceipt(){
    while(ros::ok()){
        getTrafo_Odom2Robot();
        cartesianCoordinate orient = getRobotOrientVector(trafo_Odom2Base);
        ROS_INFO_STREAM("Robot orientation: " << orient.x << "\t" << orient.y);
        ros::spinOnce();
        if(laserProcessPtr->isLaserDataAvailable){
            //laserProcessPtr->recogniseObjects();
            //laserProcessPtr->transformOdomCartCoor(trafo_Odom2Base);
            laserProcessPtr->showObjectsPose();
            laserProcessPtr->isLaserDataAvailable = false;
        }

        if(!cameraProcessPtr->mImage.empty()){
            cameraProcessPtr->detectObject(green, camObj);
            ROS_INFO_STREAM("Number of detected objects: "<< camObj.size());
        }
        sample_rate.sleep();
    }
}
