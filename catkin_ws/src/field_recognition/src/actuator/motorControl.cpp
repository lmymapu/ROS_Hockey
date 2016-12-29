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
    double offset;
    while(ros::ok()){
        ros::spinOnce();
        if(cameraProcessPtr->isCamDataAvailable){
            cameraProcessPtr->isCamDataAvailable = false;
            cameraProcessPtr->detectObject(objcolor);
            if(angularVel<0){
                if(!cameraProcessPtr->leftMostObj(objcolor, 0.5, 1, offset)){
                    vel_msg.angular.z = angularVel;
                    velocityPub.publish(vel_msg);
                }else{
                    if(!cameraProcessPtr->objectInMiddle(objcolor,offset)){
                        vel_msg.angular.z = 2*offset/(cameraProcessPtr->picWidth) * angularVel;
                        velocityPub.publish(vel_msg);
                    }else{
                        vel_msg.angular.z = 0;
                        velocityPub.publish(vel_msg);
                        return;
                    }
                }
            }else{
                if(!cameraProcessPtr->rightMostObj(objcolor, 0, 0.5, offset)){
                    vel_msg.angular.z = angularVel;
                    velocityPub.publish(vel_msg);
                }else{
                    if(!cameraProcessPtr->objectInMiddle(objcolor,offset)){
                        vel_msg.angular.z = -2*offset/(cameraProcessPtr->picWidth) * angularVel;
                        velocityPub.publish(vel_msg);
                    }else{
                        vel_msg.angular.z = 0;
                        velocityPub.publish(vel_msg);
                        return;
                    }
                }
            }
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

void motorControl::moveToLeftMostObj(double linearVel, double angularVel, double winBeg, double winEnd, Color objcolor){        //angularVel must be negative
    geometry_msgs::Twist vel_msg;
    double offsetObjMid, offsetObjRight;
    radialCoordinate ObjPose;
    bool existsObjMid(false), existsObjRight(false);
    bool existsLaserObj(false);

    bool firstCamDataReady(false), firstLaserDataReady(false);
    while(ros::ok()){
        ros::spinOnce();
        if(cameraProcessPtr->isCamDataAvailable){
            firstCamDataReady = true;
            cameraProcessPtr->detectObject(objcolor);
            existsObjMid = cameraProcessPtr->objectInMiddle(objcolor, offsetObjMid);
            existsObjRight = cameraProcessPtr->leftMostObj(objcolor, winBeg, winEnd, offsetObjRight);
            cameraProcessPtr->isCamDataAvailable = false;
        }
        if(laserProcessPtr->isLaserDataAvailable){
            firstLaserDataReady = true;
            existsLaserObj = laserProcessPtr->findClosestObjectRadialPose(-M_PI/4, M_PI/4, ObjPose);
            laserProcessPtr->isLaserDataAvailable = false;
        }
        if(!firstCamDataReady || !firstLaserDataReady){
            sample_rate.sleep();
            continue;
        }

        if(existsObjMid){
            vel_msg.angular.z = 4*offsetObjMid/(cameraProcessPtr->picWidth) * angularVel;
        }else if(existsObjRight){
            vel_msg.angular.z = 4*offsetObjRight/(cameraProcessPtr->picWidth) * angularVel;
        }else{
            vel_msg.angular.z = angularVel;
        }

        if(existsLaserObj){
            if(ObjPose.r < 0.3){
                vel_msg.angular.z = 0;
                vel_msg.linear.x = 0;
                velocityPub.publish(vel_msg);
                return;
            }else{
            vel_msg.linear.x = ObjPose.r/laserObject::MAX_DETECT_RANGE * linearVel;
            //vel_msg.angular.z = -4*angularVel*sin(ObjPose.theta);
            }
        }else{
            if(existsObjMid || existsObjRight){
                vel_msg.linear.x = linearVel;
            }else{
                vel_msg.linear.x = 0;
            }
        }
        velocityPub.publish(vel_msg);
        sample_rate.sleep();
    }
}

void motorControl::moveToRightMostObj(double linearVel, double angularVel, double winBeg, double winEnd, Color objcolor){        //angularVel must be positive
    geometry_msgs::Twist vel_msg;
    double offsetObjMid, offsetObjLeft;
    radialCoordinate ObjPose;
    bool existsObjMid(false), existsObjLeft(false);
    bool existsLaserObj(false);

    bool firstCamDataReady(false), firstLaserDataReady(false);
    while(ros::ok()){
        ros::spinOnce();
        if(cameraProcessPtr->isCamDataAvailable){
            firstCamDataReady = true;
            cameraProcessPtr->detectObject(objcolor);
            existsObjMid = cameraProcessPtr->objectInMiddle(objcolor, offsetObjMid);
            existsObjLeft = cameraProcessPtr->rightMostObj(objcolor, winBeg, winEnd, offsetObjLeft);
            cameraProcessPtr->isCamDataAvailable = false;
        }
        if(laserProcessPtr->isLaserDataAvailable){
            firstLaserDataReady = true;
            existsLaserObj = laserProcessPtr->findClosestObjectRadialPose(-M_PI/4, M_PI/4, ObjPose);
            laserProcessPtr->isLaserDataAvailable = false;
        }
        if(!firstCamDataReady || !firstLaserDataReady){
            sample_rate.sleep();
            continue;
        }

        if(existsObjMid){
            vel_msg.angular.z = -4*offsetObjMid/(cameraProcessPtr->picWidth) * angularVel;
        }else if(existsObjLeft){
            vel_msg.angular.z = -4*offsetObjLeft/(cameraProcessPtr->picWidth) * angularVel;
        }else{
            vel_msg.angular.z = angularVel;
        }

        if(existsLaserObj){
            if(ObjPose.r < 0.3){
                vel_msg.angular.z = 0;
                vel_msg.linear.x = 0;
                velocityPub.publish(vel_msg);
                return;
            }else{
            vel_msg.linear.x = ObjPose.r/laserObject::MAX_DETECT_RANGE * linearVel;
            //vel_msg.angular.z = -4*angularVel*sin(ObjPose.theta);
            }
        }else{
            if(existsObjMid || existsObjLeft){
                vel_msg.linear.x = linearVel;
            }else{
                vel_msg.linear.x = 0;
            }
        }
        velocityPub.publish(vel_msg);
        sample_rate.sleep();
    }
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
    double offset;
    bool existsObj= false;
    while(ros::ok()){
        getTrafo_Odom2Robot();
        cartesianCoordinate orient = getRobotOrientVector(trafo_Odom2Base);
        ROS_INFO_STREAM("Robot orientation: " << orient.x << "\t" << orient.y);
        ros::spinOnce();
        /*if(laserProcessPtr->isLaserDataAvailable){
            //laserProcessPtr->recogniseObjects();
            //laserProcessPtr->transformOdomCartCoor(trafo_Odom2Base);
            laserProcessPtr->showObjectsPose();
            laserProcessPtr->isLaserDataAvailable = false;
        }*/

        if(cameraProcessPtr->isCamDataAvailable){
            cameraProcessPtr->detectObject(green);
            ROS_INFO_STREAM("Number of detected objects: "<< cameraProcessPtr->cObjects.size());
            /*existsObj = cameraProcessPtr->firstObjLeft(green, offset);
            if(existsObj)
                ROS_INFO_STREAM("first object on the left found at "<< offset);
            existsObj = cameraProcessPtr->firstObjRight(green, offset);
            if(existsObj)
                ROS_INFO_STREAM("first object on the right found at "<< offset);
            existsObj = cameraProcessPtr->lastObjLeft(green, offset);
            if(existsObj)
                ROS_INFO_STREAM("last object on the left found at "<< offset);
            existsObj = cameraProcessPtr->lastObjRight(green, offset);
            if(existsObj)
                ROS_INFO_STREAM("last object on the right found at "<< offset);*/
            cameraProcessPtr->isCamDataAvailable = false;
        }
        sample_rate.sleep();
    }
}
