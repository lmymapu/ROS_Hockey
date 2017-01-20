#include "motorControl.h"


motorControl::motorControl():laserProcessPtr(NULL),cam3DProcessPtr(NULL),sample_rate(10),rotateAngleSum(0)
{
    setControlPrecision(MOTOR_ANGLE_CTRL_PREC,MOTOR_ANGLE_LOCK_PREC,MOTOR_POSE_CTRL_PREC);
#ifdef SIMULATION_MODE
    velocityPub = motor_node.advertise<geometry_msgs::Twist>("cmd_vel", 10, this);
#else
    velocityPub = motor_node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10, this);
#endif
    //poseSub = motor_node.subscribe("/odom", 1, &motorControl::poseCallback, this);
}

void motorControl::initMotor(laserScanner *ln, turtlebotCamera3D *lc){
    laserProcessPtr = ln;
    cam3DProcessPtr = lc;
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

    targetVec.normalize();
    while(ros::ok()){
        getTrafo_Odom2Robot();
        orient = getRobotOrientVector(trafo_Odom2Base);
        if((orient - targetVec).length() > 2 * motorAngleCtrlPrec){
            vel_msg.angular.z = angularVel;
            velocityPub.publish(vel_msg);
        }else if((orient - targetVec).length() > motorAngleCtrlPrec){
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

void motorControl::rotateBeyondOdomVector(double angularVel, cartesianCoordinate targetVec){
    cartesianCoordinate orientXY;
    tf::Vector3 curOrient, tarOrient;
    geometry_msgs::Twist vel_msg;
    double curAngle, lastAngle;
    if(angularVel > 0) lastAngle=-1;
    else lastAngle=1;

    tarOrient.setValue(targetVec.x, targetVec.y, 0);

    while(ros::ok()){
        getTrafo_Odom2Robot();
        orientXY = getRobotOrientVector(trafo_Odom2Base);
#ifdef DEBUG_ON
        //ROS_INFO_STREAM("Robot orientation: "<< orientXY.x << "  "<< orientXY.y);
#endif
        curOrient.setValue(orientXY.x, orientXY.y, 0);

        if(angularVel > 0){
            curAngle = curOrient.cross(tarOrient).getZ();
#ifdef DEBUG_ON
        //ROS_INFO_STREAM("Robot angle to target: "<< curAngle << " last angle: "<<lastAngle);
#endif
            if(curAngle < 0 && lastAngle >=0){
                vel_msg.angular.z = 0;
                velocityPub.publish(vel_msg);
                return;
            }else{
                vel_msg.angular.z = angularVel;
                velocityPub.publish(vel_msg);
            }
            lastAngle = curAngle;
        }else{
            curAngle = curOrient.cross(tarOrient).getZ();
            if(curAngle > 0 && lastAngle <=0){
                vel_msg.angular.z = 0;
                velocityPub.publish(vel_msg);
                return;
            }else{
                vel_msg.angular.z = angularVel;
                velocityPub.publish(vel_msg);                
            }
            lastAngle = curAngle;
        }
        sample_rate.sleep();
    }
}

bool motorControl::rotateUntilObjInMiddle(double angularVel, double angleMin, double angleMax){
    radialCoordinate objRadialPose;
    geometry_msgs::Twist vel_msg;
    double angleSum = 0;

    getTrafo_Odom2Robot();
    cartesianCoordinate orient = getRobotOrientVector(trafo_Odom2Base);
    tf::Vector3 lastOrient,currOrient;
    lastOrient.setValue(orient.x, orient.y,0);
    while(ros::ok()){
        getTrafo_Odom2Robot();
        orient = getRobotOrientVector(trafo_Odom2Base);
        currOrient.setValue(orient.x, orient.y, 0);
        angleSum -= currOrient.cross(lastOrient).getZ();
        if(angleSum < -2*M_PI || angleSum > 2*M_PI){
            return false;                                               //No object found
        }
        lastOrient = currOrient;
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
        if(laserProcessPtr->isLaserDataAvailable){
            if(!laserProcessPtr->findClosestObjectRadialPose(angleMin, angleMax, objRadialPose)){
                vel_msg.angular.z = angularVel;
                velocityPub.publish(vel_msg);
            }else{
                if(objRadialPose.theta < 0.5*motorAngleLockPrec && objRadialPose.theta > -0.5*motorAngleLockPrec){
                    vel_msg.angular.z = 0;
                    velocityPub.publish(vel_msg);
                    rotateAngleSum += angleSum;
                    return true;
                }else{
                    vel_msg.angular.z = abs(angularVel) * sin(objRadialPose.theta);
                    velocityPub.publish(vel_msg);
                }
            }
        }
        sample_rate.sleep();
    }
}

bool motorControl::rotateUntilObjInWindow(double angularVel, Color objcolor, CamLaserGuide CLparam){
    geometry_msgs::Twist vel_msg;
    double offset;
    double angleSum = 0;

    getTrafo_Odom2Robot();
    cartesianCoordinate orient = getRobotOrientVector(trafo_Odom2Base);
    tf::Vector3 lastOrient,currOrient;
    lastOrient.setValue(orient.x, orient.y,0);
    while(ros::ok()){
        getTrafo_Odom2Robot();
        orient = getRobotOrientVector(trafo_Odom2Base);
        currOrient.setValue(orient.x, orient.y, 0);
        angleSum -= currOrient.cross(lastOrient).getZ();
        if(angleSum < -2*M_PI || angleSum > 2*M_PI){
            return false;
        }
        lastOrient = currOrient;
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        cam3DProcessPtr->Cam2D_queue.callOne();
        laserProcessPtr->Laser_queue.callOne();
#endif
        if(cam3DProcessPtr->is2DImgReady()){
            cam3DProcessPtr->detectObject2D(objcolor);

            if(!cam3DProcessPtr->leftMostObj(objcolor, CLparam.camWindowBeg, CLparam.camWindowEnd, offset)){
                vel_msg.angular.z = angularVel;
                velocityPub.publish(vel_msg);
            }else{
                vel_msg.angular.z = 0;
                velocityPub.publish(vel_msg);
                rotateAngleSum += angleSum;
                return true;
            }
        }
        sample_rate.sleep();
    }
}

bool motorControl::rotateUntilNoObjInWindow(double angularVel, Color objcolor, CamLaserGuide CLparam){
    geometry_msgs::Twist vel_msg;
    double offset;
    double angleSum = 0;

    getTrafo_Odom2Robot();
    cartesianCoordinate orient = getRobotOrientVector(trafo_Odom2Base);
    tf::Vector3 lastOrient,currOrient;
    lastOrient.setValue(orient.x, orient.y,0);
    while(ros::ok()){
        getTrafo_Odom2Robot();
        orient = getRobotOrientVector(trafo_Odom2Base);
        currOrient.setValue(orient.x, orient.y, 0);
        angleSum -= currOrient.cross(lastOrient).getZ();
        if(angleSum < -2*M_PI || angleSum > 2*M_PI){
            return false;               //object is everywhere in field, no empty room
        }
        lastOrient = currOrient;
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        cam3DProcessPtr->Cam2D_queue.callOne();
        laserProcessPtr->Laser_queue.callOne();
#endif
        if(cam3DProcessPtr->is2DImgReady()){
            cam3DProcessPtr->detectObject2D(objcolor);

            if(cam3DProcessPtr->leftMostObj(objcolor, CLparam.camWindowBeg, CLparam.camWindowEnd, offset)){
                vel_msg.angular.z = angularVel;
                velocityPub.publish(vel_msg);
            }else{
                vel_msg.angular.z = 0;
                velocityPub.publish(vel_msg);
                rotateAngleSum += angleSum;
                return true;
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
        if(targetPoseInRobot.length() > motorPoseCtrlPrec){
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

void motorControl::moveBeyondOdomPose(double vel, cartesianCoordinate targetPose){
    geometry_msgs::Twist vel_msg;
    tf::Vector3 targetPoseInRobot, targetPoseInOdom;
    tf::Vector3 startPoseInOdom,curPoseInOdom;

    targetPoseInOdom.setValue(targetPose.x, targetPose.y, 0);
    getTrafo_Odom2Robot();
    startPoseInOdom = trafo_Odom2Base.getOrigin();
    double targetDist = targetPoseInOdom.distance(startPoseInOdom);
    double moveDist,moveAngle;

    while(ros::ok()){
        getTrafo_Odom2Robot();
        targetPoseInRobot = trafo_Odom2Base.inverse() * targetPoseInOdom;
        curPoseInOdom = trafo_Odom2Base.getOrigin();
        moveDist = curPoseInOdom.distance(startPoseInOdom);
        moveAngle = curPoseInOdom.angle(startPoseInOdom);
        if(moveDist < targetDist){
            vel_msg.angular.z = 8*vel * atan2(targetPoseInRobot.getY(), targetPoseInRobot.getX());
            vel_msg.linear.x = vel;
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

//this function guides the robot to the left most obj on the right section of picture.
void motorControl::moveToLeftMostObj_Simple(double linearVel, double angularVel, double minDist, CamLaserGuide CLparam){        //angularVel must be negative
    if(angularVel > 0) angularVel = -angularVel;

    geometry_msgs::Twist vel_msg;
    double offsetObjMid, offsetObjLeft;
    radialCoordinate ObjPose;
    bool existsObjMid(false), existsObjLeft(false);
    bool existsLaserObj(false);

    bool firstCamDataReady(false), firstLaserDataReady(false);
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        cam3DProcessPtr->Cam2D_queue.callOne();
        laserProcessPtr->Laser_queue.callOne();
#endif
        if(cam3DProcessPtr->is2DImgReady()){
            firstCamDataReady = true;
            cam3DProcessPtr->detectObject2D(CLparam.objColor);
            existsObjMid = cam3DProcessPtr->objectInMiddle(CLparam.objColor, offsetObjMid);
            existsObjLeft = cam3DProcessPtr->leftMostObj(CLparam.objColor, CLparam.camWindowBeg, CLparam.camWindowEnd, offsetObjLeft);
        }
        if(laserProcessPtr->isLaserDataAvailable){
            firstLaserDataReady = true;
            existsLaserObj = laserProcessPtr->findClosestObjectRadialPose(CLparam.laserWindowBeg, CLparam.laserWindowEnd, ObjPose);
        }
        if(!firstCamDataReady || !firstLaserDataReady){
            sample_rate.sleep();
            continue;
        }

        if(existsObjMid){
            vel_msg.angular.z = 0.5*offsetObjMid/(cam3DProcessPtr->imgWidth) * angularVel;
        }else if(existsObjLeft){
            vel_msg.angular.z = 1*offsetObjLeft/(cam3DProcessPtr->imgWidth) * angularVel;
        }else{
            vel_msg.angular.z = 1*angularVel;
        }

        if(existsLaserObj){
            if(ObjPose.r < minDist){
                vel_msg.angular.z = 0;
                vel_msg.linear.x = 0;
                velocityPub.publish(vel_msg);
                return;
            }else{
            vel_msg.linear.x = ObjPose.r/LASER_MAX_DETECT_RANGE * linearVel;
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

//this function guides the robot to the left most obj on the right section of picture.
void motorControl::moveToLeftMostObj(double linearVel, double angularVel, double minDist, CamLaserGuide CLparam){        //angularVel must be negative
    if(angularVel > 0) angularVel = -angularVel;

    geometry_msgs::Twist vel_msg;
    double offsetObjMid, offsetObjLeft,offsetObjLeftSection;
    radialCoordinate ObjPose;
    bool existsObjMid(false), existsObjLeft(false), existsObjLeftSection(false);
    bool existsLaserObj(false);
    bool foundObjRightSection(false);

    bool firstCamDataReady(false), firstLaserDataReady(false);
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        cam3DProcessPtr->Cam2D_queue.callOne();
        laserProcessPtr->Laser_queue.callOne();
#endif
        if(cam3DProcessPtr->is2DImgReady()){
            firstCamDataReady = true;
            cam3DProcessPtr->detectObject2D(CLparam.objColor);
            existsObjMid = cam3DProcessPtr->objectInMiddle(CLparam.objColor, offsetObjMid);
            existsObjLeft = cam3DProcessPtr->leftMostObj(CLparam.objColor, CLparam.camWindowBeg, CLparam.camWindowEnd, offsetObjLeft);
            existsObjLeftSection = cam3DProcessPtr->leftMostObj(CLparam.objColor, 0, CLparam.camWindowBeg, offsetObjLeft);
        }
        if(laserProcessPtr->isLaserDataAvailable){
            firstLaserDataReady = true;
            existsLaserObj = laserProcessPtr->findClosestObjectRadialPose(CLparam.laserWindowBeg, CLparam.laserWindowEnd, ObjPose);

        }
        if(!firstCamDataReady || !firstLaserDataReady){
            sample_rate.sleep();
            continue;
        }
        if(foundObjRightSection){
            if(existsObjLeftSection){
                vel_msg.angular.z = 6*offsetObjLeftSection/(cam3DProcessPtr->imgWidth) * angularVel;
            }else if(existsObjLeft){
                vel_msg.angular.z = 1*offsetObjLeft/(cam3DProcessPtr->imgWidth) * angularVel;
            }else if(existsObjMid){
                vel_msg.angular.z = 0.5*offsetObjMid/(cam3DProcessPtr->imgWidth) * angularVel;
            }else{
                vel_msg.angular.z = 1* angularVel;
            }
        }else{
            if(existsObjMid){
                foundObjRightSection = true;
                vel_msg.angular.z = 0.5*offsetObjMid/(cam3DProcessPtr->imgWidth) * angularVel;
            }else if(existsObjLeft){
                foundObjRightSection = true;
                vel_msg.angular.z = 3*offsetObjLeft/(cam3DProcessPtr->imgWidth) * angularVel;
            }else{
                vel_msg.angular.z = 1*angularVel;
            }
        }
        if(existsLaserObj){
            if(ObjPose.r < minDist){
                vel_msg.angular.z = 0;
                vel_msg.linear.x = 0;
                velocityPub.publish(vel_msg);
                return;
            }else{
            vel_msg.linear.x = ObjPose.r/LASER_MAX_DETECT_RANGE * linearVel;
            //vel_msg.angular.z = -4*angularVel*sin(ObjPose.theta);
            }
        }else{
            if(!foundObjRightSection){
                if(existsObjMid || existsObjLeft){
                    vel_msg.linear.x = linearVel;
                }else{
                    vel_msg.linear.x = 0;
                }
            }else{
                if(existsObjMid || existsObjLeft || existsObjLeftSection){
                    vel_msg.linear.x = linearVel;
                }else{
                    vel_msg.linear.x = 0;
                }
            }
        }
        velocityPub.publish(vel_msg);
        sample_rate.sleep();
    }
}
//this function always guide robot to the left most obj on the left section of picture
void motorControl::moveAndSearchLeftMostObj(double linearVel, double angularVel, double minDist, CamLaserGuide CLparam){        //angularVel must be negative
    if(angularVel > 0) angularVel = -angularVel;

    geometry_msgs::Twist vel_msg;
    double offsetObjMid, offsetObjLeft;
    radialCoordinate ObjPose;
    bool existsObjMid(false), existsObjLeft(false);
    bool existsLaserObj(false);

    bool firstCamDataReady(false), firstLaserDataReady(false);
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        cam3DProcessPtr->Cam2D_queue.callOne();
        laserProcessPtr->Laser_queue.callOne();
#endif
        if(cam3DProcessPtr->is2DImgReady()){
            firstCamDataReady = true;
            cam3DProcessPtr->detectObject2D(CLparam.objColor);
            existsObjMid = cam3DProcessPtr->objectInMiddle(CLparam.objColor, offsetObjMid);
            existsObjLeft = cam3DProcessPtr->leftMostObj(CLparam.objColor, CLparam.camWindowBeg, CLparam.camWindowEnd, offsetObjLeft);
        }
        if(laserProcessPtr->isLaserDataAvailable){
            firstLaserDataReady = true;
            existsLaserObj = laserProcessPtr->findClosestObjectRadialPose(CLparam.laserWindowBeg, CLparam.laserWindowEnd, ObjPose);
        }
        if(!firstCamDataReady || !firstLaserDataReady){
            sample_rate.sleep();
            continue;
        }

        if(existsObjLeft){
            vel_msg.angular.z = 4*offsetObjLeft/(cam3DProcessPtr->imgWidth) * angularVel;
        }else if(existsObjMid){
            vel_msg.angular.z = 4*offsetObjMid/(cam3DProcessPtr->imgWidth) * angularVel;
        }else{
            vel_msg.angular.z = -angularVel;
        }

        if(existsLaserObj){
            if(ObjPose.r < minDist){
                vel_msg.angular.z = 0;
                vel_msg.linear.x = 0;
                velocityPub.publish(vel_msg);
                return;
            }else{
            vel_msg.linear.x = ObjPose.r/LASER_MAX_DETECT_RANGE * linearVel;
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

//this function guides the robot to the right most obj on the left section of picture.
void motorControl::moveToRightMostObj_Simple(double linearVel, double angularVel, double minDist, CamLaserGuide CLparam){        //angularVel must be positive
    if(angularVel < 0) angularVel = -angularVel;

    geometry_msgs::Twist vel_msg;
    double offsetObjMid, offsetObjRight, offsetObjRightSection;
    radialCoordinate ObjPose;
    bool existsObjMid(false), existsObjRight(false), existsObjRightSection(false);
    bool existsLaserObj(false);

    bool firstCamDataReady(false), firstLaserDataReady(false);
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        cam3DProcessPtr->Cam2D_queue.callOne();
        laserProcessPtr->Laser_queue.callOne();
#endif
        if(cam3DProcessPtr->is2DImgReady()){
            firstCamDataReady = true;
            cam3DProcessPtr->detectObject2D(CLparam.objColor);
            existsObjMid = cam3DProcessPtr->objectInMiddle(CLparam.objColor, offsetObjMid);
            existsObjRight = cam3DProcessPtr->rightMostObj(CLparam.objColor, CLparam.camWindowBeg, CLparam.camWindowEnd, offsetObjRight);
        }
        if(laserProcessPtr->isLaserDataAvailable){
            firstLaserDataReady = true;
            existsLaserObj = laserProcessPtr->findClosestObjectRadialPose(CLparam.laserWindowBeg, CLparam.laserWindowEnd, ObjPose);

        }
        if(!firstCamDataReady || !firstLaserDataReady){
            sample_rate.sleep();
            continue;
        }

        if(existsObjMid){
            vel_msg.angular.z = -0.5*offsetObjMid/(cam3DProcessPtr->imgWidth) * angularVel;
        }else if(existsObjRight){
            vel_msg.angular.z = -2*offsetObjRight/(cam3DProcessPtr->imgWidth) * angularVel;
        }else{
            vel_msg.angular.z = 1* angularVel;
        }

        if(existsLaserObj){
            if(ObjPose.r < minDist){
                vel_msg.angular.z = 0;
                vel_msg.linear.x = 0;
                velocityPub.publish(vel_msg);
                return;
            }else{
            vel_msg.linear.x = ObjPose.r/LASER_MAX_DETECT_RANGE * linearVel;
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

//this function guides the robot to the right most obj on the left section of picture.
void motorControl::moveToRightMostObj(double linearVel, double angularVel, double minDist, CamLaserGuide CLparam){        //angularVel must be positive
    if(angularVel < 0) angularVel = -angularVel;

    geometry_msgs::Twist vel_msg;
    double offsetObjMid, offsetObjRight, offsetObjRightSection;
    radialCoordinate ObjPose;
    bool existsObjMid(false), existsObjRight(false), existsObjRightSection(false);
    bool existsLaserObj(false);
    bool foundObjLeftSection(false);

    bool firstCamDataReady(false), firstLaserDataReady(false);
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        cam3DProcessPtr->Cam2D_queue.callOne();
        laserProcessPtr->Laser_queue.callOne();
#endif
        if(cam3DProcessPtr->is2DImgReady()){
            firstCamDataReady = true;
            cam3DProcessPtr->detectObject2D(CLparam.objColor);
            existsObjMid = cam3DProcessPtr->objectInMiddle(CLparam.objColor, offsetObjMid);
            existsObjRight = cam3DProcessPtr->rightMostObj(CLparam.objColor, CLparam.camWindowBeg, CLparam.camWindowEnd, offsetObjRight);
            existsObjRightSection = cam3DProcessPtr->rightMostObj(CLparam.objColor, CLparam.camWindowEnd, 1, offsetObjRightSection);
        }
        if(laserProcessPtr->isLaserDataAvailable){
            firstLaserDataReady = true;
            existsLaserObj = laserProcessPtr->findClosestObjectRadialPose(CLparam.laserWindowBeg, CLparam.laserWindowEnd, ObjPose);

        }
        if(!firstCamDataReady || !firstLaserDataReady){
            sample_rate.sleep();
            continue;
        }

        if(foundObjLeftSection){
            if(existsObjRightSection){
                vel_msg.angular.z = -6*offsetObjRightSection/(cam3DProcessPtr->imgWidth) * angularVel;
            }else if(existsObjRight){
                vel_msg.angular.z = -1*offsetObjRight/(cam3DProcessPtr->imgWidth) * angularVel;
            }else if(existsObjMid){
                vel_msg.angular.z = -0.5*offsetObjMid/(cam3DProcessPtr->imgWidth) * angularVel;
            }else{
                vel_msg.angular.z = 1* angularVel;
            }
        }else{
            if(existsObjMid){
                foundObjLeftSection = true;
                vel_msg.angular.z = -0.5*offsetObjMid/(cam3DProcessPtr->imgWidth) * angularVel;
            }else if(existsObjRight){
                foundObjLeftSection = true;
                vel_msg.angular.z = -2*offsetObjRight/(cam3DProcessPtr->imgWidth) * angularVel;
            }else{
                vel_msg.angular.z = 1* angularVel;
            }
        }

        if(existsLaserObj){
            if(ObjPose.r < minDist){
                vel_msg.angular.z = 0;
                vel_msg.linear.x = 0;
                velocityPub.publish(vel_msg);
                return;
            }else{
            vel_msg.linear.x = ObjPose.r/LASER_MAX_DETECT_RANGE * linearVel;
            //vel_msg.angular.z = -4*angularVel*sin(ObjPose.theta);
            }
        }else{
            if(!foundObjLeftSection){
                if(existsObjMid || existsObjRight){
                    vel_msg.linear.x = linearVel;
                }else{
                    vel_msg.linear.x = 0;
                }
            }else{
                if(existsObjMid || existsObjRight || existsObjRightSection){
                    vel_msg.linear.x = linearVel;
                }else{
                    vel_msg.linear.x = 0;
                }
            }
        }
        velocityPub.publish(vel_msg);
        sample_rate.sleep();
    }
}
//this function always guide robot to the right most obj on the right section of picture
void motorControl::moveAndSearchRightMostObj(double linearVel, double angularVel, double minDist, CamLaserGuide CLparam){        //angularVel must be positive
    if(angularVel < 0) angularVel = -angularVel;

    geometry_msgs::Twist vel_msg;
    double offsetObjMid, offsetObjRight;
    radialCoordinate ObjPose;
    bool existsObjMid(false), existsObjRight(false);
    bool existsLaserObj(false);

    bool firstCamDataReady(false), firstLaserDataReady(false);
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        cam3DProcessPtr->Cam2D_queue.callOne();
        laserProcessPtr->Laser_queue.callOne();
#endif
        if(cam3DProcessPtr->is2DImgReady()){
            firstCamDataReady = true;
            cam3DProcessPtr->detectObject2D(CLparam.objColor);
            existsObjMid = cam3DProcessPtr->objectInMiddle(CLparam.objColor, offsetObjMid);
            existsObjRight = cam3DProcessPtr->rightMostObj(CLparam.objColor, CLparam.camWindowBeg, CLparam.camWindowEnd, offsetObjRight);
        }
        if(laserProcessPtr->isLaserDataAvailable){
            firstLaserDataReady = true;
            existsLaserObj = laserProcessPtr->findClosestObjectRadialPose(CLparam.laserWindowBeg, CLparam.laserWindowEnd, ObjPose);

        }
        if(!firstCamDataReady || !firstLaserDataReady){
            sample_rate.sleep();
            continue;
        }

        if(existsObjRight){
            vel_msg.angular.z = -4*offsetObjRight/(cam3DProcessPtr->imgWidth) * angularVel;
        }else if(existsObjMid){
            vel_msg.angular.z = -4*offsetObjMid/(cam3DProcessPtr->imgWidth) * angularVel;
        }else{
            vel_msg.angular.z = -angularVel;
        }

        if(existsLaserObj){
            if(ObjPose.r < minDist){
                vel_msg.angular.z = 0;
                vel_msg.linear.x = 0;
                velocityPub.publish(vel_msg);
                return;
            }else{
            vel_msg.linear.x = ObjPose.r/LASER_MAX_DETECT_RANGE * linearVel;
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

void motorControl::moveUntilMinDist(double vel, double dist, double detectAngleRange){
    radialCoordinate objRadialPose;
    geometry_msgs::Twist vel_msg;
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        cam3DProcessPtr->Cam2D_queue.callOne();
        laserProcessPtr->Laser_queue.callOne();
#endif
        if(laserProcessPtr->isLaserDataAvailable){

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
        if(targetPoseInRobot.length() > 4*motorPoseCtrlPrec){
            vel_msg.angular.z = 8*vel * atan2(targetPoseInRobot.getY(), targetPoseInRobot.getX());
            vel_msg.linear.x = vel;
            velocityPub.publish(vel_msg);
        }else if(targetPoseInRobot.length() > motorPoseCtrlPrec){
            vel_msg.angular.z = 2*vel * atan2(targetPoseInRobot.getY(), targetPoseInRobot.getX());
            vel_msg.linear.x = vel * targetPoseInRobot.length();
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

void motorControl::moveBeyondDist(double vel, double dist){
    geometry_msgs::Twist vel_msg;
    tf::Vector3 startPoseInOdom,curPoseInOdom;
    double moveDist;

    getTrafo_Odom2Robot();
    startPoseInOdom = trafo_Odom2Base.getOrigin();
    while(ros::ok()){
        getTrafo_Odom2Robot();
        curPoseInOdom = trafo_Odom2Base.getOrigin();
        moveDist = curPoseInOdom.distance(startPoseInOdom);
        if(moveDist < dist){
            vel_msg.linear.x = vel;
            vel_msg.angular.z = 0;
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
        ROS_INFO_STREAM("Robot pose: "<<trafo_Odom2Base.getOrigin().getX() << "  "<<trafo_Odom2Base.getOrigin().getY());
        ROS_INFO_STREAM("Robot orientation: " << orient.x << "\t" << orient.y);
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        cam3DProcessPtr->Cam2D_queue.callOne();
        laserProcessPtr->Laser_queue.callOne();
#endif
        if(laserProcessPtr->isLaserDataAvailable){
            //laserProcessPtr->recogniseObjects();
            //laserProcessPtr->transformOdomCartCoor(trafo_Odom2Base);
            laserProcessPtr->showObjectsPose();

        }

        if(cam3DProcessPtr->is2DImgReady()){
            cam3DProcessPtr->detectObject2D(green);
            cam3DProcessPtr->printObjects();
        }
        sample_rate.sleep();
    }
}

void motorControl::poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Transform Coordinates
    std::unique_lock<std::mutex> lock(mutexPose);
    double quatx= msg->pose.pose.orientation.x;
    double quaty= msg->pose.pose.orientation.y;
    double quatz= msg->pose.pose.orientation.z;
    double quatw= msg->pose.pose.orientation.w;
    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, theta);
}
