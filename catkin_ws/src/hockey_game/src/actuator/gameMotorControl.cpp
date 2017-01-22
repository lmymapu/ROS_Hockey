#include "gameMotorControl.h"

gameMotorControl::gameMotorControl()
{
}

void gameMotorControl::rotateToRandomVector(double angularVel){
    cartesianCoordinate tarVec(rand()%100, rand()%100);
    tarVec.x = tarVec.x/100; tarVec.y = tarVec.y/100;
    rotateBeyondOdomVector(angularVel, tarVec);
}

//robot stops either when he reaches border area or there is obstacle ahead
void gameMotorControl::moveMaxDist(double linearVel){
    radialCoordinate objRadialPose;
    bool detectedCloseObj(false);
    geometry_msgs::Twist vel_msg;
    while(ros::ok()){
        getRobotWorldPose();
        if(isRobotAtBorder()){
            vel_msg.linear.x = 0;
            velocityPub.publish(vel_msg);
            return;
        }
        if(laserProcessPtr->isLaserDataAvailable){
            detectedCloseObj = laserProcessPtr->findClosestObjectRadialPose(-M_PI/2, M_PI/2, objRadialPose);
            if(detectedCloseObj && objRadialPose.r < 2*OBJ_ROBOT_SAFE_RADIUS){
                vel_msg.linear.x = 0;
                velocityPub.publish(vel_msg);
                return;
            }
        }
        vel_msg.linear.x = linearVel;
        velocityPub.publish(vel_msg);
        sample_rate.sleep();
    }
}

bool gameMotorControl::rotateUntil3DObjInWindow(double angularVel, Color objcolor, CamLaserGuide CLparam, vector<controlMessage> &msg){
    msg.clear();
    geometry_msgs::Twist vel_msg;
    CamObject3D obj;
    double angleSum = 0;
#ifdef DEBUG_MOTOR
            ROS_INFO("SEARCH_PUCK:start to find puck");
#endif
    getRobotWorldPose();
    tf::Vector3 lastOrient,currOrient;
    lastOrient.setValue(robotWorldOrient.x, robotWorldOrient.y, 0);
    while(ros::ok()){
        getRobotWorldPose();
        currOrient.setValue(robotWorldOrient.x, robotWorldOrient.y, 0);
        angleSum -= currOrient.cross(lastOrient).getZ();
        if(angleSum < -2*M_PI || angleSum > 2*M_PI){
#ifdef DEBUG_MOTOR
            ROS_INFO("SEARCH_PUCK:rotate 360 degree but find no puck");
#endif
            msg.push_back(FOUND_NO_PUCK);
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
            if(!cam3DProcessPtr->leftMostObj(objcolor, CLparam.camWindowBeg, CLparam.camWindowEnd, obj)){
                vel_msg.angular.z = angularVel;
                velocityPub.publish(vel_msg);
            }else{
#ifdef DEBUG_MOTOR
            ROS_INFO("SEARCH_PUCK: Found a puck");
#endif
                vel_msg.angular.z = 0;
                velocityPub.publish(vel_msg);
                msg.push_back(FOUND_A_PUCK);
                rotateAngleSum += angleSum;
                return true;
            }
        }
        sample_rate.sleep();
    }
}

bool gameMotorControl::rotateUntilNo3DObjInWindow(double angularVel, Color objcolor, CamLaserGuide CLparam, vector<controlMessage> &msg){
    msg.clear();
    geometry_msgs::Twist vel_msg;
    CamObject3D obj;
    double angleSum = 0;

    getRobotWorldPose();
    tf::Vector3 lastOrient,currOrient;
    lastOrient.setValue(robotWorldOrient.x, robotWorldOrient.y, 0);
    while(ros::ok()){
        getRobotWorldPose();
        currOrient.setValue(robotWorldOrient.x, robotWorldOrient.y, 0);
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
        if(cam3DProcessPtr->isImgDataReady()){
            cam3DProcessPtr->detectObject3D(objcolor);

            if(cam3DProcessPtr->leftMostObj(objcolor, CLparam.camWindowBeg, CLparam.camWindowEnd, obj)){
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

bool gameMotorControl::moveAndCatchObj_simple(double linVel, double angVel, CamLaserGuide CLparam, vector<controlMessage> &msg){
    msg.clear();
    stateCatchPuck stat = CATCH_OBJ_FAR;
    angVel = abs(angVel);
    geometry_msgs::Twist vel_msg;
    laserObject lobj;
    CamObject3D cObjLeft, cObjMid, cObjRight;
    bool existsCobjMid(false), existsCobjRight(false), existsCobjLeft(false);
    bool existsLobj(false);
    cartesianCoordinate detectedObjPose;

    bool firstCamDataReady(false), firstLaserDataReady(false);
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif

        if(cam3DProcessPtr->is2DImgReady()){
            firstCamDataReady = true;
            cam3DProcessPtr->detectObject2D(CLparam.objColor);
            existsCobjLeft = cam3DProcessPtr->rightMostObj(CLparam.objColor, 0, CLparam.camWindowBeg, cObjLeft);
            existsCobjMid = cam3DProcessPtr->leftMostObj(CLparam.objColor, CLparam.camWindowBeg, CLparam.camWindowEnd, cObjMid);
            existsCobjRight = cam3DProcessPtr->leftMostObj(CLparam.objColor, CLparam.camWindowEnd, 1, cObjRight);
#ifdef DEBUG_MOTOR
            //cam3DProcessPtr->printObjects();
#endif
        }
        if(laserProcessPtr->isLaserDataAvailable){
            firstLaserDataReady = true;
            existsLobj = laserProcessPtr->findClosestObjectRadialPose(CLparam.laserWindowBeg, CLparam.laserWindowEnd, lobj);
        }
        if(!firstCamDataReady || !firstLaserDataReady){
            //sample_rate.sleep();
            continue;
        }

        switch(stat){
        case CATCH_OBJ_FAR:
#ifdef DEBUG_MOTOR
            ROS_INFO("CATCH PUCK: curr_stat=CATCH_OBJ_FAR");
#endif
            if(!existsCobjLeft && !existsCobjRight && !existsCobjMid){
                stat=CATCH_OBJ_LOST;
                break;
            }
            if(existsLobj && lobj.isObjInTrack()){
                stat=CATCH_OBJ_NEAR;
                break;
            }else{
                stat=CATCH_OBJ_FAR;
                break;
            }
        case CATCH_OBJ_NEAR:
#ifdef DEBUG_MOTOR
            ROS_INFO("CATCH PUCK: curr_stat=CATCH_OBJ_NEAR");
#endif
            if(!existsCobjLeft && !existsCobjRight && !existsCobjMid){
                stat=CATCH_OBJ_LOST;
                break;
            }
            if(existsLobj && lobj.isObjInTrack()){
                if(lobj.closestPoint.r < OBJ_ROBOT_DIAMETER/2 + OBJ_PUCK_LOW_RADIUS + 0.03){
                    stat=CATCH_OBJ_CATCHED;
                    break;
                }else{
                    stat=CATCH_OBJ_NEAR;
                    break;
                }
            }else{
                stat=CATCH_OBJ_FAR;
                break;
            }
        case CATCH_OBJ_LOST:
#ifdef DEBUG_MOTOR
            ROS_INFO("CATCH PUCK: curr_stat=CATCH_OBJ_LOST");
#endif
            msg.push_back(PUCK_OUT_OF_SIGHT);
            return false;
        case CATCH_OBJ_CATCHED:
#ifdef DEBUG_MOTOR
            ROS_INFO("CATCH PUCK: curr_stat=CATCH_OBJ_CATCHED");
#endif
            if(!existsCobjMid){
#ifdef DEBUG_MOTOR
            ROS_INFO("CATCH PUCK: wrong puck catched");
#endif
                msg.push_back(PUCK_CATCHED_ERROR);
                return false;
            }else{
#ifdef DEBUG_MOTOR
            ROS_INFO("CATCH PUCK: correct puck catched");
#endif
                msg.push_back(PUCK_CATCHED);
                return true;
            }
            break;
        }

        switch(stat){
        case CATCH_OBJ_FAR:
            if(existsCobjMid){
                vel_msg.angular.z = -2*(cObjMid.mMassCenter.x-320)/(cam3DProcessPtr->imgWidth) * angVel;
            }else if(existsCobjLeft){
                vel_msg.angular.z = -2*(cObjLeft.mMassCenter.x-320)/(cam3DProcessPtr->imgWidth) * angVel;
            }else if(existsCobjRight){
                vel_msg.angular.z = -2*(cObjRight.mMassCenter.x-320)/(cam3DProcessPtr->imgWidth) * angVel;
            }
            vel_msg.linear.x = linVel;
            velocityPub.publish(vel_msg);
            break;
        case CATCH_OBJ_NEAR:
            vel_msg.angular.z = 4*angVel*sin(lobj.closestPoint.theta);
            vel_msg.linear.x = linVel/2;
            velocityPub.publish(vel_msg);
            break;
        default:break;
        }
    }
}

bool gameMotorControl::moveAndCatchObj(double linVel, double angVel, CamLaserGuide CLparam, cartesianCoordinate &objPose, vector<controlMessage> &msg){
    msg.clear();
    angVel = abs(angVel);
    geometry_msgs::Twist vel_msg;
    laserObject lobj;
    double offsetObjLeft, offsetObjMid, offsetObjRight;
    bool existsCobjMid(false), existsCobjRight(false), existsCobjLeft(false);
    bool existsLobj(false);
    cartesianCoordinate detectedObjPose;

    bool firstCamDataReady(false), firstLaserDataReady(false);
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif

        if(cam3DProcessPtr->is2DImgReady()){
            firstCamDataReady = true;
            cam3DProcessPtr->detectObject2D(CLparam.objColor);
            existsCobjLeft = cam3DProcessPtr->rightMostObj(CLparam.objColor, 0, CLparam.camWindowBeg, offsetObjLeft);
            existsCobjMid = cam3DProcessPtr->leftMostObj(CLparam.objColor, CLparam.camWindowBeg, CLparam.camWindowEnd, offsetObjMid);
            existsCobjRight = cam3DProcessPtr->leftMostObj(CLparam.objColor, CLparam.camWindowEnd, 1, offsetObjRight);
        }
        if(laserProcessPtr->isLaserDataAvailable){
            firstLaserDataReady = true;
            existsLobj = laserProcessPtr->findClosestObjectRadialPose(CLparam.laserWindowBeg, CLparam.laserWindowEnd, lobj);
        }
        if(!firstCamDataReady || !firstLaserDataReady){
            //sample_rate.sleep();
            continue;
        }

        if(existsCobjMid){
            vel_msg.angular.z = -2*offsetObjMid/(cam3DProcessPtr->imgWidth) * angVel;
        }else if(existsCobjLeft){
            vel_msg.angular.z = -2*offsetObjLeft/(cam3DProcessPtr->imgWidth) * angVel;
        }else if(existsCobjRight){
            vel_msg.angular.z = -2*offsetObjRight/(cam3DProcessPtr->imgWidth) * angVel;
        }else{
            vel_msg.angular.z = 0;
            vel_msg.linear.x = 0;
            velocityPub.publish(vel_msg);
            msg.push_back(PUCK_OUT_OF_SIGHT);
#ifdef DEBUG_MOTOR
            ROS_INFO("Can't see puck by catching it");
#endif
            return false;
        }

        if(!existsLobj){
            vel_msg.linear.x = linVel;
        }else{
            if(lobj.isObjInTrack()){
                if(lobj.closestPoint.r > MOTOR_SAFE_DIST){
                    vel_msg.linear.x = linVel;
                }else if(lobj.closestPoint.r > MOTOR_DANGER_DIST){
                    vel_msg.linear.x = linVel/2;
                    vel_msg.angular.z = 4*angVel*sin(lobj.closestPoint.theta);
                }else if(lobj.closestPoint.r > OBJ_ROBOT_DIAMETER/2 + OBJ_PUCK_LOW_RADIUS + 0.03){
                    if(!cam3DProcessPtr->leftMostObj(CLparam.objColor, CLparam.camWindowBeg, CLparam.camWindowEnd, offsetObjMid)){        //if detect no puck, then this object is an obstacle
                        vel_msg.angular.z = 0;
                        vel_msg.linear.x = 0;
                        velocityPub.publish(vel_msg);
                        msg.push_back(OBSTACLE_AHEAD);
#ifdef DEBUG_MOTOR
            ROS_INFO("See obstacle by catching puck");
#endif
                        return false;
                    }else{
                        vel_msg.angular.z = 4*angVel*sin(lobj.closestPoint.theta);
                        vel_msg.linear.x = linVel/2;
                    }
                }else{
                    vel_msg.angular.z = 0;
                    vel_msg.linear.x = 0;
                    velocityPub.publish(vel_msg);
                    ros::Duration(1).sleep();
                    if(cam3DProcessPtr->leftMostObj(CLparam.objColor, CLparam.camWindowBeg, CLparam.camWindowEnd, offsetObjMid)){       //confirm again if correct puck is caught
                        objPose=lobj.getWorldPose(OBJ_PUCK_LOW_RADIUS);
                        msg.push_back(PUCK_CATCHED);
#ifdef DEBUG_MOTOR
            ROS_INFO("puck catched");
#endif
                        return true;
                    }else{
#ifdef DEBUG_MOTOR
            ROS_INFO("error puck catched");
#endif
                        msg.push_back(PUCK_CATCHED_ERROR);
                        return false;
                    }
                }

            }
        }
        velocityPub.publish(vel_msg);

    }
}

void gameMotorControl::rotateBeyondWorldVector(double angularVel, cartesianCoordinate targetVec){
    tf::Vector3 curOrient, tarOrient;
    geometry_msgs::Twist vel_msg;
    double curAngle, lastAngle;
    if(angularVel > 0) lastAngle=-1;
    else lastAngle=1;

    tarOrient.setValue(targetVec.x, targetVec.y, 0);

    while(ros::ok()){
        getRobotWorldPose();
#ifdef DEBUG_ON
        ROS_INFO_STREAM("Robot orientation: "<< robotWorldOrient.x << "  "<< robotWorldOrient.y);
#endif
        curOrient.setValue(robotWorldOrient.x, robotWorldOrient.y, 0);

        if(angularVel > 0){
            curAngle = curOrient.cross(tarOrient).getZ();
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

void gameMotorControl::rotateBeyondAngel(double angularVel, double rad){
    double targetAngle;
    cartesianCoordinate targetVec;
    getRobotWorldPose();
    if(angularVel > 0){
        targetAngle = robotWorldAngle + rad;
        targetVec.setVal(cos(targetAngle), sin(targetAngle));
    }else{
        targetAngle = robotWorldAngle - rad;
        targetVec.setVal(cos(targetAngle), sin(targetAngle));
    }
    rotateBeyondWorldVector(angularVel, targetVec);
}

void gameMotorControl::rotateAndPointToWorldVec(double angVel, cartesianCoordinate target){
    getRobotWorldPose();
    cartesianCoordinate tarVec = target - robotWorldCoor;
    cartesianCoordinate midVec;
    tarVec.normalize();
    tf::Vector3 tarOrient, startOrient;
    tarOrient.setValue(tarVec.x, tarVec.y, 0);
    startOrient.setValue(robotWorldOrient.x, robotWorldOrient.y, 0);
    if(startOrient.cross(tarOrient).getZ()>0){
        rotateBeyondWorldVector(angVel,tarVec);
    }else if(startOrient.cross(tarOrient).getZ()<0){
        rotateBeyondWorldVector(-angVel,tarVec);
    }else{
        midVec = getYfromX(robotWorldOrient);
        rotateBeyondWorldVector(angVel,midVec);
        rotateBeyondWorldVector(angVel, tarVec);
    }
}

bool gameMotorControl::rotateUntilNoObjAtBack(double angVel){
    geometry_msgs::Twist vel_msg;
    laserObject lobj;
    bool existsLobj(false);
    while(ros::ok()){
        if(laserProcessPtr->isLaserDataAvailable){
            existsLobj = laserProcessPtr->findClosestObjectRadialPose(-M_PI, -M_PI/2, lobj);
            if(!existsLobj || !lobj.isObjInTrack() || lobj.closestPoint.r > 0.5){
                return true;
            }else{
                vel_msg.angular.z = -abs(angVel);
                velocityPub.publish(vel_msg);
            }
            existsLobj = laserProcessPtr->findClosestObjectRadialPose(M_PI/2, M_PI, lobj);
            if(!existsLobj || !lobj.isObjInTrack() || lobj.closestPoint.r > 0.5){
                return true;
            }else{
                vel_msg.angular.z = abs(angVel);
                velocityPub.publish(vel_msg);
            }
        }
    }
}

bool gameMotorControl::movePuckToGoal_simple(double linVel, double angVel, double objRadius, cartesianCoordinate targetPose, vector<controlMessage> &msg){
    msg.clear();

    stateShootPuck stat=SHOOT_ADVANCING;
    geometry_msgs::Twist vel_msg;
    tf::Vector3 targetPoseInRobot, targetPoseInWorld;
    tf::Vector3 objPoseInWorld;
    laserObject lObjLeft, lObjMid, lObjRight;
    bool existsLObjLeft(false), existsLObjMid(false), existsLObjRight(false);
    bool borderFront(false);
    radialCoordinate nearestBorder;
    PuckInMap puckObj;

    targetPoseInWorld.setValue(targetPose.x, targetPose.y, 0);
    while(ros::ok()){
        if(laserProcessPtr->isLaserDataAvailable){
            existsLObjMid = laserProcessPtr->findClosestObjectInTrack(-OBJ_ROBOT_CATCH_RADIUS, OBJ_ROBOT_CATCH_RADIUS, lObjMid);
            existsLObjLeft = laserProcessPtr->findClosestObjectRadialPose(OBJ_ROBOT_CATCH_RADIUS, OBJ_ROBOT_SAFE_RADIUS, lObjLeft);
            existsLObjRight = laserProcessPtr->findClosestObjectRadialPose(-OBJ_ROBOT_SAFE_RADIUS, -OBJ_ROBOT_CATCH_RADIUS, lObjRight);
            puckObj.poseInWorld = lObjMid.getWorldPose(objRadius);
            objPoseInWorld.setValue(puckObj.poseInWorld.x, puckObj.poseInWorld.y, 0);
        }else{
            continue;
        }
        getRobotWorldPose();
        getClosestDistToBorder();
        borderFront = closeToBorder_front(nearestBorder);
        targetPoseInRobot = trafo_World2Base.inverse() * targetPoseInWorld;
        switch(stat){
        case SHOOT_ADVANCING:
#ifdef DEBUG_MOTOR
            ROS_INFO("SHOOT PUCK: advance to goal");
#endif
            if(!existsLObjMid){
                stat=SHOOT_OBJ_LOST;
            }else{
                if(lObjMid.closestPoint.r > OBJ_ROBOT_DIAMETER/2 + OBJ_PUCK_LOW_RADIUS + 0.05){
                    stat=SHOOT_OBJ_LOST;
                }else{
                    if((objPoseInWorld-targetPoseInWorld).length() < motorPoseCtrlPrec || hockeyField.isPuckInGate(puckObj)){
                        stat=SHOOT_ARRIVED;
                        break;
                    }
                    if(existsLObjLeft || existsLObjRight || borderFront){
                        stat = SHOOT_STEER;
                    }else{
                        stat = SHOOT_ADVANCING;
                    }
                }
            }
            break;
        case SHOOT_STEER:
#ifdef DEBUG_MOTOR
            ROS_INFO("SHOOT PUCK: steer away obstacle");
#endif
            if(!existsLObjMid){
                stat=SHOOT_OBJ_LOST;
            }else{
                if(lObjMid.closestPoint.r > OBJ_ROBOT_DIAMETER/2 + OBJ_PUCK_LOW_RADIUS + 0.05){
                    stat=SHOOT_OBJ_LOST;
                }else{
                    if((objPoseInWorld-targetPoseInWorld).length() < motorPoseCtrlPrec || hockeyField.isPuckInGate(puckObj)){
                        stat=SHOOT_ARRIVED;
                        break;
                    }
                    if(existsLObjLeft || existsLObjRight || borderFront){
                        stat = SHOOT_STEER;
                    }else{
                        stat = SHOOT_ADVANCING;
                    }
                }
            }
            break;
        case SHOOT_OBJ_LOST:
#ifdef DEBUG_MOTOR
            ROS_INFO("SHOOT PUCK: puck get lost");
#endif
            msg.push_back(PUCK_OUT_OF_SIGHT);
            return false;
            break;
        case SHOOT_ARRIVED:
#ifdef DEBUG_MOTOR
            ROS_INFO("SHOOT PUCK: goal reached");
#endif
            msg.push_back(GOAL);
            return true;
            break;
        }

        switch(stat){
        case SHOOT_ADVANCING:
            vel_msg.angular.z = 8*angVel * atan2(targetPoseInRobot.getY(), targetPoseInRobot.getX());
            vel_msg.linear.x = linVel * sqrt(pow(targetPoseInRobot.getX(), 2) + pow(targetPoseInRobot.getY(), 2));
            velocityPub.publish(vel_msg);
            break;
        case SHOOT_STEER:
            if(closeToBorder_left(nearestBorder) && closeToBorder_right(nearestBorder)){
                if(distToBorder[0].theta + distToBorder[1].theta > 0){
                    rotateBeyondAngel(-angVel, M_PI/4);
                }else{
                    rotateBeyondAngel(angVel, M_PI/4);
                }
                break;
            }
            if(closeToBorder_right(nearestBorder) && existsLObjLeft){
                rotateBeyondAngel(angVel, lObjLeft.closestPoint.theta);
                break;
            }
            if(closeToBorder_left(nearestBorder) && existsLObjRight){
                rotateBeyondAngel(-angVel, -lObjLeft.closestPoint.theta);
                break;
            }
            if(closeToBorder_front(nearestBorder)){
                if(nearestBorder.theta < 0){
                    vel_msg.angular.z = 8* angVel * (nearestBorder.theta + M_PI/2);
                    vel_msg.linear.x = linVel * nearestBorder.r;
                }else{
                    vel_msg.angular.z = 8* angVel * (nearestBorder.theta - M_PI/2);
                    vel_msg.linear.x = linVel * nearestBorder.r;
                }
                velocityPub.publish(vel_msg);
                break;
            }
            if(existsLObjLeft){
                vel_msg.angular.z = 8* angVel * (lObjLeft.closestPoint.theta - M_PI/2);
                vel_msg.linear.x = linVel * lObjLeft.closestPoint.r;
                velocityPub.publish(vel_msg);
                break;
            }
            if(existsLObjRight){
                vel_msg.angular.z = 8* angVel * (lObjRight.closestPoint.theta + M_PI/2);
                vel_msg.linear.x = linVel * lObjRight.closestPoint.r;
                velocityPub.publish(vel_msg);
                break;
            }
            break;
        default:break;
        }
    }
}

bool gameMotorControl::movePuckToGoal(double linVel, double angVel, double objRadius, cartesianCoordinate targetPose, vector<controlMessage> &msg){
    msg.clear();
    geometry_msgs::Twist vel_msg;
    tf::Vector3 targetPoseInRobot, targetPoseInWorld;
    tf::Vector3 objPoseInWorld;
    cartesianCoordinate objPose2D;
    laserObject lobj;
    PuckInMap puckObj;

    targetPoseInWorld.setValue(targetPose.x, targetPose.y, 0);
    while(ros::ok()){
        getRobotWorldPose();
        if(laserProcessPtr->isLaserDataAvailable){
            if(!laserProcessPtr->findClosestObjectRadialPose(-M_PI/12,M_PI/12,lobj)){
#ifdef DEBUG_MOTOR
                ROS_INFO("Puck get lost, no puck in hand");
                laserProcessPtr->showObjectsPose();
#endif
                msg.push_back(PUCK_LOST);
                return false;
            }else{
                if(lobj.closestPoint.r > OBJ_ROBOT_DIAMETER/2 + OBJ_PUCK_LOW_RADIUS + 0.05){
#ifdef DEBUG_MOTOR
                ROS_INFO("Puck too far, get lost");
                ROS_INFO("Puck position [%f %f]", lobj.closestPoint.r, lobj.closestPoint.theta*180/M_PI);
#endif
                    msg.push_back(PUCK_LOST);
                    return false;
                }
                objPose2D = lobj.getWorldPose(objRadius);
                objPoseInWorld.setValue(objPose2D.x, objPose2D.y, 0);
                puckObj.poseInWorld = cartesianCoordinate(objPoseInWorld.getX(), objPoseInWorld.getY());
                if((objPoseInWorld-targetPoseInWorld).length() < motorPoseCtrlPrec || hockeyField.isPuckInGate(puckObj)){
#ifdef DEBUG_MOTOR
                ROS_INFO("Goal reached, puck position: [%f %f]", puckObj.poseInWorld.x, puckObj.poseInWorld.y);
#endif
                    msg.push_back(GOAL);
                    return true;
                }
            }
            if(laserProcessPtr->findClosestObjectRadialPose(M_PI/12,M_PI/2,lobj)){
                if(lobj.isObjInTrack() && lobj.closestPoint.r < MOTOR_DANGER_DIST){
#ifdef DEBUG_MOTOR
                ROS_INFO("Detect obstacle on the left: [%f %f]", lobj.closestPoint.r, lobj.closestPoint.theta);
#endif
                    msg.push_back(OBSTACLE_AHEAD);
                    return false;
                }
            }
            if(laserProcessPtr->findClosestObjectRadialPose(-M_PI/2,-M_PI/12,lobj)){
                if(lobj.isObjInTrack() && lobj.closestPoint.r < MOTOR_DANGER_DIST){
#ifdef DEBUG_MOTOR
                ROS_INFO("Detect obstacle on the right: [%f %f]", lobj.closestPoint.r, lobj.closestPoint.theta);
#endif
                    msg.push_back(OBSTACLE_AHEAD);
                    return false;
                }
            }
        }
        targetPoseInRobot = trafo_World2Base.inverse() * targetPoseInWorld;
        vel_msg.angular.z = 8*angVel * atan2(targetPoseInRobot.getY(), targetPoseInRobot.getX());
        vel_msg.linear.x = linVel * sqrt(pow(targetPoseInRobot.getX(), 2) + pow(targetPoseInRobot.getY(), 2));
        velocityPub.publish(vel_msg);
    }
}

void gameMotorControl::testDataReceiptInGame(){
    while(ros::ok()){
        getRobotWorldPose();
        ROS_INFO_STREAM("robot angle: "<<robotWorldAngle);
        ROS_INFO_STREAM("robot pose: "<<robotWorldCoor.x << "  "<<robotWorldCoor.y);
        ROS_INFO_STREAM("robot orient: "<<robotWorldOrient.x << "  "<<robotWorldOrient.y);
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        cam3DProcessPtr->Img_queue.callOne();
        cam3DProcessPtr->PtCloud_queue.callOne();
        laserProcessPtr->Laser_queue.callOne();
#endif
        /*
        if(laserProcessPtr->isLaserDataAvailable){
            laserProcessPtr->showObjectsPose();
            laserProcessPtr->isLaserDataAvailable = false;
        }

        if(cam3DProcessPtr->isImgDataReady()){
            cam3DProcessPtr->detectObject3D(hockeyField.teamColor);
            cam3DProcessPtr->printObjects();
            cam3DProcessPtr->setImgDataDirty();
        }*/
        //sample_rate.sleep();
    }
}

void gameMotorControl::getRobotWorldPose(){
    try{
        listener.waitForTransform("/world", "/base_footprint", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/world", "/base_footprint", ros::Time(0), trafo_World2Base);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
    robotWorldAngle = trafo_World2Base.getRotation().getAxis().getZ() * trafo_World2Base.getRotation().getAngle();
    if(robotWorldAngle>M_PI) robotWorldAngle -= 2*M_PI;
    robotWorldOrient.x = cos(robotWorldAngle);
    robotWorldOrient.y = sin(robotWorldAngle);
    robotWorldCoor.x = trafo_World2Base.getOrigin().getX();
    robotWorldCoor.y = trafo_World2Base.getOrigin().getY();
}

bool gameMotorControl::isRobotAtBorder(){
    return robotWorldCoor.x>hockeyField.b-hockeyField.delta_b ||
            robotWorldCoor.x<hockeyField.delta_b ||
            robotWorldCoor.y<hockeyField.delta_a ||
            robotWorldCoor.y>3*hockeyField.a - hockeyField.delta_a;
}

bool gameMotorControl::isRobotOutofBorder(){
    return robotWorldCoor.x>hockeyField.b ||
            robotWorldCoor.x<0 ||
            robotWorldCoor.y<0 ||
            robotWorldCoor.y>3*hockeyField.a;
}

bool gameMotorControl::getClosestDistToBorder(){
    distToBorder.clear();
    radialCoordinate rpose;
    vector<tf::Vector3> borderWorldCoor(4);
    vector<tf::Vector3> borderRobotCoor(4);
    borderWorldCoor[0].setValue(robotWorldCoor.x, 0, 0);
    borderRobotCoor[0] = trafo_World2Base.inverse() * borderWorldCoor[0];
    rpose.r = sqrt(pow(borderRobotCoor[0].getX(),2) + pow(borderRobotCoor[0].getY(),2));
    rpose.theta = -robotWorldAngle-M_PI/2;
    if(rpose.r < hockeyField.delta_a){
        distToBorder.push_back(rpose);
    }

    borderWorldCoor[1].setValue(hockeyField.b, robotWorldCoor.y, 0);
    borderRobotCoor[1] = trafo_World2Base.inverse() * borderWorldCoor[1];
    rpose.r = sqrt(pow(borderRobotCoor[1].getX(),2) + pow(borderRobotCoor[1].getY(),2));
    rpose.theta = -robotWorldAngle;
    if(rpose.r < hockeyField.delta_a){
        distToBorder.push_back(rpose);
    }

    borderWorldCoor[2].setValue(robotWorldCoor.x, 3*hockeyField.a, 0);
    borderRobotCoor[2] = trafo_World2Base.inverse() * borderWorldCoor[2];
    rpose.r = sqrt(pow(borderRobotCoor[2].getX(),2) + pow(borderRobotCoor[2].getY(),2));
    rpose.theta = M_PI/2-robotWorldAngle;
    if(rpose.r < hockeyField.delta_a){
        distToBorder.push_back(rpose);
    }

    borderWorldCoor[3].setValue(0, robotWorldCoor.y, 0);
    borderRobotCoor[3] = trafo_World2Base.inverse() * borderWorldCoor[3];
    rpose.r = sqrt(pow(borderRobotCoor[3].getX(),2) + pow(borderRobotCoor[3].getY(),2));
    rpose.theta = M_PI-robotWorldAngle;
    if(rpose.r < hockeyField.delta_a){
        distToBorder.push_back(rpose);
    }

    if(distToBorder.size() != 0){
        return true;
    }else{
        return false;
    }
}

bool gameMotorControl::closeToBorder_front(radialCoordinate &dist){
    bool isFound(false);
    dist.r = hockeyField.delta_a;
    for(vector<radialCoordinate>::iterator it=distToBorder.begin(); it!=distToBorder.end(); ++it){
        if(it->theta > -M_PI/2 && it->theta <= M_PI/2 && it->r < dist.r){
            dist.r=it->r; dist.theta=it->theta;
            isFound = true;
        }
    }
    return isFound;
}

bool gameMotorControl::closeToBorder_back(radialCoordinate &dist){
    bool isFound(false);
    dist.r = hockeyField.delta_a;
    for(vector<radialCoordinate>::iterator it=distToBorder.begin(); it!=distToBorder.end(); ++it){
        if(((it->theta > M_PI/2 && it->theta <= M_PI) ||
                (it->theta > -M_PI && it->theta <= -M_PI/2)) &&
                it->r < dist.r){
            dist.r=it->r; dist.theta=it->theta;
            isFound = true;
        }
    }
    return isFound;
}

bool gameMotorControl::closeToBorder_left(radialCoordinate &dist){
    bool isFound(false);
    dist.r = hockeyField.delta_a;
    for(vector<radialCoordinate>::iterator it=distToBorder.begin(); it!=distToBorder.end(); ++it){
        if(it->theta > 0 && it->theta <= M_PI && it->r < dist.r){
            dist.r=it->r; dist.theta=it->theta;
            isFound = true;
        }
    }
    return isFound;
}

bool gameMotorControl::closeToBorder_right(radialCoordinate &dist){
    bool isFound(false);
    dist.r = hockeyField.delta_a;
    for(vector<radialCoordinate>::iterator it=distToBorder.begin(); it!=distToBorder.end(); ++it){
        if(it->theta > -M_PI && it->theta <= 0 && it->r < dist.r){
            dist.r=it->r; dist.theta=it->theta;
            isFound = true;
        }
    }
    return isFound;
}

bool gameMotorControl::hasReachedGoalDegree()
{
    std::unique_lock<std::mutex> lock(mutexPose);
    #ifdef DEBUG_ON
    ROS_INFO("Goal: %f, Pos: %f", goalTheta, theta);
    #endif
    float a = fabsf(goalTheta - theta);
    return a < 0.015; // about 1 degree
}

// Send a velocity command v
void gameMotorControl::moveForward()
{ 
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = FORWARD_SPEED_MPS;
    velocityPub.publish(msg);
};

// Send a turn command
void gameMotorControl::turn(float degree)
{
    bool turnLeft;
    float rad = 2*PI/360*degree;
    if(rad > 0)
        turnLeft = true;
    else {
        turnLeft = false;
    }

    #ifdef SINGLE_FIFO_TH_DATAREQ
    ros::spinOnce();
    #endif
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    if (turnLeft == true) {
        #ifdef DEBUG_ON
        ROS_INFO("Turn left!");
        #endif
        msg.angular.z = TURN_PARAM;
        goalTheta = fmod(theta + rad, 2*PI);
        if(goalTheta > PI)
            goalTheta = goalTheta - 2*PI;
    }
    else {
        #ifdef DEBUG_ON
        ROS_INFO("Turn right!");
        #endif
        msg.angular.z = -TURN_PARAM;
        goalTheta = fmod(theta + rad, 2*PI);
        if(-goalTheta > PI)
            goalTheta = goalTheta + 2*PI;
    }
  
    ros::Rate r(30);
    while (!hasReachedGoalDegree()) {
        velocityPub.publish(msg);
        #ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
        #endif
    }
}

void gameMotorControl::findPuck(Color objcolor)
{
    std::vector<CamObject3D> obj;
    ROS_INFO("Find Puck");
    ros::Rate r(20);
    while(ros::ok())
    {
        if(cam3DProcessPtr->isImgDataReady()){
            cam3DProcessPtr->detectObject3D(objcolor,obj);
	    if(!obj.empty())
	    {	
                for(size_t i=0, iend=obj.size(); i<iend; i++)
                {   
                    #ifdef DEBUG_ON
                    ROS_INFO("Object[%f], x:%f, y:%f, z:%f\n", i, obj[i].m3DPosition.x, obj[i].m3DPosition.y, obj[i].m3DPosition.z);
                    #endif
		    if(fabsf(obj[i].m3DPosition.x)<0.2 && fabsf(obj[i].m3DPosition.y)<2.5 && obj[i].m3DPosition.z<2 && obj[i].m3DPosition.z !=0)
	            {
                        //#ifdef DEBUG_ON		
		        ROS_INFO("Object[%f], Puck found at x:%f, y:%f, z:%f\n", obj[i].m3DPosition.x, obj[i].m3DPosition.y, obj[i].m3DPosition.z);
                        //#endif	
		        return;
	            }
                }		
	    }
	    turn(4);
	}
	#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
        #endif
    }
}

// Allign to the center of a Puck
void gameMotorControl::allignToCenter(Color objcolor)
{
    ROS_INFO("Allign to center");
    double start_time =ros::Time::now().toSec();
    double current_time;
    double duration;
    std::vector<CamObject3D> obj;
    ros::Rate r(10);
    int closestObject = 0;
    while(ros::ok())
    {
        if(cam3DProcessPtr->isImgDataReady()){
            cam3DProcessPtr->detectObject3D(objcolor,obj);
	    if(!obj.empty())
	    {
                current_time =ros::Time::now().toSec();
                duration = current_time - start_time;
		closestObject=0;
                for(size_t i=1, iend=obj.size(); i<iend; i++)
                {
		    if(obj[i].m3DPosition.z < obj[closestObject].m3DPosition.z && obj[i].m3DPosition.z != 0)
		    {
                        closestObject = i;
		    }
                }	        

	        if(obj[closestObject].m3DPosition.x>0 && fabsf(obj[closestObject].m3DPosition.x)>0.015 && obj[closestObject].m3DPosition.z !=0 && duration < 20)
	        {					
		    turn(-1);
	        }
                else if(obj[closestObject].m3DPosition.x<0 && fabsf(obj[closestObject].m3DPosition.x)>0.015 && obj[closestObject].m3DPosition.z !=0 && duration < 20)
	        {			
	            turn(1);
	        }
	        else if(obj[closestObject].m3DPosition.z !=0 || duration >= 20)
	        {
                    if(duration >= 20)
                        ROS_INFO("Timeout");
                    ROS_INFO("Turned to center of the Object");
	            #ifdef DEBUG_ON
                    ROS_INFO("Object[%f], Puck found at x:%f, y:%f, z:%f\n", obj[closestObject].m3DPosition.x, obj[closestObject].m3DPosition.y, obj[closestObject].m3DPosition.z);	
                    #endif
		    return;
	        }
	    }
        }
	#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
        #endif
    }
}

// Move to the puck
void gameMotorControl::moveToObject()
{
    ROS_INFO("Move to Object");
    laserObject lobj;
    laserProcessPtr->findClosestObjectRadialPose(-M_PI/6,M_PI/6,lobj);
    while(lobj.closestPoint.r > 0.7){
        moveForward();
	laserProcessPtr->findClosestObjectRadialPose(-M_PI/6,M_PI/6,lobj);
        #ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
        #endif
    }
    // Near field allignment with laser
    while(fabsf(lobj.closestPoint.theta)>M_PI/90){
	#ifdef DEBUG_ON
        ROS_INFO("Theta closest point: %f",lobj.closestPoint.theta);
        #endif
        laserProcessPtr->findClosestObjectRadialPose(-M_PI/6,M_PI/6,lobj);
        if(lobj.closestPoint.theta<0)
            turn(-1);
        else
            turn(1);
    }
    // Catch Puck
    while(lobj.closestPoint.r > 0.3){
        moveForward();
	laserProcessPtr->findClosestObjectRadialPose(-M_PI/6,M_PI/6,lobj);
        #ifdef DEBUG_ON
        ROS_INFO("Closest Point %f", lobj.closestPoint.r);
        #endif
        #ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
        #endif
    }
    ROS_INFO("Puck catched");
}
