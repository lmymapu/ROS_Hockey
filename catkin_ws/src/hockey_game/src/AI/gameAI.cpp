#include "gameAI.h"


void gameAI::stateTransition(){
    Color puckColor = hockeyField.teamColor;
    cartesianCoordinate puckWorldPose, goalWorldPose;
    CamObject3D puckCamObj;
    laserObject puckLaserObj;
    PuckInMap puckObj;
    GateInMap gateObj;
    switch(puckColor){
    case bl:
        gateObj = hockeyField.blGate;
        break;
    case yell:
        gateObj = hockeyField.yellGate;
        break;
    default:
        break;
    }

    switch(hockeyField.pucksInGoal){
    case 0:
        goalWorldPose = gateObj.poseInWorld - cartesianCoordinate(hockeyField.b/8, 0);
        break;
    case 1:
        goalWorldPose = gateObj.poseInWorld;
        break;
    case 2:
        goalWorldPose = gateObj.poseInWorld + cartesianCoordinate(hockeyField.b/8, 0);
        break;
    default:
        goalWorldPose = gateObj.poseInWorld;
        break;
    }

    switch(stat){
    case SEARCH_PUCK:
#ifdef DEBUG_STATE_TRANS
        ROS_INFO("SEARCH_PUCK");
#endif
#ifdef EASY_GAME_CONTROL
        motorProcess.findPuck(puckColor);
        motorProcess.allignToCenter(puckColor);
        stat=CATCH_PUCK;
#endif

#ifdef COMPLEX_GAME_CONTROL
        motorProcess.resetAggregateRotation();
        motorProcess.rotateUntil3DObjInWindow(GAMEAI_NORMAL_ANGVEL, puckColor, CamLaserGuide(0.5,0.8,0,0,puckColor),msg_buffer);
        if(existsMessage(FOUND_NO_PUCK)){
            start_laserDrv.call(srv_start);
            laserScanOn = true;
            motorProcess.rotateToRandomVector(1.5*GAMEAI_NORMAL_ANGVEL);
            motorProcess.moveMaxDist(GAMEAI_NORMAL_LINVEL);
            stop_laserDrv.call(srv_stop);
            laserScanOn = false;
            stat=SEARCH_PUCK;                                       //if no puck found, search again
            break;
        }
        if(existsMessage(FOUND_A_PUCK)){
            stat=CATCH_PUCK;
            /*
            if(!cam3DProcessPtr->rightMostObj(puckColor,0.4,0.6,puckCamObj)){
                stat=SEARCH_PUCK;
                break;
            }
            puckObj.setValue(0,puckColor,puck,puckCamObj.getWorldPose(OBJ_PUCK_UP_RADIUS));
            if(!hockeyField.isPuckInGate(puckObj)){
                stat=CATCH_PUCK;
                break;
            }else{
                motorProcess.rotateUntilNo3DObjInWindow(0.5*GAMEAI_NORMAL_ANGVEL, puckColor, CamLaserGuide(0.4, 0.5, 0,0, puckColor), msg_buffer);
                motorProcess.rotateUntilNo3DObjInWindow(0.5*GAMEAI_NORMAL_ANGVEL, puckColor, CamLaserGuide(0.5, 0.6, 0,0, puckColor), msg_buffer);
                if(motorProcess.getAggregateRotation() > 2*M_PI){
                    start_laserDrv.call(srv_start);
                    laserScanOn = true;
                    motorProcess.rotateToRandomVector(1.5*GAMEAI_NORMAL_ANGVEL);
                    motorProcess.moveMaxDist(GAMEAI_NORMAL_LINVEL);
                    stop_laserDrv.call(srv_stop);
                    laserScanOn = false;
                }
                stat=SEARCH_PUCK;
                break;
            }*/
        }
#endif
    case CATCH_PUCK:
#ifdef DEBUG_STATE_TRANS
        ROS_INFO("CATCH_PUCK");
#endif
#ifdef EASY_GAME_CONTROL
        start_laserDrv.call(srv_start);
        laserScanOn = true;
        motorProcess.moveToObject();
        stop_laserDrv.call(srv_stop);
        laserScanOn = false;
        stat=SHOOT_PUCK;
#endif
#ifdef COMPLEX_GAME_CONTROL
        start_laserDrv.call(srv_start);
        laserScanOn = true;
        motorProcess.moveAndCatchObj_simple(GAMEAI_NORMAL_LINVEL, GAMEAI_NORMAL_ANGVEL, CamLaserGuide(0.5,0.8,-M_PI/2,M_PI/2,puckColor), msg_buffer);
        stop_laserDrv.call(srv_stop);
        laserScanOn = false;
        if(existsMessage(PUCK_CATCHED)){
            stat=SHOOT_PUCK;
            break;
        }
        if(existsMessage(PUCK_OUT_OF_SIGHT)){
            stat=SEARCH_PUCK;
            break;
        }
        if(existsMessage(PUCK_CATCHED_ERROR)){
            stat=DETACH_PUCK;
            break;
        }
        break;
#endif
    case SHOOT_PUCK:
#ifdef DEBUG_STATE_TRANS
        ROS_INFO("SHOOT_PUCK");
#endif
        start_laserDrv.call(srv_start);
        laserScanOn = true;
        laserProcessPtr->setLaserDataDirty();
        if(!laserProcessPtr->findClosestObjectRadialPose_blk(-M_PI/12,M_PI/12,puckLaserObj)){
            stat=SEARCH_PUCK;
            break;
        }else{
            puckObj.poseInWorld = puckLaserObj.getWorldPose(OBJ_PUCK_LOW_RADIUS);
            if(hockeyField.isPuckInGate(puckObj)){
                stat=DETACH_PUCK;
                break;
            }
        }
        motorProcess.rotateAndPointToWorldVec(GAMEAI_NORMAL_ANGVEL, goalWorldPose);
        motorProcess.movePuckToGoal_simple(GAMEAI_NORMAL_LINVEL, GAMEAI_NORMAL_ANGVEL, OBJ_PUCK_LOW_RADIUS, goalWorldPose, msg_buffer);
        stop_laserDrv.call(srv_stop);
        laserScanOn = false;
        if(existsMessage(PUCK_LOST)){
            stat=SEARCH_PUCK;
            break;
        }
        if(existsMessage(GOAL)){
            hockeyField.pucksInGoal++;
            if(hockeyField.pucksInGoal == 3){
                stat = GAME_OVER;
            }else{
                stat=DETACH_PUCK;
            }
            break;
        }
        break;
    case DETACH_PUCK:
#ifdef DEBUG_STATE_TRANS
        ROS_INFO("DETACH_PUCK");
#endif
        start_laserDrv.call(srv_start);
        laserScanOn = true;
        motorProcess.rotateUntilNoObjAtBack(GAMEAI_NORMAL_ANGVEL);
        motorProcess.moveBeyondDist(-GAMEAI_NORMAL_LINVEL, 0.2);
        motorProcess.rotateBeyondWorldVector(GAMEAI_NORMAL_ANGVEL, cartesianCoordinate(-1,-1));
        stop_laserDrv.call(srv_stop);
        laserScanOn = false;
        stat=SEARCH_PUCK;
        break;
    case GAME_OVER:
        break;
    }
}

bool gameAI::existsMessage(const controlMessage msg){
    for(vector<controlMessage>::iterator it=msg_buffer.begin(); it!=msg_buffer.end(); ++it){
        if(*it == msg) return true;
    }
    return false;
}

void gameAI::testDataReceipt(){
    ros::Rate sample_rate(10);
    laserObject lobj;
    cartesianCoordinate cPose;
    bool existsLobj(false);

    start_laserDrv.call(srv_start);
    laserScanOn = true;
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

        if(laserProcessPtr->isLaserDataAvailable){
            laserProcessPtr->showObjectsPose();
            existsLobj = laserProcessPtr->findClosestObjectRadialPose(-M_PI/2, M_PI/2, lobj);
            cPose=lobj.getCamPose();
            ROS_INFO("laser object in CAM: [%f %f]", cPose.x, cPose.y);
        }

        if(cam3DProcessPtr->isImgDataReady()){
            cam3DProcessPtr->detectObject3D(green);
            cam3DProcessPtr->printObjects();

            //cam3DProcessPtr->setImgDataDirty();
        }
        sample_rate.sleep();
    }

}

void gameAI::startFighting(){
#ifdef MULTI_TH_DATAREQ
    data_request.start();
#endif
#ifdef MULTI_FIFO_TH_DATAREQ
    laser_request.start();
    cam3DImg_request.start();
    cam3DPtCloud_request.start();
#endif
    motorProcess.getMap(hockeyField);
    motorProcess.initGameMotor(laserProcessPtr, cam3DProcessPtr);
    laserProcessPtr->setDetectRange(LASER_MAX_DETECT_RANGE);
    ROS_INFO("Hockey Game Started!");
#ifdef TEST_MODE
    start_laserDrv.call(srv_start);
    laserScanOn = true;
    motorProcess.rotateAndPointToWorldVec(GAMEAI_NORMAL_ANGVEL, hockeyField.yellGate.poseInWorld);
    motorProcess.movePuckToGoal_simple(GAMEAI_NORMAL_LINVEL, GAMEAI_NORMAL_ANGVEL, OBJ_PUCK_LOW_RADIUS, hockeyField.yellGate.poseInWorld, msg_buffer);
    stop_laserDrv.call(srv_stop);
    laserScanOn = false;
#else
    while(1){
        stateTransition();
        if(stat==GAME_OVER){
#ifdef MULTI_TH_DATAREQ
            data_request.stop();
#endif
            ROS_INFO("Hockey Game Over!");
            break;
        }
    }
#endif
}
