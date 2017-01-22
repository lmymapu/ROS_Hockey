#include "fieldRecogAI.h"

#ifdef SINGLE_FIFO_TH_DATAREQ
fieldRecogAI::fieldRecogAI()
{
    stat = start_pos0;
    isRecogJobFinished = false;
    isMovingFinished = false;
    xAxisInOdom.x = 0; xAxisInOdom.y = -1;
    yAxisInOdom.x = 1; yAxisInOdom.y = 0;
    send_trafo = AI_node.serviceClient<hockey_game::convert_frames>("convertFrames");
    start_laserDrv = AI_node.serviceClient<std_srvs::Empty>("start_motor");
    stop_laserDrv = AI_node.serviceClient<std_srvs::Empty>("stop_motor");
    //motorProcessPtr = new motorControl(laserProcess, cameraProcess);
}
#endif

#ifdef MULTI_TH_DATAREQ
fieldRecogAI::fieldRecogAI():data_request(NUM_OF_DATAREQ_THREAD)
{
    stat = start_pos0;
    isRecogJobFinished = false;
    isMovingFinished = false;
    xAxisInOdom.x = 0; xAxisInOdom.y = -1;
    yAxisInOdom.x = 1; yAxisInOdom.y = 0;
    send_trafo = AI_node.serviceClient<hockey_game::convert_frames>("convertFrames");
    start_laserDrv = AI_node.serviceClient<std_srvs::Empty>("start_motor");
    stop_laserDrv = AI_node.serviceClient<std_srvs::Empty>("stop_motor");
    //motorProcessPtr = new motorControl(laserProcess, cameraProcess);
}
#endif

fieldRecogAI::fieldRecogAI(laserScanner *lp, turtlebotCamera3D *cp):data_request(NUM_OF_DATAREQ_THREAD)
{
    stat = start_pos0;
    isRecogJobFinished = false;
    isMovingFinished = false;
    xAxisInOdom.x = 0; xAxisInOdom.y = -1;
    yAxisInOdom.x = 1; yAxisInOdom.y = 0;
    send_trafo = AI_node.serviceClient<hockey_game::convert_frames>("convertFrames");
    start_laserDrv = AI_node.serviceClient<std_srvs::Empty>("start_motor");
    stop_laserDrv = AI_node.serviceClient<std_srvs::Empty>("stop_motor");
    //motorProcessPtr = new motorControl(laserProcess, cameraProcess);
    laserProcessPtr = lp; cameraProcessPtr = cp;
}

void fieldRecogAI::nextStateControl(){
    switch(stat){
    case start_pos0:
        if(isRecogJobFinished){                 //move to the next state when recognition job is finished
            stat = move_01;
            isRecogJobFinished = false;
        }
        break;
    case move_01:
        if(isMovingFinished){
            stat = halt_pos1;
            isMovingFinished = false;
        }
        break;
    case halt_pos1:
        if(isRecogJobFinished){                 //move to the next state when recognition job is finished
            stat = move_12;
            isRecogJobFinished = false;
        }
        break;
    case move_12:
        if(isMovingFinished){
            stat = halt_pos2;
            isMovingFinished = false;
        }
        break;
    case halt_pos2:
        break;
    default:
        break;
    }
}

void fieldRecogAI::stateAction(){
    cartesianCoordinate tarVec, tarPose;
    cartesianCoordinate objPostOdomPose;
    double moveLength;
    CamObject3D post3DCam;
    PostInMap postObj(OBJ_POST_RADIUS);
    double objOffsetInPic;

    switch(stat){
    case start_pos0:
        //rotate to post No.0 and detect its pose
        motorProcess.rotateBeyondOdomVector(RECAI_NORMAL_ANGVEL, cartesianCoordinate(0, 1));
        motorProcess.rotateBeyondOdomVector(RECAI_NORMAL_ANGVEL, cartesianCoordinate(-1, 0));
        motorProcess.rotateUntilNoObjInWindow(RECAI_NORMAL_ANGVEL, green, CamLaserGuide(0.5, 1, 0, 0, green));
        motorProcess.rotateUntilObjInWindow(-RECAI_NORMAL_ANGVEL, green, CamLaserGuide(0.4, 0.8, 0, 0, green));
        ros::Duration(1).sleep();
        if(cameraProcessPtr->leftMostObj_blk(green,0.4, 0.8, post3DCam)){
            objPostOdomPose = post3DCam.getOdomPose(OBJ_POST_RADIUS);
            postObj.setValue(0, green, post, objPostOdomPose);
            hockeyField.postObjs[0] = postObj;
#ifdef DEBUG_STATE_TRANS
        cameraProcessPtr->printObjects();
        ROS_INFO("3D camera object :[%f %f]", post3DCam.m3DPositionMedian.z, -post3DCam.m3DPositionMedian.x);
        ROS_INFO("post 0 [%f %f]", postObj.poseInOdom.x, postObj.poseInOdom.y);
#endif
        }else{
            ROS_INFO("post 0 recognition failed");
        }

        //rotate to post No.1 and detect its pose
        motorProcess.rotateUntilNoObjInWindow(RECAI_NORMAL_ANGVEL, green, CamLaserGuide(0.4, 0.8, 0, 0, green));
        motorProcess.rotateUntilObjInWindow(RECAI_NORMAL_ANGVEL, green, CamLaserGuide(0.2, 0.6, 0, 0, green));
        ros::Duration(1).sleep();
        if(cameraProcessPtr->rightMostObj_blk(green, 0.2, 0.6, post3DCam)){
            objPostOdomPose = post3DCam.getOdomPose(OBJ_POST_RADIUS);
            postObj.setValue(1, green, post, objPostOdomPose);
            hockeyField.postObjs[1] = postObj;
#ifdef DEBUG_STATE_TRANS
        cameraProcessPtr->printObjects();
        ROS_INFO("3D camera object :[%f %f]", post3DCam.m3DPositionMedian.z, -post3DCam.m3DPositionMedian.x);
        ROS_INFO("post 1 [%f %f]", postObj.poseInOdom.x, postObj.poseInOdom.y);
#endif
        }else{
            ROS_INFO("post 1 recognition failed");
        }

        //acquire x and y axis of world frame
        xAxisInOdom = calculateUnitVec(hockeyField.postObjs[1].poseInOdom, hockeyField.postObjs[0].poseInOdom);
        yAxisInOdom = getYfromX(xAxisInOdom);
        hockeyField.b = normVec(hockeyField.postObjs[1].poseInOdom, hockeyField.postObjs[0].poseInOdom);
#ifdef DEBUG_STATE_TRANS
        ROS_INFO("xAxis: [%f, %f]", xAxisInOdom.x, xAxisInOdom.y);
        ROS_INFO("yAxis: [%f, %f]", yAxisInOdom.x, yAxisInOdom.y);
        ROS_INFO("b = %f", hockeyField.b);
#endif
        isRecogJobFinished = true;
        break;
    case move_01:
        //rotate to post No.4 and detect its pose
        motorProcess.rotateBeyondOdomVector(RECAI_NORMAL_ANGVEL, xAxisInOdom);
        motorProcess.moveBeyondDist(RECAI_NORMAL_LINVEL,abs(hockeyField.postObjs[1].poseInOdom * xAxisInOdom)-1.2 * OBJ_ROBOT_DIAMETER);
        isMovingFinished = true;
        break;
    case halt_pos1:
        motorProcess.rotateBeyondOdomVector(RECAI_NORMAL_ANGVEL,yAxisInOdom);
        motorProcess.rotateBeyondOdomVector(RECAI_NORMAL_ANGVEL,-xAxisInOdom);
        ros::Duration(1).sleep();
        if(cameraProcessPtr->leftMostObj_blk(green,0.45, 1, post3DCam)){
            objPostOdomPose = post3DCam.getOdomPose(OBJ_POST_RADIUS);
            postObj.setValue(4, green, post, objPostOdomPose);
            hockeyField.postObjs[4] = postObj;
        }else{
            ROS_INFO("post 4 recognition failed");
        }
        hockeyField.a = normVec(hockeyField.postObjs[4].poseInOdom, hockeyField.postObjs[0].poseInOdom);
        isRecogJobFinished = true;
        break;
    case move_12:
        motorProcess.rotateBeyondOdomVector(RECAI_NORMAL_ANGVEL, -yAxisInOdom-xAxisInOdom);
        tarPose = hockeyField.postObjs[0].poseInOdom + cartesianCoordinate(hockeyField.b/2 * xAxisInOdom.x, hockeyField.b/2 * xAxisInOdom.y)
                + cartesianCoordinate(OBJ_ROBOT_DIAMETER * yAxisInOdom.x, OBJ_ROBOT_DIAMETER * yAxisInOdom.y);
        motorProcess.moveBeyondOdomPose(RECAI_NORMAL_LINVEL, tarPose);
        isMovingFinished = true;
    case halt_pos2:
        motorProcess.rotateBeyondOdomVector(-RECAI_NORMAL_ANGVEL, yAxisInOdom);
        ros::Duration(1).sleep();
        determineTeamPuckColor();
        activateWorldCoordinate();
        calculateAllPostsPose();
        calculateAllGatesPose();
        isRecogJobFinished = true;
        break;
    default:
        break;
    }
}

void fieldRecogAI::activateWorldCoordinate(){
    srv_conv.request.xAxis_x = xAxisInOdom.x;
    srv_conv.request.xAxis_y = xAxisInOdom.y;
    srv_conv.request.yAxis_x = yAxisInOdom.x;
    srv_conv.request.yAxis_y = yAxisInOdom.y;
    srv_conv.request.offset_x = hockeyField.postObjs[0].poseInOdom.x;
    srv_conv.request.offset_y = hockeyField.postObjs[0].poseInOdom.y;
    if(send_trafo.call(srv_conv)){
        ROS_INFO_STREAM("Rotation resp.to world coordinate is" << srv_conv.response.theta_degree);
    }else{
        ROS_ERROR("tranformation to world coordinate failed!");
    }
}

void fieldRecogAI::calculateAllPostsPose(){
    double a=hockeyField.a;
    double b=hockeyField.b;
    try{
        listener.waitForTransform("/world", "/odom", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/world", "/odom", ros::Time(0), trafo_World2Odom);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
    hockeyField.postObjs[0].convert_Odom2World(trafo_World2Odom);
    hockeyField.postObjs[1].convert_Odom2World(trafo_World2Odom);
    hockeyField.postObjs[4].convert_Odom2World(trafo_World2Odom);

    hockeyField.postObjs[5].poseInOdom = hockeyField.postObjs[1].poseInOdom + cartesianCoordinate(a*yAxisInOdom.x, a*yAxisInOdom.y);
    hockeyField.postObjs[5].convert_Odom2World(trafo_World2Odom);
    hockeyField.postObjs[6].poseInOdom = hockeyField.postObjs[0].poseInOdom + cartesianCoordinate(3*a/2*yAxisInOdom.x, 3*a/2*yAxisInOdom.y);
    hockeyField.postObjs[6].convert_Odom2World(trafo_World2Odom);
    hockeyField.postObjs[7].poseInOdom = hockeyField.postObjs[1].poseInOdom + cartesianCoordinate(3*a/2*yAxisInOdom.x, 3*a/2*yAxisInOdom.y);
    hockeyField.postObjs[7].convert_Odom2World(trafo_World2Odom);
    hockeyField.postObjs[8].poseInOdom = hockeyField.postObjs[0].poseInOdom + cartesianCoordinate(2*a*yAxisInOdom.x, 2*a*yAxisInOdom.y);
    hockeyField.postObjs[8].convert_Odom2World(trafo_World2Odom);
    hockeyField.postObjs[9].poseInOdom = hockeyField.postObjs[1].poseInOdom + cartesianCoordinate(2*a*yAxisInOdom.x, 2*a*yAxisInOdom.y);
    hockeyField.postObjs[9].convert_Odom2World(trafo_World2Odom);
    hockeyField.postObjs[12].poseInOdom = hockeyField.postObjs[0].poseInOdom + cartesianCoordinate(3*a*yAxisInOdom.x, 3*a*yAxisInOdom.y);
    hockeyField.postObjs[12].convert_Odom2World(trafo_World2Odom);
    hockeyField.postObjs[13].poseInOdom = hockeyField.postObjs[1].poseInOdom + cartesianCoordinate(3*a*yAxisInOdom.x, 3*a*yAxisInOdom.y);
    hockeyField.postObjs[13].convert_Odom2World(trafo_World2Odom);
}

void fieldRecogAI::calculateAllGatesPose(){
    double a=hockeyField.a;
    double b=hockeyField.b;
    try{
        listener.waitForTransform("/world", "/odom", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/world", "/odom", ros::Time(0), trafo_World2Odom);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
    if(hockeyField.teamColor==yell){
        hockeyField.blGate.poseInOdom = hockeyField.postObjs[0].poseInOdom + cartesianCoordinate(b/2*xAxisInOdom.x, b/2*xAxisInOdom.y) + cartesianCoordinate(3*a/8*yAxisInOdom.x, 3*a/8*yAxisInOdom.y);
        hockeyField.blGate.width = b/3; hockeyField.blGate.height = a/4;
        hockeyField.blGate.convert_Odom2World(trafo_World2Odom);

        hockeyField.yellGate.poseInOdom = hockeyField.postObjs[0].poseInOdom + cartesianCoordinate(b/2*xAxisInOdom.x, b/2*xAxisInOdom.y) + cartesianCoordinate(21*a/8*yAxisInOdom.x, 21*a/8*yAxisInOdom.y);
        hockeyField.yellGate.width = b/3; hockeyField.yellGate.height = a/4;
        hockeyField.yellGate.convert_Odom2World(trafo_World2Odom);
    }
    if(hockeyField.teamColor==bl){
        hockeyField.yellGate.poseInOdom = hockeyField.postObjs[0].poseInOdom + cartesianCoordinate(b/2*xAxisInOdom.x, b/2*xAxisInOdom.y) + cartesianCoordinate(3*a/8*yAxisInOdom.x, 3*a/8*yAxisInOdom.y);
        hockeyField.yellGate.width = b/3; hockeyField.yellGate.height = a/4;
        hockeyField.yellGate.convert_Odom2World(trafo_World2Odom);

        hockeyField.blGate.poseInOdom = hockeyField.postObjs[0].poseInOdom + cartesianCoordinate(b/2*xAxisInOdom.x, b/2*xAxisInOdom.y) + cartesianCoordinate(21*a/8*yAxisInOdom.x, 21*a/8*yAxisInOdom.y);
        hockeyField.blGate.width = b/3; hockeyField.blGate.height = a/4;
        hockeyField.blGate.convert_Odom2World(trafo_World2Odom);
    }
}

void fieldRecogAI::determineTeamPuckColor(){
    CamObject3D yellPuck, blPuck;
    bool blPuckdetected(false), yellPuckdetected(false);
    blPuckdetected = cameraProcessPtr->smallestZObj_blk(bl, 0, 1, blPuck);
#ifdef DEBUG_STATE_TRANS
    cameraProcessPtr->printObjects();
#endif
    yellPuckdetected = cameraProcessPtr->smallestZObj_blk(yell, 0, 1, yellPuck);
#ifdef DEBUG_STATE_TRANS
    cameraProcessPtr->printObjects();
#endif
    if(!blPuckdetected && yellPuckdetected){
        hockeyField.teamColor = yell;
        ROS_INFO("detected yell puck, yellPuck Z value: %f", yellPuck.m3DPositionMedian.z);
    }else if(blPuckdetected && !yellPuckdetected){
        hockeyField.teamColor = bl;
        ROS_INFO("detected bl puck, blPuck Z value: %f", blPuck.m3DPositionMedian.z);
    }else if(blPuckdetected && yellPuckdetected){
        ROS_INFO("detected bl & yell puck, yellPuck Z value: %f, blPuck Z value: %f", yellPuck.m3DPositionMedian.z, blPuck.m3DPositionMedian.z);
        if(yellPuck.m3DPositionMedian.z < blPuck.m3DPositionMedian.z){
            hockeyField.teamColor = yell;
        }else{
            hockeyField.teamColor = bl;
        }
    }else{
        ROS_WARN("Warning, no puck detected");
    }
}

void fieldRecogAI::determineTeamGateColor(){

}

void fieldRecogAI::testDataReceipt(){
    ros::Rate sample_rate(10);
    bool toStartLaser(true);
    ros::Duration(3).sleep();
    activateWorldCoordinate();
    while(1){
    int count = 0;
    if(toStartLaser)
        start_laserDrv.call(srv_start);
    else
        stop_laserDrv.call(srv_stop);
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
        }

        if(!cameraProcessPtr->isEmpty()){
            cameraProcessPtr->detectObject3D(bl);
            cameraProcessPtr->printObjects();

            //cam3DProcess.setImgDataDirty();
        }
        count++;
        if(count>100) break;
        sample_rate.sleep();
    }
    toStartLaser=!toStartLaser;
    }

}

void fieldRecogAI::startFieldRecognition(){
#ifdef MULTI_TH_DATAREQ
    data_request.start();
#endif
#ifdef MULTI_FIFO_TH_DATAREQ
    laser_request.start();
    cam2D_request.start();
#endif
    motorProcess.addCam3DSensor(cameraProcessPtr);
    motorProcess.addLaserSensor(laserProcessPtr);
    ROS_INFO("Start recognizing field");
#ifdef TEST_MODE
    testDataReceipt();
//    motorProcess.testDataReceipt();
#else
    while(1){
        stateAction();
        nextStateControl();
        if(isRecogJobFinished && stat == halt_pos2){
#ifdef MULTI_FIFO_TH_DATAREQ
            laser_request.stop();
            cam2D_request.stop();
#endif
#ifdef MULTI_TH_DATAREQ
            data_request.stop();
#endif
            ROS_INFO("Recognition Job completed");

            break;
        }
    }
#endif
}
