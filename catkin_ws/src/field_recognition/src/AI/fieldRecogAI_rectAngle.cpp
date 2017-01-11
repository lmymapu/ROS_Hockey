#include "fieldRecogAI.h"
#ifdef MULTI_TH_DATAREQ
fieldRecogAI::fieldRecogAI():data_request(NUM_OF_DATAREQ_THREAD)
{
    stat = start_pos0;
    isRecogJobFinished = false;
    isMovingFinished = false;
    xAxisInOdom.x = 0; xAxisInOdom.y = -1;
    yAxisInOdom.x = 1; yAxisInOdom.y = 0;
    send_trafo = AI_node.serviceClient<field_recognition::convert_frames>("convertFrames");
    //motorProcessPtr = new motorControl(laserProcess, cameraProcess);
}
#endif

#ifdef MULTI_FIFO_TH_DATAREQ
fieldRecogAI::fieldRecogAI():laser_request(1, &(laserProcess.Laser_queue)),cam2D_request(1, &(cameraProcess.Cam2D_queue))
{
    stat = start_pos0;
    isRecogJobFinished = false;
    isMovingFinished = false;
    xAxisInOdom.x = 0; xAxisInOdom.y = -1;
    yAxisInOdom.x = 1; yAxisInOdom.y = 0;
    send_trafo = AI_node.serviceClient<field_recognition::convert_frames>("convertFrames");
    //motorProcessPtr = new motorControl(laserProcess, cameraProcess);
}
#endif

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
        if(isRecogJobFinished){
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
        if(isRecogJobFinished){
            stat = move_23;
            isRecogJobFinished = false;
        }
        break;
    case move_23:
        if(isMovingFinished){
            stat = halt_pos3;
            isMovingFinished = false;
        }
        break;
    case halt_pos3:
        if(isRecogJobFinished){
            stat = move_30;
            isRecogJobFinished = false;
        }
        break;
    case move_30:
        if(isMovingFinished){
            stat = end_pos0;
            isMovingFinished = false;
        }
        break;
    case end_pos0:
        break;
    default:
        break;
    }
}

void fieldRecogAI::stateAction(){
    cartesianCoordinate tarVec, tarPose;
    radialCoordinate objRadialPose;
    cartesianCoordinate objRobotCartPose, objOdomCartPose;
    PostInMap postObj(OBJ_POST_RADIUS);
    double objOffsetInPic;

    switch(stat){
    case start_pos0:
        //turn left by 180 degree
        motorProcess.rotateBeyondOdomVector(RECAI_NORMAL_ANGVEL, cartesianCoordinate(-1,0));
        isRecogJobFinished = true;
        break;
    case move_01:
        motorProcess.rotateUntilNoObjInWindow(-RECAI_NORMAL_ANGVEL,green, CamLaserGuide(0, 0.55, 0, 0, green));
        motorProcess.moveToLeftMostObj(RECAI_NORMAL_LINVEL, -RECAI_NORMAL_ANGVEL, RECAI_MINDIST_OBJ, CamLaserGuide(0.5,1,-M_PI/9,M_PI/4,green));
        isMovingFinished = true;
        break;
    case halt_pos1:
        //recognise the nearest object with laser
        if(laserProcess.findLeftMostObjectRadialPose_blk(-M_PI/2, M_PI/6, objRadialPose)){
            objOdomCartPose = calculatePostOdomPose(objRadialPose);
            postObj.setValue(0, green, post, objOdomCartPose);
            hockeyField.postObjs[0] = postObj;
        }else{
            ROS_ERROR("Post 0 detection failed!");
        }
        isRecogJobFinished = true;
        break;
    case move_12:
        //turn left by 90°.
        motorProcess.rotateBeyondOdomVector(RECAI_NORMAL_ANGVEL, cartesianCoordinate(1,-1));
        motorProcess.moveBeyondOdomPose(RECAI_NORMAL_LINVEL, cartesianCoordinate(0,0));
        motorProcess.rotateBeyondOdomVector(-RECAI_NORMAL_ANGVEL, cartesianCoordinate(-1,0));
        motorProcess.rotateUntilNoObjInWindow(RECAI_NORMAL_ANGVEL,green, CamLaserGuide(0.45, 1, 0, 0, green));
        motorProcess.moveToRightMostObj(RECAI_NORMAL_LINVEL, RECAI_NORMAL_ANGVEL, RECAI_MINDIST_OBJ, CamLaserGuide(0,0.5,-M_PI/4,M_PI/9,green));
        isMovingFinished = true;
        break;
    case halt_pos2:
        //recognise the first object on the right
        if(laserProcess.findRightMostObjectRadialPose_blk(-M_PI/6, M_PI/2, objRadialPose)){
            objOdomCartPose = calculatePostOdomPose(objRadialPose);
            postObj.setValue(1, green, post, objOdomCartPose);
            hockeyField.postObjs[1] = postObj;
        }else{
            ROS_ERROR("Post 1 detection failed!");
        }
        xAxisInOdom = calculateUnitVec(hockeyField.postObjs[1].poseInOdom, hockeyField.postObjs[0].poseInOdom);
        yAxisInOdom = getYfromX(xAxisInOdom);
        isRecogJobFinished = true;
        break;
    case move_23:
        motorProcess.rotateBeyondOdomVector(-RECAI_NORMAL_ANGVEL, yAxisInOdom - xAxisInOdom);
        motorProcess.moveBeyondOdomPose(RECAI_NORMAL_LINVEL, cartesianCoordinate(0,0));
        motorProcess.rotateBeyondOdomVector(RECAI_NORMAL_ANGVEL, -xAxisInOdom);
        motorProcess.moveToLeftMostObj_Simple(RECAI_NORMAL_LINVEL, -RECAI_NORMAL_ANGVEL, RECAI_MINDIST_OBJ, CamLaserGuide(0.5,1,-M_PI/9,M_PI/4,green));
        isMovingFinished = true;
        break;
    case halt_pos3:
        //recognise the first object on the right
        if(laserProcess.findLeftMostObjectRadialPose_blk(-M_PI/2, M_PI/6, objRadialPose)){
            objOdomCartPose = calculatePostOdomPose(objRadialPose);
            postObj.setValue(4, green, post, objOdomCartPose);
            hockeyField.postObjs[4] = postObj;
        }else{
            ROS_ERROR("Post 1 detection failed!");
        }
        hockeyField.b = normVec(hockeyField.postObjs[1].poseInOdom, hockeyField.postObjs[0].poseInOdom);
        hockeyField.a = normVec(hockeyField.postObjs[4].poseInOdom, hockeyField.postObjs[0].poseInOdom);
        isRecogJobFinished = true;
        break;
    case move_30:
        //turn left until roboter orientes perpendicular to y axis
        motorProcess.rotateBeyondOdomVector(-RECAI_NORMAL_ANGVEL,xAxisInOdom);
        //move forward
        motorProcess.moveBeyondOdomPose(RECAI_NORMAL_LINVEL, cartesianCoordinate(0,0));
        motorProcess.rotateBeyondOdomVector(-RECAI_NORMAL_ANGVEL, -yAxisInOdom);
        isMovingFinished = true;
        break;
    case end_pos0:
        determineTeamGateColor();
        activateWorldCoordinate();
        calculateAllPostsPose();
        calculateAllGatesPose();
        isRecogJobFinished = true;
        break;
    }
}

cartesianCoordinate fieldRecogAI::calculatePostOdomPose(cartesianCoordinate robotPose){
    try{
        listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), trafo_Odom2Base);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
    tf::Vector3 objPoseInRobot, objPoseInOdom;
    cartesianCoordinate odomPose;
    objPoseInRobot.setValue(robotPose.x * (1 + OBJ_POST_RADIUS/sqrt(pow(robotPose.x,2)+pow(robotPose.y,2))), robotPose.y * (1 + OBJ_POST_RADIUS/sqrt(pow(robotPose.x,2)+pow(robotPose.y,2))), 0);
    objPoseInOdom = trafo_Odom2Base * objPoseInRobot;
    odomPose.x = objPoseInOdom.getX();
    odomPose.y = objPoseInOdom.getY();
    return odomPose;
}

cartesianCoordinate fieldRecogAI::calculatePostOdomPose(radialCoordinate robotPose){
    try{
        listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), trafo_Odom2Base);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
    tf::Vector3 objPoseInRobot, objPoseInOdom;
    cartesianCoordinate odomPose;
    objPoseInRobot.setValue((robotPose.r + OBJ_POST_RADIUS) * cos(robotPose.theta), (robotPose.r + OBJ_POST_RADIUS) * sin(robotPose.theta), 0);
    objPoseInOdom = trafo_Odom2Base * objPoseInRobot;
    odomPose.x = objPoseInOdom.getX();
    odomPose.y = objPoseInOdom.getY();
    return odomPose;
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
    if(hockeyField.teamColor=yellow){
        hockeyField.blueGate.poseInOdom = cartesianCoordinate(b/2*xAxisInOdom.x, b/2*xAxisInOdom.y) + cartesianCoordinate(3*a/8*yAxisInOdom.x, 3*a/8*yAxisInOdom.y);
        hockeyField.blueGate.width = b/3; hockeyField.blueGate.height = a/4;
        hockeyField.blueGate.convert_Odom2World(trafo_World2Odom);

        hockeyField.yellowGate.poseInOdom = cartesianCoordinate(b/2*xAxisInOdom.x, b/2*xAxisInOdom.y) + cartesianCoordinate(21*a/8*yAxisInOdom.x, 21*a/8*yAxisInOdom.y);
        hockeyField.yellowGate.width = b/3; hockeyField.yellowGate.height = a/4;
        hockeyField.yellowGate.convert_Odom2World(trafo_World2Odom);
    }
    if(hockeyField.teamColor=blue){
        hockeyField.yellowGate.poseInOdom = cartesianCoordinate(b/2*xAxisInOdom.x, b/2*xAxisInOdom.y) + cartesianCoordinate(3*a/8*yAxisInOdom.x, 3*a/8*yAxisInOdom.y);
        hockeyField.yellowGate.width = b/3; hockeyField.yellowGate.height = a/4;
        hockeyField.yellowGate.convert_Odom2World(trafo_World2Odom);

        hockeyField.blueGate.poseInOdom = cartesianCoordinate(b/2*xAxisInOdom.x, b/2*xAxisInOdom.y) + cartesianCoordinate(21*a/8*yAxisInOdom.x, 21*a/8*yAxisInOdom.y);
        hockeyField.blueGate.width = b/3; hockeyField.blueGate.height = a/4;
        hockeyField.blueGate.convert_Odom2World(trafo_World2Odom);
    }
}

void fieldRecogAI::determineTeamPuckColor(){
    double closestYellowPuck, closestBluePuck;
    cameraProcess.isCamDataAvailable = false;
    cameraProcess.rightMostObj_blk(yellow,0,1,closestYellowPuck);
    cameraProcess.rightMostObj_blk(blue,0,1,closestBluePuck);
    if(closestYellowPuck > closestBluePuck){
        hockeyField.teamColor=yellow;
    }else{
        hockeyField.teamColor=blue;
    }
}

void fieldRecogAI::determineTeamGateColor(){
    double objoffset;
    cameraProcess.isCamDataAvailable = false;
    if(cameraProcess.rightMostObj_blk(blue,0,1,objoffset)){
        hockeyField.teamColor=yellow;
    }
    if(cameraProcess.rightMostObj_blk(yellow,0,1,objoffset)){
        hockeyField.teamColor=blue;
    }
}

void fieldRecogAI::testDataReceipt(){
    ros::Rate sample_rate(10);
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
        cameraProcess.Cam2D_queue.callOne();
        laserProcess.Laser_queue.callOne();
#endif
        if(laserProcess.isLaserDataAvailable){
            laserProcess.showObjectsPose();
            laserProcess.isLaserDataAvailable = false;
        }

        if(cameraProcess.isCamDataAvailable){
            cameraProcess.detectObject(green);
            ROS_INFO_STREAM("Number of detected objects: "<< cameraProcess.cObjects.size());
            cameraProcess.isCamDataAvailable = false;
        }
        sample_rate.sleep();
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
    motorProcess.initMotor(&laserProcess, &cameraProcess);
    ROS_INFO("Start recognizing field");
#ifdef TEST_MODE
    testDataReceipt();
//    motorProcess.testDataReceipt();
#else
    while(1){
        stateAction();
        nextStateControl();
        if(isRecogJobFinished && stat == end_pos0){
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
