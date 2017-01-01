#include "fieldRecogAI.h"

fieldRecogAI::fieldRecogAI():laserProcess(),cameraProcess(),motorProcess()
{
    stat = start_pos0;
    isRecogJobFinished = false;
    isMovingFinished = false;
    xAxisInOdom.x = 0; xAxisInOdom.y = -1;
    yAxisInOdom.x = 1; yAxisInOdom.y = 0;
    //motorProcessPtr = new motorControl(laserProcess, cameraProcess);
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
    ObjectInMap postObj;
    double objOffsetInPic;

    switch(stat){
    case start_pos0:
        //turn left by 180 degree
        tarVec.setVal(-1,0);
        motorProcess.rotateToOdomVector(NORMAL_ANGULAR_VEL, tarVec);
        //slowly rotate right, until the camera captures a green post at front
        //motorProcess.rotateUntilObjInMiddle(-FINECTRL_ANGULAR_VEL, green);
        isRecogJobFinished = true;
        break;
    case move_01:
        //move forward until the nearest obj is within a minimal range
        motorProcess.moveToLeftMostObj(NORMAL_LINEAR_VEL, -NORMAL_ANGULAR_VEL, MINDIST_OBJECT, CamLaserGuide(0.5,1,-M_PI/9,M_PI/4,green));
        isMovingFinished = true;
        break;
    case halt_pos1:
        //recognise the nearest object with laser
        if(laserProcess.findLeftMostObjectRadialPose_blk(-M_PI/2, M_PI/6, objRadialPose)){
            objRobotCartPose = radial2cart(objRadialPose);
            objOdomCartPose = calculateObjOdomPose(objRobotCartPose);
            postObj.setValue(0, green, post, objOdomCartPose);
            hockeyField.postObjs[0] = postObj;
        }else{
            ROS_ERROR("Post 0 detection failed!");
        }
        isRecogJobFinished = true;
        break;
    case move_12:
        //turn left by 90°.
        tarVec.setVal(1,-1);
        motorProcess.rotateToOdomVector(-NORMAL_ANGULAR_VEL, tarVec);
        tarPose.setVal(0,0);
        motorProcess.moveToOdomPose(NORMAL_LINEAR_VEL, tarPose);
        tarVec.setVal(-1,0);
        motorProcess.rotateToOdomVector(-NORMAL_ANGULAR_VEL, tarVec);
        motorProcess.moveToRightMostObj(NORMAL_LINEAR_VEL, NORMAL_ANGULAR_VEL, MINDIST_OBJECT, CamLaserGuide(0,0.5,-M_PI/4,M_PI/9,green));
        isMovingFinished = true;
        break;
    case halt_pos2:
        //recognise the first object on the right
        if(laserProcess.findRightMostObjectRadialPose_blk(-M_PI/6, M_PI/2, objRadialPose)){
            objRobotCartPose = radial2cart(objRadialPose);
            objOdomCartPose = calculateObjOdomPose(objRobotCartPose);
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
        motorProcess.rotateToOdomVector(NORMAL_ANGULAR_VEL, yAxisInOdom - xAxisInOdom);
        tarPose.setVal(0,0);
        motorProcess.moveToOdomPose(NORMAL_LINEAR_VEL, tarPose);
        motorProcess.rotateToOdomVector(NORMAL_ANGULAR_VEL, -xAxisInOdom);
        motorProcess.moveToLeftMostObj(NORMAL_LINEAR_VEL, -NORMAL_ANGULAR_VEL, MINDIST_OBJECT, CamLaserGuide(0.5,1,-M_PI/9,M_PI/4,green));
        isMovingFinished = true;
        break;
    case halt_pos3:
        //recognise the first object on the right
        if(laserProcess.findLeftMostObjectRadialPose_blk(-M_PI/2, M_PI/6, objRadialPose)){
            objRobotCartPose = radial2cart(objRadialPose);
            objOdomCartPose = calculateObjOdomPose(objRobotCartPose);
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
        motorProcess.rotateToOdomVector(-NORMAL_ANGULAR_VEL,xAxisInOdom);
        //move forward
        motorProcess.moveByDist(NORMAL_LINEAR_VEL, hockeyField.b/2);
        motorProcess.rotateToOdomVector(-NORMAL_ANGULAR_VEL, -yAxisInOdom);
        isMovingFinished = true;
        break;
    case end_pos0:
        //turn left until roboter orients along -y-axis
        //recognize the color of gate.
        //recognize team color
        //turn right around until roboter orients along y-axis
        //calculate positions of other posts and register them
        //calculate a, b
        isRecogJobFinished = true;
        break;
    }
}

cartesianCoordinate fieldRecogAI::calculateObjOdomPose(cartesianCoordinate robotPose){
    try{
        listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), trafo_Odom2Base);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
    tf::Vector3 objPoseInRobot, objPoseInOdom;
    cartesianCoordinate odomPose;
    objPoseInRobot.setValue(robotPose.x, robotPose.y, 0);
    objPoseInOdom = trafo_Odom2Base * objPoseInRobot;
    odomPose.x = objPoseInOdom.getX();
    odomPose.y = objPoseInOdom.getY();
    return odomPose;
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
        ros::spinOnce();
        if(laserProcess.isLaserDataAvailable){
            //laserProcess.recogniseObjects();
            //laserProcess.transformOdomCartCoor(trafo_Odom2Base);
            //laserProcess.showObjectsPose();
            laserProcess.showObjectsPose();
            laserProcess.isLaserDataAvailable = false;
        }
        /*
        if(!cameraProcess.mImage.empty()){
            cameraProcess.detectObject(green);
            ROS_INFO_STREAM("Number of detected objects: "<< cameraProcess.cObjects.size());
        }*/
        sample_rate.sleep();
    }
}

void fieldRecogAI::startFieldRecognition(){
    motorProcess.initMotor(&laserProcess, &cameraProcess);
    ROS_INFO("Start recognizing field");
    //motorProcess.moveToLeftMostObj(0.5, 0.5, 0.5, 1, 0.4, green);
   // motorProcess.moveToRightMostObj(0.5, 0.5, 0, 0.5, 0.4, green);
    while(1){
        stateAction();
        nextStateControl();
    }
}
