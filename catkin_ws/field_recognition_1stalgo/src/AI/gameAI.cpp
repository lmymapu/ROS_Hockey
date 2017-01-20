#include "gameAI.h"


void gameAI::stateTransition(){
    switch(stat){
    case SEARCH_PUCK:
        if(existsMessage(ALL_PUCKS_IN_GATE) || existsMessage(TIME_OUT)){
            stat=GAME_OVER;
            break;
        }
        if(existsMessage(FOUND_A_PUCK)){
            stat=CATCH_PUCK;
            break;
        }
    case CATCH_PUCK:
        if(existsMessage(TIME_OUT)){
            stat=GAME_OVER;
            break;
        }
        if(existsMessage(PUCK_OUT_OF_SIGHT)){
            stat=SEARCH_PUCK;
            break;
        }
        if(existsMessage(PUCK_CATCHED)){
            stat=SHOOT_PUCK;
            break;
        }
    case SHOOT_PUCK:
        if(existsMessage(TIME_OUT)){
            stat=GAME_OVER;
            break;
        }
        if(existsMessage(GOAL)){
            stat=DETACH_PUCK;
            break;
        }
        if(existsMessage(PUCK_LOST)){
            stat=SEARCH_PUCK;
            break;
        }
    case DETACH_PUCK:
        if(existsMessage(TIME_OUT)){
            stat=GAME_OVER;
            break;
        }
        if(existsMessage(PUCK_DETACHED)){
            stat=SEARCH_PUCK;
            break;
        }
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
    bool toStartLaser(true);
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
#ifdef MULTI_FIFO_DATAREQ
        cam3DProcess.Img_queue.callOne();
        cam3DProcess.PtCloud_queue.callOne();
        laserProcess.Laser_queue.callOne();
#endif

        if(laserProcess.isLaserDataAvailable){
            laserProcess.showObjectsPose();
            laserProcess.isLaserDataAvailable = false;
        }

        if(!cam3DProcess.isEmpty()){
            cam3DProcess.detectObject(green);
            cam3DProcess.printObjects();

            //cam3DProcess.setImgDataDirty();
        }
        count++;
        if(count>100) break;
        sample_rate.sleep();
    }
    toStartLaser=!toStartLaser;
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

    motorProcess.initGameMotor(&laserProcess, &cam3DProcess);
    ROS_INFO("Hockey Game Started!");
#ifdef TEST_MODE
    testDataReceipt();
#else
    while(1){
        stateTransition();
        if(stat=GAME_OVER){
#ifdef MULTI_FIFO_TH_DATAREQ
            laser_request.stop();
            cam3DImg_request.stop();
            cam3DPtCloud_request.stop();
#endif
#ifdef MULTI_TH_DATAREQ
            data_request.stop();
#endif
            ROS_INFO("Hockey Game Over!");
            break;
        }
    }
#endif
}
