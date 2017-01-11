#include "gameAI.h"
#ifdef MULTI_TH_DATAREQ
gameAI::gameAI(const Map &field):stat(SEARCH_PUCK),data_request(NUM_OF_DATAREQ_THREAD)
{
    hockeyField = field;
}
#else
gameAI::gameAI(const Map &field):stat(SEARCH_PUCK)
{
    hockeyField = field;
}
#endif

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
    while(ros::ok()){
        try{
            listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), trafo_Odom2Base);
        }catch(tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }
        ROS_INFO_STREAM("robot pose: "<<trafo_Odom2Base.getOrigin().getX() << "  "<<trafo_Odom2Base.getOrigin().getY());
#ifndef MULTI_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        ros::spinOnce();
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
        sample_rate.sleep();
    }

}

void gameAI::startFighting(){
//    data_request.start();
    motorProcess.initGameMotor(&laserProcess, &cam3DProcess);
    ROS_INFO("Hockey Game Started!");
#ifdef TEST_MODE
    testDataReceipt();
#else
    while(1){
        stateTransition();
        if(stat=GAME_OVER){
            ROS_INFO("Hockey Game Over!");
            break;
        }
    }
#endif
}
