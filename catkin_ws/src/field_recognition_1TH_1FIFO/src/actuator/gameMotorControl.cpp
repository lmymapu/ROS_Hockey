#include "gameMotorControl.h"

gameMotorControl::gameMotorControl()
{

}

void gameMotorControl::testDataReceiptInGame(){
    while(ros::ok()){
        try{
            listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), trafo_Odom2Base);
        }catch(tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }
        ROS_INFO_STREAM("robot pose: "<<trafo_Odom2Base.getOrigin().getX() << "  "<<trafo_Odom2Base.getOrigin().getY());
 //       ros::spinOnce();
        if(laserProcessPtr->isLaserDataAvailable){
            laserProcessPtr->showObjectsPose();
            laserProcessPtr->isLaserDataAvailable = false;
        }

        if(cam3DProcessPtr->isImgDataReady()){
            cam3DProcessPtr->detectObject(green);
            cam3DProcessPtr->printObjects();
            cam3DProcessPtr->setImgDataDirty();
        }
        sample_rate.sleep();
    }
}
