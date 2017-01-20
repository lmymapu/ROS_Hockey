#include "laserScanner.h"
#include <boost/bind.hpp>
#include <iostream>
#define RAD2DEG(x) ((x)*180./M_PI)


#define SHOW_RADIAL_COORDINATE

//#define AVERAGE_ANGLE_ALGORITHM
using namespace std;
#ifdef SINGLE_FIFO_TH_DATAREQ
laserScanner::laserScanner():laserMaxDetectRange(LASER_MAX_DETECT_RANGE),sample_rate(10)
{
    isLaserDataAvailable=false;
#ifndef SIMULATION_MODE
    laserSub = node.subscribe("/scan_laser", LASER_FIFO_LENGTH, &laserScanner::scanCallback, this);
#else
    laserSub = node.subscribe("/base_scan_1", LASER_FIFO_LENGTH, &laserScanner::scanCallback, this);
#endif
}
#endif

#ifdef MULTI_TH_DATAREQ
laserScanner::laserScanner():laserMaxDetectRange(LASER_MAX_DETECT_RANGE),sample_rate(10)
{
    isLaserDataAvailable=false;
#ifndef SIMULATION_MODE
    laserSub = node.subscribe("/scan_laser", LASER_FIFO_LENGTH, &laserScanner::scanCallback, this);
#else
    laserSub = node.subscribe("/base_scan_1", LASER_FIFO_LENGTH, &laserScanner::scanCallback, this);
#endif
}
#endif


laserScanner::laserScanner(const laserScanner &lobj):laserMaxDetectRange(LASER_MAX_DETECT_RANGE),sample_rate(10){
    //this->scanPtr = lobj.scanPtr;
    lObjects = lobj.lObjects;
}

void laserScanner::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scanPtr){
    unique_lock<mutex> lock(laserMutex);
    laserObject lobj;
    isLaserDataAvailable=true;
//    sensor_msgs::LaserScan::ConstPtr sptr = scanPtr;

    lObjects.clear();
    int angle_count = (scanPtr->angle_max-scanPtr->angle_min) / scanPtr->angle_increment;
    int scan_index_beg= LASER_ANGLE_OFFSET / scanPtr->angle_increment + 1;
    int scan_index_end= (LASER_ANGLE_OFFSET - (2*M_PI - scanPtr->angle_max + scanPtr->angle_min)) / scanPtr->angle_increment;

    int obj_index_beg=0; int obj_index_end=0;
    double obj_angle_beg=0;
    double obj_angle_end=0; double obj_angle_min=0;
    int obj_cnt=0;
    bool isObj=false;
    double distDiff=0; double distLast=0; double distCurr=0; double distMin=scanPtr->range_max;
    radialCoordinate objPoseRadial;


    for(int i=scan_index_beg; i!=scan_index_end; ++i){
        if(i>angle_count){
            i=0;
        }
        if(scanPtr->intensities[i]<LASER_LEAST_INTENSITY){
            continue;
        }else{
            distLast=distCurr;
            distCurr=scanPtr->ranges[i];
            distDiff=distCurr - distLast;
            if(isObj){
                if(scanPtr->ranges[i] < distMin){
                    distMin=scanPtr->ranges[i];

                    obj_angle_min=scanPtr->angle_min + i * scanPtr->angle_increment + LASER_ANGLE_OFFSET;
                    if(obj_angle_min > M_PI) obj_angle_min -= 2*M_PI;
                    lobj.closestPoint.r = distMin;
                    lobj.closestPoint.theta = obj_angle_min;

                }
                if(distDiff>LASER_DETECT_THRES){
                    isObj=false;
                    obj_index_end=i;
                    lobj.getDiameter();
                    if(obj_cnt<=LASER_MAX_OBJS && distMin<laserMaxDetectRange){
                        lObjects.push_back(lobj);
                        obj_cnt++;
                    }
                    distMin=scanPtr->range_max;
                }else{
                    lobj.endPoint.r = distCurr;
                    lobj.endPoint.theta = scanPtr->angle_min + i * scanPtr->angle_increment + LASER_ANGLE_OFFSET;
                    if(lobj.endPoint.theta > M_PI) lobj.endPoint.theta -= 2*M_PI;
                }
            }else{
                if(distDiff < -LASER_DETECT_THRES && distCurr<laserMaxDetectRange){
                    isObj=true;
                    obj_index_beg=i;
                    distMin=scanPtr->ranges[i];

                    obj_angle_beg=scanPtr->angle_min + i * scanPtr->angle_increment + LASER_ANGLE_OFFSET;
                    obj_angle_min=obj_angle_beg;
                    if(obj_angle_min > M_PI) obj_angle_min -= 2*M_PI;
                    lobj.beginPoint.theta = obj_angle_min;
                    lobj.beginPoint.r = distMin;
                    lobj.endPoint.theta = obj_angle_min;
                    lobj.endPoint.r = distMin;
                    lobj.closestPoint.theta = obj_angle_min;
                    lobj.closestPoint.r = distMin;
                }
            }
        }
    }

}


void laserScanner::showObjectsPose(){
    unique_lock<mutex> lock(laserMutex);
    int obj_index=0;
    cartesianCoordinate objPose;
#ifdef SHOW_RADIAL_COORDINATE
    for(vector<laserObject>::iterator it=lObjects.begin(); it!=lObjects.end(); ++it){
        ROS_INFO("Object %d Position [%f %f]", obj_index, (it->closestPoint).r, RAD2DEG((it->closestPoint).theta));
        ROS_INFO("Object %d Begin Position [%f %f]", obj_index, (it->beginPoint).r, RAD2DEG((it->beginPoint).theta));
        ROS_INFO("Object %d End Position [%f %f]", obj_index, (it->endPoint).r, RAD2DEG((it->endPoint).theta));
        ROS_INFO("Object %d Diameter: %f", obj_index, it->objDiameter);
        /*objPose = it->getWorldPose(0);
        ROS_INFO("Object %d World Position [%f %f]", obj_index, objPose.x, objPose.y);
        objPose = it->getOdomPose(0);
        ROS_INFO("Object %d Odom Position [%f %f]", obj_index, objPose.x, objPose.y);*/
        obj_index++;
    }
    obj_index=0;
#endif
    cout<<endl;
    isLaserDataAvailable=false;
}
bool laserScanner::findLeftMostObjectRadialPose(double angleMin, double angleMax, radialCoordinate &leftPose){
    unique_lock<mutex> lock(laserMutex);
    leftPose.theta = -M_PI;
    bool isFound = false;
    for(vector<laserObject>::iterator it=lObjects.begin(); it!=lObjects.end(); ++it){
        if((it->closestPoint).theta < angleMax && (it->closestPoint).theta > angleMin && (it->closestPoint).theta > leftPose.theta){
            leftPose.r = (it->closestPoint).r;
            leftPose.theta = (it->closestPoint).theta;
            isFound=true;
        }
    }
    isLaserDataAvailable=false;
    return isFound;
}

bool laserScanner::findLeftMostObjectRadialPose(double angleMin, double angleMax, laserObject &leftPose){
    unique_lock<mutex> lock(laserMutex);
    leftPose.closestPoint.theta = -M_PI;
    bool isFound = false;
    for(vector<laserObject>::iterator it=lObjects.begin(); it!=lObjects.end(); ++it){
        if((it->closestPoint).theta < angleMax && (it->closestPoint).theta > angleMin && (it->closestPoint).theta > leftPose.closestPoint.theta){
            leftPose = *it;
            isFound=true;
        }
    }
    isLaserDataAvailable=false;
    return isFound;
}

bool laserScanner::findLeftMostObjectRadialPose_blk(double angleMin, double angleMax, radialCoordinate &leftPose){
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif

        if(isLaserDataAvailable){
            return findLeftMostObjectRadialPose(angleMin, angleMax, leftPose);
        }else{
            sample_rate.sleep();
        }
    }
}

bool laserScanner::findLeftMostObjectRadialPose_blk(double angleMin, double angleMax, laserObject &leftPose){
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif

        if(isLaserDataAvailable){
            return findLeftMostObjectRadialPose(angleMin, angleMax, leftPose);
        }else{
            sample_rate.sleep();
        }
    }
}

bool laserScanner::findRightMostObjectRadialPose(double angleMin, double angleMax, radialCoordinate &rightPose){
    unique_lock<mutex> lock(laserMutex);
    rightPose.theta = M_PI;
    bool isFound = false;
    for(vector<laserObject>::iterator it=lObjects.begin(); it!=lObjects.end(); ++it){
        if((it->closestPoint).theta < angleMax && (it->closestPoint).theta > angleMin && (it->closestPoint).theta < rightPose.theta){
            rightPose.r = (it->closestPoint).r;
            rightPose.theta = (it->closestPoint).theta;
            isFound=true;
        }
    }
    isLaserDataAvailable=false;
    return isFound;
}

bool laserScanner::findRightMostObjectRadialPose(double angleMin, double angleMax, laserObject &rightPose){
    unique_lock<mutex> lock(laserMutex);
    rightPose.closestPoint.theta = M_PI;
    bool isFound = false;
    for(vector<laserObject>::iterator it=lObjects.begin(); it!=lObjects.end(); ++it){
        if((it->closestPoint).theta < angleMax && (it->closestPoint).theta > angleMin && (it->closestPoint).theta < rightPose.closestPoint.theta){
            rightPose = *it;
            isFound=true;
        }
    }
    isLaserDataAvailable=false;
    return isFound;
}

bool laserScanner::findRightMostObjectRadialPose_blk(double angleMin, double angleMax, radialCoordinate &rightPose){
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        Laser_queue.callOne();
#endif
        if(isLaserDataAvailable){
            return findRightMostObjectRadialPose(angleMin, angleMax, rightPose);
        }else{
            sample_rate.sleep();
        }
    }
}

bool laserScanner::findRightMostObjectRadialPose_blk(double angleMin, double angleMax, laserObject &rightPose){
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        Laser_queue.callOne();
#endif
        if(isLaserDataAvailable){
            return findRightMostObjectRadialPose(angleMin, angleMax, rightPose);
        }else{
            sample_rate.sleep();
        }
    }
}

bool laserScanner::findClosestObjectRadialPose(double angleMin, double angleMax, radialCoordinate &closestDist){
    unique_lock<mutex> lock(laserMutex);
    closestDist.r=laserMaxDetectRange;
    bool isFound=false;
    for(vector<laserObject>::iterator it=lObjects.begin(); it!=lObjects.end(); ++it){
        if((it->closestPoint).r < closestDist.r && (it->closestPoint).theta < angleMax && (it->closestPoint).theta > angleMin){
            closestDist.r=(it->closestPoint).r;
            closestDist.theta=(it->closestPoint).theta;
            isFound=true;
        }
    }
    isLaserDataAvailable=false;
    return isFound;
}

bool laserScanner::findClosestObjectRadialPose(double angleMin, double angleMax, laserObject &closestDist){
    unique_lock<mutex> lock(laserMutex);
    closestDist.closestPoint.r=laserMaxDetectRange;
    bool isFound=false;
    for(vector<laserObject>::iterator it=lObjects.begin(); it!=lObjects.end(); ++it){
        if((it->closestPoint).r < closestDist.closestPoint.r && (it->closestPoint).theta < angleMax && (it->closestPoint).theta > angleMin){
            closestDist = *it;
            isFound=true;
        }
    }
    isLaserDataAvailable=false;
    return isFound;
}

bool laserScanner::findClosestObjectRadialPose_blk(double angleMin, double angleMax, radialCoordinate &closestDist){
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        Laser_queue.callOne();
#endif
        if(isLaserDataAvailable){
            return findClosestObjectRadialPose(angleMin, angleMax, closestDist);
        }else{
            sample_rate.sleep();
        }
    }
}

bool laserScanner::findClosestObjectRadialPose_blk(double angleMin, double angleMax, laserObject &closestDist){
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif

        if(isLaserDataAvailable){
            return findClosestObjectRadialPose(angleMin, angleMax, closestDist);
        }else{
            sample_rate.sleep();
        }
    }
}

