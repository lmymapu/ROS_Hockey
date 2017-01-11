#include "laserobject.h"
#include <boost/bind.hpp>
#include <iostream>
#define RAD2DEG(x) ((x)*180./M_PI)

//#define SHOW_CARTESIAN_COORDINATE
#define SHOW_RADIAL_COORDINATE
//#define SHOW_ODOM_COORDINATE
//#define AVERAGE_ANGLE_ALGORITHM
using namespace std;
laserObject::laserObject():laserMaxDetectRange(LASER_MAX_DETECT_RANGE),sample_rate(10)
{
    isLaserDataAvailable=false;
#ifndef SIMULATION_MODE
    laserSub = node.subscribe("/scan_laser", 10, &laserObject::scanCallback, this);
#else
    laserSub = node.subscribe("/base_scan_1", 10, &laserObject::scanCallback, this);
#endif
}

laserObject::laserObject(const laserObject &lobj):laserMaxDetectRange(LASER_MAX_DETECT_RANGE),sample_rate(10){
    //this->scanPtr = lobj.scanPtr;
    objectsInRadialCoor = lobj.objectsInRadialCoor;
    objectsInOdomCartCoor = lobj.objectsInOdomCartCoor;
    objectsInRobotCartCoor = lobj.objectsInRobotCartCoor;
}
laserObject::laserObject(sensor_msgs::LaserScan::ConstPtr scan):laserMaxDetectRange(LASER_MAX_DETECT_RANGE),sample_rate(10){
#ifndef SIMULATION_MODE
    laserSub = node.subscribe("/scan_laser", 10, &laserObject::scanCallback, this);
#else
    laserSub = node.subscribe("/base_scan_1", 10, &laserObject::scanCallback, this);
#endif
}

void laserObject::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scanPtr){
    unique_lock<mutex> lock(laserMutex);
    isLaserDataAvailable=true;
#ifndef SIMULATION_MODE

    objectsInRadialCoor.clear();
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
#ifndef AVERAGE_ANGLE_ALGORITHM
                    obj_angle_min=scanPtr->angle_min + i * scanPtr->angle_increment + LASER_ANGLE_OFFSET;
                    if(obj_angle_min > M_PI) obj_angle_min -= 2*M_PI;
#endif
                }
                if(distDiff>LASER_DETECT_THRES){
                    isObj=false;
                    obj_index_end=i;
#ifdef AVERAGE_ANGLE_ALGORITHM
                    obj_angle_end=scanPtr->angle_min + obj_index_end * scanPtr->angle_increment + LASER_ANGLE_OFFSET;
                    if(obj_angle_end > M_PI) obj_angle_end -= 2*M_PI;
                    objPoseRadial.theta= (obj_angle_end+obj_angle_beg)/2;
#else
                    objPoseRadial.theta = obj_angle_min;
#endif
                    objPoseRadial.r=distMin;
                    //
                    if(obj_cnt<=LASER_MAX_OBJS && distMin<laserMaxDetectRange){
                        objectsInRadialCoor.push_back(objPoseRadial);
                        obj_cnt++;
                    }
                    distMin=scanPtr->range_max;
                }
            }else{
                if(distDiff < -LASER_DETECT_THRES && distCurr<laserMaxDetectRange){
                    isObj=true;
                    obj_index_beg=i;
                    distMin=scanPtr->ranges[i];
#ifndef AVERAGE_ANGLE_ALGORITHM
                    obj_angle_min=scanPtr->angle_min + i * scanPtr->angle_increment + LASER_ANGLE_OFFSET;
                    if(obj_angle_min > M_PI) obj_angle_min -= 2*M_PI;
#else
                    obj_angle_beg=scanPtr->angle_min + obj_index_beg * scanPtr->angle_increment + LASER_ANGLE_OFFSET;
                    if(obj_angle_beg > M_PI) obj_angle_beg -= 2*M_PI;
#endif
                }
            }
        }
    }


#else
    objectsInRadialCoor.clear();
    int angle_count = (scanPtr->angle_max-scanPtr->angle_min) / scanPtr->angle_increment;

    int scan_index_beg= 0;
    int scan_index_end= (scanPtr->angle_max - scanPtr->angle_min) / scanPtr->angle_increment;

    int obj_index_beg=0; int obj_index_end=0;
    double obj_angle_beg=0;
    double obj_angle_end=0; double obj_angle_min=0;
    int obj_cnt=0;
    bool isObj=false;
    double distDiff=0; double distLast=0; double distCurr=0; double distMin=scanPtr->range_max;
    radialCoordinate objPoseRadial;


    for(int i=scan_index_beg; i!=scan_index_end; ++i){

        distLast=distCurr;
        distCurr=scanPtr->ranges[i];
        distDiff=distCurr - distLast;
        if(isObj){
            if(scanPtr->ranges[i] < distMin){
                distMin=scanPtr->ranges[i];
                obj_angle_min=scanPtr->angle_min + i * scanPtr->angle_increment;

            }
            if(distDiff>LASER_DETECT_THRES){
                isObj=false;
                obj_index_end=i;

                objPoseRadial.theta = obj_angle_min;
                objPoseRadial.r=distMin;
                //
                if(obj_cnt<=LASER_MAX_OBJS && distMin<laserMaxDetectRange){
                    objectsInRadialCoor.push_back(objPoseRadial);
                    obj_cnt++;
                }
                distMin=scanPtr->range_max;
            }
        }else{
            if(distDiff < -LASER_DETECT_THRES && distCurr<laserMaxDetectRange){
                isObj=true;
                obj_index_beg=i;
                distMin=scanPtr->ranges[i];
                obj_angle_min=scanPtr->angle_min + i * scanPtr->angle_increment;

            }
        }

    }

#endif
}


void laserObject::showObjectsPose(){
    unique_lock<mutex> lock(laserMutex);
    int obj_index=0;
#ifdef SHOW_RADIAL_COORDINATE
    for(vector<radialCoordinate>::iterator it=objectsInRadialCoor.begin(); it!=objectsInRadialCoor.end(); ++it){
        ROS_INFO("Object %d Position [%f %f]", obj_index, it->r, RAD2DEG(it->theta));
        obj_index++;
    }
    obj_index=0;
#endif
#ifdef SHOW_CARTESIAN_COORDINATE
    for(vector<cartesianCoordinate>::iterator it=objectsInRobotCartCoor.begin(); it!=objectsInRobotCartCoor.end(); ++it){
        ROS_INFO("Object %d Position in robot coordinates [%f %f]", obj_index, it->x, it->y);
        obj_index++;
    }
    obj_index=0;
#endif
#ifdef SHOW_ODOM_COORDINATE
    for(vector<cartesianCoordinate>::iterator it=objectsInOdomCartCoor.begin(); it!=objectsInOdomCartCoor.end(); ++it){
        ROS_INFO("Object %d Position in Odom coordinates [%f %f]", obj_index, it->x, it->y);
        obj_index++;
    }
    obj_index=0;
#endif
    cout<<endl;
}
bool laserObject::findLeftMostObjectRadialPose(double angleMin, double angleMax, radialCoordinate &leftPose){
    unique_lock<mutex> lock(laserMutex);
    leftPose.theta = -M_PI;
    bool isFound = false;
    for(vector<radialCoordinate>::iterator it=objectsInRadialCoor.begin(); it!=objectsInRadialCoor.end(); ++it){
        if(it->theta < angleMax && it->theta > angleMin && it->theta > leftPose.theta){
            leftPose.r = it->r;
            leftPose.theta = it->theta;
            isFound=true;
        }
    }
    return isFound;
}

bool laserObject::findLeftMostObjectRadialPose_blk(double angleMin, double angleMax, radialCoordinate &leftPose){
    while(ros::ok()){
        ros::spinOnce();
        if(isLaserDataAvailable){
            isLaserDataAvailable=false;
            return findLeftMostObjectRadialPose(angleMin, angleMax, leftPose);
        }else{
            sample_rate.sleep();
        }
    }
}

bool laserObject::findRightMostObjectRadialPose(double angleMin, double angleMax, radialCoordinate &rightPose){
    unique_lock<mutex> lock(laserMutex);
    rightPose.theta = M_PI;
    bool isFound = false;
    for(vector<radialCoordinate>::iterator it=objectsInRadialCoor.begin(); it!=objectsInRadialCoor.end(); ++it){
        if(it->theta < angleMax && it->theta > angleMin && it->theta < rightPose.theta){
            rightPose.r = it->r;
            rightPose.theta = it->theta;
            isFound=true;
        }
    }
    return isFound;
}

bool laserObject::findRightMostObjectRadialPose_blk(double angleMin, double angleMax, radialCoordinate &rightPose){
    while(ros::ok()){
        ros::spinOnce();
        if(isLaserDataAvailable){
            isLaserDataAvailable=false;
            return findRightMostObjectRadialPose(angleMin, angleMax, rightPose);
        }else{
            sample_rate.sleep();
        }
    }
}

bool laserObject::findClosestObjectRadialPose(double angleMin, double angleMax, radialCoordinate &closestDist){
    unique_lock<mutex> lock(laserMutex);
    closestDist.r=laserMaxDetectRange;
    bool isFound=false;
    for(vector<radialCoordinate>::iterator it=objectsInRadialCoor.begin(); it!=objectsInRadialCoor.end(); ++it){
        if(it->r < closestDist.r && it->theta < angleMax && it->theta > angleMin){
            closestDist.r=it->r;
            closestDist.theta=it->theta;
            isFound=true;
        }
    }
    return isFound;
}

bool laserObject::findClosestObjectRadialPose_blk(double angleMin, double angleMax, radialCoordinate &closestDist){
    while(ros::ok()){
        ros::spinOnce();
        if(isLaserDataAvailable){
            isLaserDataAvailable=false;
            return findClosestObjectRadialPose(angleMin, angleMax, closestDist);
        }else{
            sample_rate.sleep();
        }
    }
}

void laserObject::transformOdomCartCoor(tf::StampedTransform trafo_Odom2Base){
    tf::Vector3 objPoseInRobot, objPoseInOdom;
    cartesianCoordinate obj2DPoseInOdom;
    for(vector<cartesianCoordinate>::iterator it=objectsInRobotCartCoor.begin(); it!=objectsInRobotCartCoor.end(); ++it){
        objPoseInRobot.setValue(it->x, it->y, 0);
        objPoseInOdom = trafo_Odom2Base * objPoseInRobot;
        obj2DPoseInOdom.x = objPoseInOdom.getX();
        obj2DPoseInOdom.y = objPoseInOdom.getY();
        objectsInOdomCartCoor.push_back(obj2DPoseInOdom);
    }
}
