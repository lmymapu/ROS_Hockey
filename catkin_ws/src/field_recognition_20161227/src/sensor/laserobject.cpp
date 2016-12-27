#include "laserobject.h"
#include <boost/bind.hpp>
#include <iostream>
#define RAD2DEG(x) ((x)*180./M_PI)

//#define SHOW_CARTESIAN_COORDINATE
#define SHOW_RADIAL_COORDINATE
//#define SHOW_ODOM_COORDINATE
//#define AVERAGE_ANGLE_ALGORITHM
using namespace std;
laserObject::laserObject()
{
    objectsInRadialCoor = vector<radialCoordinate>(0);
    objectsInRobotCartCoor = vector<cartesianCoordinate>(0);
    objectsInOdomCartCoor = vector<cartesianCoordinate>(0);
}

laserObject::laserObject(const laserObject &lobj):scanPtr(lobj.scanPtr){
    //this->scanPtr = lobj.scanPtr;
    objectsInRadialCoor = lobj.objectsInRadialCoor;
    objectsInOdomCartCoor = lobj.objectsInOdomCartCoor;
    objectsInRobotCartCoor = lobj.objectsInRobotCartCoor;
}
laserObject::laserObject(sensor_msgs::LaserScan::ConstPtr scan):scanPtr(scan){
    objectsInRadialCoor = vector<radialCoordinate>(0);
    objectsInRobotCartCoor = vector<cartesianCoordinate>(0);
    objectsInOdomCartCoor = vector<cartesianCoordinate>(0);
}

#ifndef SIMULATION_MODE
void laserObject::recogniseObjects(){
    objectsInRadialCoor.clear();
    objectsInRobotCartCoor.clear();
    objectsInOdomCartCoor.clear();
    int angle_count = (scanPtr->angle_max-scanPtr->angle_min) / scanPtr->angle_increment;
    int scan_index_beg= ANGLE_OFFSET / scanPtr->angle_increment + 1;
    int scan_index_end= (ANGLE_OFFSET - (2*M_PI - scanPtr->angle_max + scanPtr->angle_min)) / scanPtr->angle_increment;

    int obj_index_beg=0; int obj_index_end=0;
    double obj_angle_beg=0;
    double obj_angle_end=0; double obj_angle_min=0;
    int obj_cnt=0;
    bool isObj=false;
    double distDiff=0; double distLast=0; double distCurr=0; double distMin=scanPtr->range_max;
    radialCoordinate objPoseRadial;
    cartesianCoordinate objPoseRobotCart;


    for(int i=scan_index_beg; i!=scan_index_end; ++i){
        if(i>angle_count){
            i=0;
        }
        if(scanPtr->intensities[i]<LEAST_INTENSITY){
            continue;
        }else{
            distLast=distCurr;
            distCurr=scanPtr->ranges[i];
            distDiff=distCurr - distLast;
            if(isObj){
                if(scanPtr->ranges[i] < distMin){
                    distMin=scanPtr->ranges[i];
#ifndef AVERAGE_ANGLE_ALGORITHM
                    obj_angle_min=scanPtr->angle_min + i * scanPtr->angle_increment + ANGLE_OFFSET;
                    if(obj_angle_min > M_PI) obj_angle_min -= 2*M_PI;
#endif
                }
                if(distDiff>DETECT_THRESHOLD_DISTDIFF){
                    isObj=false;
                    obj_index_end=i;
#ifdef AVERAGE_ANGLE_ALGORITHM
                    obj_angle_end=scanPtr->angle_min + obj_index_end * scanPtr->angle_increment + ANGLE_OFFSET;
                    if(obj_angle_end > M_PI) obj_angle_end -= 2*M_PI;
                    objPoseRadial.theta= (obj_angle_end+obj_angle_beg)/2;
#else
                    objPoseRadial.theta = obj_angle_min;
#endif
                    objPoseRadial.r=distMin;
                    objPoseRobotCart = radial2cart(objPoseRadial);
                    //
                    if(obj_cnt<=MAX_NUMOF_OBJECTS && distMin<MAX_DETECT_RANGE){
                        objectsInRadialCoor.push_back(objPoseRadial);
                        objectsInRobotCartCoor.push_back(objPoseRobotCart);
                        obj_cnt++;
                    }
                    distMin=scanPtr->range_max;
                }
            }else{
                if(distDiff < -DETECT_THRESHOLD_DISTDIFF && distCurr<MAX_DETECT_RANGE){
                    isObj=true;
                    obj_index_beg=i;
                    distMin=scanPtr->ranges[i];
#ifndef AVERAGE_ANGLE_ALGORITHM
                    obj_angle_min=scanPtr->angle_min + i * scanPtr->angle_increment + ANGLE_OFFSET;
                    if(obj_angle_min > M_PI) obj_angle_min -= 2*M_PI;
#else
                    obj_angle_beg=scanPtr->angle_min + obj_index_beg * scanPtr->angle_increment + ANGLE_OFFSET;
                    if(obj_angle_beg > M_PI) obj_angle_beg -= 2*M_PI;
#endif
                }
            }
        }
    }

}
#else
void laserObject::recogniseObjects(){
    objectsInRadialCoor.clear();
    objectsInRobotCartCoor.clear();
    objectsInOdomCartCoor.clear();
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
    cartesianCoordinate objPoseRobotCart;


    for(int i=scan_index_beg; i!=scan_index_end; ++i){

        distLast=distCurr;
        distCurr=scanPtr->ranges[i];
        distDiff=distCurr - distLast;
        if(isObj){
            if(scanPtr->ranges[i] < distMin){
                distMin=scanPtr->ranges[i];
                obj_angle_min=scanPtr->angle_min + i * scanPtr->angle_increment;

            }
            if(distDiff>DETECT_THRESHOLD_DISTDIFF){
                isObj=false;
                obj_index_end=i;

                objPoseRadial.theta = obj_angle_min;
                objPoseRadial.r=distMin;
                objPoseRobotCart = radial2cart(objPoseRadial);
                //
                if(obj_cnt<=MAX_NUMOF_OBJECTS && distMin<MAX_DETECT_RANGE){
                    objectsInRadialCoor.push_back(objPoseRadial);
                    objectsInRobotCartCoor.push_back(objPoseRobotCart);
                    obj_cnt++;
                }
                distMin=scanPtr->range_max;
            }
        }else{
            if(distDiff < -DETECT_THRESHOLD_DISTDIFF && distCurr<MAX_DETECT_RANGE){
                isObj=true;
                obj_index_beg=i;
                distMin=scanPtr->ranges[i];
                obj_angle_min=scanPtr->angle_min + i * scanPtr->angle_increment;

            }
        }

    }
}

#endif

void laserObject::showLaserScanMap(){
    int angle_count = (scanPtr->angle_max-scanPtr->angle_min) / scanPtr->angle_increment;
    for(int i=0; i<=angle_count; ++i){
        float degree = RAD2DEG(scanPtr->angle_min + scanPtr->angle_increment * i) + RAD2DEG(ANGLE_OFFSET);
        if(degree > 180) degree -= 360;
        ROS_INFO(": [%f, %f]", degree, scanPtr->ranges[i]);
    }
}

void laserObject::showObjectsPose(){
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

radialCoordinate laserObject::findClosestObjectRadialPose(double angleMin, double angleMax){
    radialCoordinate closestDist;
    closestDist.r=scanPtr->range_max;
    for(vector<radialCoordinate>::iterator it=objectsInRadialCoor.begin(); it!=objectsInRadialCoor.end(); ++it){
        if(it->r < closestDist.r && it->theta < angleMax && it->theta > angleMin){
            closestDist.r=it->r;
            closestDist.theta=it->theta;
        }
    }
    return closestDist;
}

cartesianCoordinate laserObject::findClosestObjectCartPose(double angleMin, double angleMax){
    radialCoordinate closestDist = findClosestObjectRadialPose(angleMin, angleMax);
    return radial2cart(closestDist);
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
