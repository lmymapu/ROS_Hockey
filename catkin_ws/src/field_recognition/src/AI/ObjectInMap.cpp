#include "ObjectInMap.h"

ObjectInMap::ObjectInMap():objColor(otherColor),objType(otherObject),objID(0){
}

void ObjectInMap::convert_Odom2World(tf::StampedTransform trafo){
    tf::Vector3 objPoseInWorld, objPoseInOdom;
    objPoseInOdom.setValue(poseInOdom.x, poseInOdom.y, 0);
    objPoseInWorld = trafo * objPoseInOdom;
}

void ObjectInMap::setValue(int ID, Color clr, ObjectType typ, cartesianCoordinate odomPose){
    objID = ID; objColor = clr; objType = typ;
    poseInOdom.x = odomPose.x;
    poseInOdom.y = odomPose.y;
}
