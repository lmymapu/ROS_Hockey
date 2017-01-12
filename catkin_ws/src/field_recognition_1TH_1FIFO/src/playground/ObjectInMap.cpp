#include "ObjectInMap.h"

ObjectInMap::ObjectInMap():objColor(otherColor),objType(otherObject),objID(0){
}

void ObjectInMap::convert_Odom2World(tf::StampedTransform trafo){
    tf::Vector3 objPoseInWorld, objPoseInOdom;
    objPoseInOdom.setValue(poseInOdom.x, poseInOdom.y, 0);
    objPoseInWorld = trafo * objPoseInOdom;
    poseInWorld.setVal(objPoseInWorld.getX(),objPoseInWorld.getY());
}

void ObjectInMap::setValue(int ID, Color clr, ObjectType typ, cartesianCoordinate odomPose){
    objID = ID; objColor = clr; objType = typ;
    poseInOdom.x = odomPose.x;
    poseInOdom.y = odomPose.y;
}

void GateInMap::calculateCorners(){
    cornersInWorld[0].setVal(poseInWorld.x - width/2 , poseInWorld.y - height/2);
    cornersInWorld[1].setVal(poseInWorld.x + width/2 , poseInWorld.y - height/2);
    cornersInWorld[2].setVal(poseInWorld.x + width/2 , poseInWorld.y + height/2);
    cornersInWorld[3].setVal(poseInWorld.x - width/2 , poseInWorld.y + height/2);
}
