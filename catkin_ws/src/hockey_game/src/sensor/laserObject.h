#ifndef LASEROBJECT_H
#define LASEROBJECT_H
#include "CommonMath.h"
#include "globalConfig.h"

class laserObject
{
public:
    laserObject(){}
    radialCoordinate closestPoint;
    radialCoordinate beginPoint;
    radialCoordinate endPoint;
    double objDiameter;
    void getDiameter();
    cartesianCoordinate getOdomPose(double objRadius);
    cartesianCoordinate getWorldPose(double objRadius);
    cartesianCoordinate getCamPose();
    bool isObjInTrack();
    bool isObjInRegion(double ybeg, double yend);
};

#endif // LASEROBJECT_H
