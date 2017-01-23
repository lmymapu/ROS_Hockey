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
    cartesianCoordinate closestPointWorld;
    cartesianCoordinate beginPointWorld;
    cartesianCoordinate endPointWorld;
    double objDiameter;
    void getDiameter();
    cartesianCoordinate getOdomPose(double objRadius);
    cartesianCoordinate getWorldPose(double objRadius);
    cartesianCoordinate getCamPose();
    void getAllWorldPose();
    bool isObjInTrack();
    bool isObjInRegion(double ybeg, double yend);
};

#endif // LASEROBJECT_H
