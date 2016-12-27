#ifndef COMMONMATH_H
#define COMMONMATH_H
#include "ros/ros.h"

struct radialCoordinate{
    double r;
    double theta;
    radialCoordinate(){r=0;theta=0;}
};

struct cartesianCoordinate{
    double x;
    double y;
    cartesianCoordinate(){x=0;y=0;}
    cartesianCoordinate(double xVal, double yVal):x(xVal),y(yVal){}
    cartesianCoordinate operator-();
    cartesianCoordinate operator-(const cartesianCoordinate& rhs);
    cartesianCoordinate operator+(const cartesianCoordinate& rhs);
    double operator*(const cartesianCoordinate& rhs);
};

cartesianCoordinate radial2cart(radialCoordinate);
cartesianCoordinate calculateUnitVec(cartesianCoordinate point1, cartesianCoordinate point0);
double normVec(cartesianCoordinate point1, cartesianCoordinate point0);
cartesianCoordinate getXfromY(cartesianCoordinate yAxis);
cartesianCoordinate getYfromX(cartesianCoordinate xAxis);

#endif // COMMONMATH_H
