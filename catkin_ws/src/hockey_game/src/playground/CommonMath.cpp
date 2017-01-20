#include "CommonMath.h"

cartesianCoordinate radial2cart(radialCoordinate radial){
    cartesianCoordinate cart;
    cart.x=radial.r*cos(radial.theta);
    cart.y=radial.r*sin(radial.theta);
    return cart;
}
double normVec(cartesianCoordinate point1, cartesianCoordinate point0){
    return sqrt(pow(point1.y-point0.y , 2) + pow(point1.x-point0.x , 2));
}

cartesianCoordinate calculateUnitVec(cartesianCoordinate point1, cartesianCoordinate point0){
    cartesianCoordinate vec;
    double len = normVec(point1, point0);
    vec.y = (point1.y - point0.y)/len;
    vec.x = (point1.x - point0.x)/len;
    return vec;
}

cartesianCoordinate getXfromY(cartesianCoordinate yAxis){
    cartesianCoordinate xAxis;
    xAxis.x = yAxis.y;
    xAxis.y = -yAxis.x;
    return xAxis;
}

cartesianCoordinate getYfromX(cartesianCoordinate xAxis){
    cartesianCoordinate yAxis;
    yAxis.x = -xAxis.y;
    yAxis.y = xAxis.x;
    return yAxis;
}

cartesianCoordinate cartesianCoordinate::operator-(){
    double res_x=-x;
    double res_y=-y;
    return cartesianCoordinate(res_x, res_y);
}

cartesianCoordinate cartesianCoordinate::operator -(const cartesianCoordinate &rhs){
    double res_x = x - rhs.x;
    double res_y = y - rhs.y;
    return cartesianCoordinate(res_x, res_y);
}

cartesianCoordinate cartesianCoordinate::operator +(const cartesianCoordinate &rhs){
    double res_x = x + rhs.x;
    double res_y = y + rhs.y;
    return cartesianCoordinate(res_x, res_y);
}

double cartesianCoordinate::operator *(const cartesianCoordinate &rhs){
    double prod=x * rhs.x + y * rhs.y;
    return prod;
}

void cartesianCoordinate::normalize(){
    double r = length();
    x = x/r;
    y = y/r;
}

