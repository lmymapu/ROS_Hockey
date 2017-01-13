#ifndef OBJECTINMAP_H
#define OBJECTINMAP_H
#include "tf/transform_listener.h"
#include <vector>
#include "CommonMath.h"
using namespace std;
enum Color
{
    blue, yellow, green, red, black, otherColor
};
enum ObjectType
{
    puck, gate, line, border, post, otherObject
};

class ObjectInMap
{
public:
    int objID;
    Color objColor;
    ObjectType objType;
    ObjectInMap();
    cartesianCoordinate poseInOdom;
    cartesianCoordinate poseInWorld;
    void convert_Odom2World(tf::StampedTransform trafo);
    void setValue(int, Color, ObjectType, cartesianCoordinate);
};

class PostInMap : public ObjectInMap
{
public:
    PostInMap():radius(0.1){objColor = green; objType = post;}
    PostInMap(double r):radius(r){objColor = green; objType = post;}
    double radius;
};

class PuckInMap : public ObjectInMap
{
public:
    PuckInMap():upperRadius(0.1),lowerRadius(0.2){objColor = blue; objType = puck;}
    PuckInMap(double ur, double lr, Color c):upperRadius(ur),lowerRadius(lr){objColor = c; objType = puck;}
    double upperRadius;
    double lowerRadius;
};

class GateInMap : public ObjectInMap
{
public:
    GateInMap():width(1),height(0.3),cornersInWorld(4){objColor = blue; objType = gate;}
    GateInMap(double field_a, double field_b, Color c):height(field_a/4),width(field_b/3),cornersInWorld(4){objColor = c; objType = gate;}
    double width;
    double height;
    vector<cartesianCoordinate> cornersInWorld;
    void calculateCorners();
};

#endif // OBJECTINMAP_H
