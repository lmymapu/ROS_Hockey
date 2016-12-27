#ifndef OBJECTINMAP_H
#define OBJECTINMAP_H
#include "tf/transform_listener.h"
#include "CommonMath.h"
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

#endif // OBJECTINMAP_H
