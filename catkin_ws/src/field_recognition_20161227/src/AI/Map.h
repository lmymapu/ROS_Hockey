#ifndef MAP_H
#define MAP_H

#include <vector>
#include "ObjectInMap.h"

using namespace std;
class Map
{
public:
    const static int NUM_OF_POSTS = 14;
    const static int NUM_OF_PUCKS = 3;
    Map();
    double a,b;
    Color teamColor;
    cartesianCoordinate gateYellowCenter;
    cartesianCoordinate gateBlueCenter;
    tf::StampedTransform trafo_Odom2World;
    vector<ObjectInMap> postObjs;
    vector<ObjectInMap> puckObjs;
};

#endif // MAP_H
