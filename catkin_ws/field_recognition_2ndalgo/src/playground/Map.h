#ifndef MAP_H
#define MAP_H

#include <vector>
#include "ObjectInMap.h"
#include "globalConfig.h"
#include <fstream>

using namespace std;
class Map
{
public:
    Map();
    double a,b;
    Color teamColor;
    tf::StampedTransform trafo_Odom2World;
    vector<PostInMap> postObjs;
    vector<PuckInMap> puckObjs;
    GateInMap yellowGate;
    GateInMap blueGate;
    void printMap();
};

#endif // MAP_H
