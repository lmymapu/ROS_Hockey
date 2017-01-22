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
    Map(double av, double bv, Color c);
    double a,b;
    double delta_a, delta_b;        //border region tolerance
    Color teamColor;
    tf::StampedTransform trafo_Odom2World;
    vector<PostInMap> postObjs;
    vector<PuckInMap> puckObjs;
    GateInMap yellGate;
    GateInMap blGate;
    unsigned int pucksInGoal;
    void printMap();
    bool isPuckInGate(const PuckInMap pk);
};

#endif // MAP_H
