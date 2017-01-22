#include "Map.h"

Map::Map():postObjs(MAP_NUMOF_POSTS),puckObjs(MAP_NUMOF_PUCKS)
{
a=0; b=0;teamColor=bl;
delta_a=MAP_BORDER_Y; delta_b=MAP_BORDER_X;
pucksInGoal =0;
}

Map::Map(double av, double bv, Color c):a(av), b(bv), teamColor(c), postObjs(MAP_NUMOF_POSTS),puckObjs(MAP_NUMOF_PUCKS){
    pucksInGoal = 0;delta_a=MAP_BORDER_Y; delta_b=MAP_BORDER_X;
}

void Map::printMap(){
    fstream outputMap;
    outputMap.open("/home/gruppe5/catkin_ws/src/hockey_game/mapinfo.txt", fstream::out);
    outputMap << "Team color is " << teamColor <<endl;
    outputMap << "playground size: a = "<<a<<", b = "<<b<<endl;
    outputMap << "post 0: [ " << postObjs[0].poseInWorld.x <<"\t"<< postObjs[0].poseInWorld.y <<" ]" <<endl;
    outputMap << "post 1: [ " << postObjs[1].poseInWorld.x <<"\t"<< postObjs[1].poseInWorld.y <<" ]" <<endl;
    outputMap << "post 4: [ " << postObjs[4].poseInWorld.x <<"\t"<< postObjs[4].poseInWorld.y <<" ]" <<endl;
    outputMap << "post 5: [ " << postObjs[5].poseInWorld.x <<"\t"<< postObjs[5].poseInWorld.y <<" ]" <<endl;
    outputMap << "post 6: [ " << postObjs[6].poseInWorld.x <<"\t"<< postObjs[6].poseInWorld.y <<" ]" <<endl;
    outputMap << "post 7: [ " << postObjs[7].poseInWorld.x <<"\t"<< postObjs[7].poseInWorld.y <<" ]" <<endl;
    outputMap << "post 8: [ " << postObjs[8].poseInWorld.x <<"\t"<< postObjs[8].poseInWorld.y <<" ]" <<endl;
    outputMap << "post 9: [ " << postObjs[9].poseInWorld.x <<"\t"<< postObjs[9].poseInWorld.y <<" ]" <<endl;
    outputMap << "post 12: [ " << postObjs[12].poseInWorld.x <<"\t"<< postObjs[12].poseInWorld.y <<" ]" <<endl;
    outputMap << "post 13: [ " << postObjs[13].poseInWorld.x <<"\t"<< postObjs[13].poseInWorld.y <<" ]" <<endl;
    outputMap << "Yellow gate center:" << yellGate.poseInWorld.x <<"\t"<< yellGate.poseInWorld.y<<endl;
    outputMap << "Bellow gate center:" << blGate.poseInWorld.x <<"\t"<< blGate.poseInWorld.y<<endl;
    outputMap.close();
}

bool Map::isPuckInGate(const PuckInMap pk){
    if(teamColor = yell){
        return (pk.poseInWorld.x < yellGate.poseInWorld.x+b/6 && pk.poseInWorld.x > yellGate.poseInWorld.x-b/6 &&
                pk.poseInWorld.y < yellGate.poseInWorld.y+a/8 && pk.poseInWorld.y > yellGate.poseInWorld.y-a/8);
    }
    if(teamColor = bl){
        return (pk.poseInWorld.x < blGate.poseInWorld.x+b/6 && pk.poseInWorld.x > blGate.poseInWorld.x-b/6 &&
                pk.poseInWorld.y < blGate.poseInWorld.y+a/8 && pk.poseInWorld.y > blGate.poseInWorld.y-a/8);
    }
}
