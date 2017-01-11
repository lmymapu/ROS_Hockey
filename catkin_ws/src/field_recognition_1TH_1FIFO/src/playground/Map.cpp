#include "Map.h"

Map::Map():postObjs(MAP_NUMOF_POSTS),puckObjs(MAP_NUMOF_PUCKS)
{
a=0; b=0;
}

void Map::printMap(){
    fstream outputMap;
    outputMap.open("/home/gu47liy/Work/LKC++/catkin_ws/src/field_recognition/mapinfo.txt", fstream::out);
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
    outputMap.close();
}
