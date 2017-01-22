#include "3DCamera.h"
#include "globalConfig.h"
#include <ctime>

#define _DEBUG_MODE_

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera1");
    turtlebotCamera3D camera;
    std::vector<CamObject3D> currObjects;
    ros::Rate r(10);

    while(true)
    {
        if(!camera.isEmpty())
        {
			int start_s = clock();
            camera.detectObject3D(yell);
            float fRate = float(CLOCKS_PER_SEC)/float(clock()-start_s);
#ifdef _DEBUG_MODE_
            camera.printObjects();
            ROS_INFO("Frame rate: %f", fRate);
#endif
        }
        ros::spinOnce();
        //r.sleep();
    }
    return 0;
}
