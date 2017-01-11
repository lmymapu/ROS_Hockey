#ifndef _3DCAMOBJECT_H_
#define _3DCAMOBJECT_H_

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include "ObjectInMap.h"

class CamObject3D
{
public:
    Color mColor;
    ObjectType mType;
    // position in the image
    cv::Point mMassCenter;
    pcl::PointXYZ m3DPosition;
    cv::Moments mMoment;
    // only consider 2D distance
    float mEulerDistance;
    int mNumPoints;
    int mTotalNumPoints;
    CamObject3D();
};

#endif
