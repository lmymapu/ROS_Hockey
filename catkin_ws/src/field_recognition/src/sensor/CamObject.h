#ifndef _CAM_OBJECT_H_
#define _CAM_OBJECT_H_

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ObjectInMap.h"

class CamObject
{
public:
    Color mColor;
    ObjectType mType;
    cv::Point2f mPosition;
    cv::Moments mMoment;
    CamObject();
};

#endif
