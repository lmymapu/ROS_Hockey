#ifndef _3DCAMOBJECT_H_
#define _3DCAMOBJECT_H_

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include "ObjectInMap.h"
#include <tf/transform_listener.h>

class CamObject3D
{
public:
    Color mColor;
    ObjectType mType;
    // position in the image
    cv::Point mMassCenter;
    cv::Point mTopPoint;
    pcl::PointXYZ m3DPosition;
    pcl::PointXYZ m3DPositionMedian;
    cv::Moments mMoment;
    // only consider 2D distance
    float mEulerDistance;
    float mEulerDistanceMedian;
    float mLaserDistance;
    int mNumPoints;
    int mTotalNumPoints;
    float mAreaInImage;
    bool mOnFloor;

    CamObject3D();
    bool operator < (const CamObject3D& aObject) const
    {
        return (mEulerDistance<aObject.mEulerDistance);
    }
    cartesianCoordinate getOdomPose(double objRadius);
    cartesianCoordinate getWorldPose(double objRadius);
};

#endif
