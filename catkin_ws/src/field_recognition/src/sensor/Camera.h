#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <ros/ros.h>
#include <string.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <ros/callback_queue.h>
#include <map>
#include <mutex>
#include "CamObject.h"
#include "globalConfig.h"

class Trackbar;

class sepDataQueue
{
public:
    ros::NodeHandle nh;
#ifdef SINGLE_FIFO_TH_DATAREQ
    sepDataQueue(){}
#endif
#ifdef MULTI_TH_DATAREQ
    sepDataQueue(){}
#endif
#ifdef MULTI_FIFO_DATAREQ
    ros::CallbackQueue Cam2D_queue;
    sepDataQueue(){nh.setCallbackQueue(&Cam2D_queue);}
#endif
#ifdef MULTI_FIFO_TH_DATAREQ
    ros::CallbackQueue Cam2D_queue;
    sepDataQueue(){nh.setCallbackQueue(&Cam2D_queue);}
#endif
};

class turtlebotCamera:public sepDataQueue
{
public:
//    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber imSub;
    cv::Mat mImage;
    cv_bridge::CvImagePtr cv_ptr;
    std::map<Color, cv::Scalar> lowRanges;
    std::map<Color, cv::Scalar> highRanges;
    std::vector<cv::Scalar> lowRangesRed;
    std::vector<cv::Scalar> highRangesRed;
    std::vector<CamObject> cObjects;
    turtlebotCamera();

    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    std::vector<CamObject> detectObject(Color enColor, cv::Scalar &lowRange, cv::Scalar &highRange,
                                     cv::Scalar &lowRange1, cv::Scalar &highRange1);

    void detectObject(Color enColor);
    void detectObject(Color enColor, std::vector<CamObject> &obj);
    bool objectInMiddle(Color enColor, double &offset);
    bool rightMostObj(Color enColor, double winBeg, double winEnd, double &offset);
    bool rightMostObj_blk(Color enColor, double winBeg, double winEnd, double &offset);
    bool leftMostObj(Color enColor, double winBeg, double winEnd, double &offset);
    void setMiddleRange(double mr){camMidRegionArea=mr;}

protected:
    Trackbar *mTrackbar;
    Trackbar *mTrackbar1;
    std::mutex mMutexImg;
    ros::Rate Cam_rate;
    double camMidRegionArea;
public:
    bool isCamDataAvailable;
    double picWidth, picHeight;
};



#endif
