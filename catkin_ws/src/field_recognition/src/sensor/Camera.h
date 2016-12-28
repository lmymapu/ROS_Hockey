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
#include "CamObject.h"
#include "globalConfig.h"

class Trackbar;

class turtlebotCamera
{
public:
    ros::NodeHandle nh;
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
    turtlebotCamera(const turtlebotCamera&);

    void getCameraData();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    std::vector<CamObject> detectObject(Color enColor, cv::Scalar &lowRange, cv::Scalar &highRange,
                                     cv::Scalar &lowRange1, cv::Scalar &highRange1);

    void detectObject(Color enColor);
    bool objectInMiddle(Color enColor, double &offset);
    bool firstObjRight(Color enColor, double &offset);
    bool firstObjLeft(Color enColor, double &offset);
    bool lastObjRight(Color enColor, double &offset);
    bool lastObjLeft(Color enColor, double &offset);
    bool firstObjToMiddle(Color enColor, double &offset);

private:
    Trackbar *mTrackbar;
    Trackbar *mTrackbar1;
    ros::CallbackQueue camera_queue;
public:
    bool isCamDataAvailable;
    double picWidth, picHeight;
    const static double PICTURE_MIDDLE_REGION = 0.1;
};

#endif
