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
    turtlebotCamera();
    turtlebotCamera(const turtlebotCamera&);

    void getCameraData();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    std::vector<CamObject> detectObject(Color enColor, cv::Scalar &lowRange, cv::Scalar &highRange,
                                     cv::Scalar &lowRange1, cv::Scalar &highRange1);

    void detectObject(Color enColor, std::vector<CamObject>& cObjects);
    bool objectInMiddle(Color enColor);

private:
    Trackbar *mTrackbar;
    Trackbar *mTrackbar1;
    ros::CallbackQueue camera_queue;
};

#endif
