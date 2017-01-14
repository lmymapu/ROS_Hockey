#ifndef _3DCAMERA_H_
#define _3DCAMERA_H_

#include <ros/ros.h>
#include <string.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <map>
#include <mutex>
#include "3DCamObject.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "globalConfig.h"

class Trackbar;

class sepDataQueue3D
{
public:
    ros::NodeHandle nh, nh3D;
#ifdef SINGLE_FIFO_TH_DATAREQ
    sepDataQueue3D(){}
#endif
#ifdef MULTI_TH_DATAREQ
    sepDataQueue3D(){}
#endif
#ifdef MULTI_FIFO_DATAREQ
    ros::CallbackQueue Img_queue,PtCloud_queue;
    sepDataQueue3D(){
        nh.setCallbackQueue(&Img_queue);
        nh3D.setCallbackQueue(&PtCloud_queue);
    }
#endif
#ifdef MULTI_FIFO_TH_DATAREQ
    ros::CallbackQueue Img_queue,PtCloud_queue;
    sepDataQueue3D(){
        nh.setCallbackQueue(&Img_queue);
        nh3D.setCallbackQueue(&PtCloud_queue);
    }
#endif
};

class turtlebotCamera3D:public sepDataQueue3D
{
public:
    std::map<Color, cv::Scalar> lowRanges;
    std::map<Color, cv::Scalar> highRanges;
    std::vector<cv::Scalar> lowRangesRed;
    std::vector<cv::Scalar> highRangesRed;
    std::vector<CamObject3D> cObjects3D;
    double imgWidth, imgHeight;

    turtlebotCamera3D();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& msg);
    std::vector<CamObject3D> detectObject(Color enColor, cv::Scalar &lowRange, cv::Scalar &highRange,
                                     cv::Scalar &lowRange1, cv::Scalar &highRange1);

    void detectObject(Color enColor, std::vector<CamObject3D>& cObjects);
    void detectObject(Color enColor);
    bool isEmpty();
    bool isImgDataReady(){return (is2DImgAvailable && is3DImgAvailable);}
    bool setImgDataDirty(){is2DImgAvailable = false; is3DImgAvailable = false;}
    // print attribute of the objet
    void printObjects();

private:
    // ros node handle
    image_transport::ImageTransport it;
    // ros::NodeHandle nh;
    // image subscriber
    image_transport::Subscriber imSub;
    // point cloud subscriber
    ros::Subscriber pcSub;

    // image data
    cv::Mat mImage;
    int mLength;
    int mWidth;
    // point cloud
    pcl::PointCloud<pcl::PointXYZ> mPointCloud;

    // mutex
    std::mutex mMutexImg;
    std::mutex mMutexPointCloud;

    // trackbar for configuring the RGB ranges
    Trackbar *mTrackbar;
    Trackbar *mTrackbar1;

    // flags: image data available
    bool is2DImgAvailable;
    bool is3DImgAvailable;

    // threshold to get 3D position
    float mMaxDistance;
    // calculate 3D position of the objects
    void getPointPose(const std::vector<std::vector<cv::Point> >& contours,
                            std::vector<pcl::PointXYZ>& pointPose, std::vector<int>& aviNumPoints, std::vector<int> &aviTotalNumPoints);
};

#endif
