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

    void detectObject2D(Color enColor, std::vector<CamObject3D>& cObjects);
    void detectObject2D(Color enColor);
    void detectObject3D(Color enColor, std::vector<CamObject3D>& cObjects);
    void detectObject3D(Color enColor);
    bool objectInMiddle(Color enColor, double &offset);
    bool rightMostObj(Color enColor, double winBeg, double winEnd, double &offset);     //offset: the distance to the middle of an image
    bool rightMostObj(Color enColor, double winBeg, double winEnd, CamObject3D &obj);
    bool rightMostObj_blk(Color enColor, double winBeg, double winEnd, CamObject3D &obj);
    bool rightMostObj_blk(Color enColor, double winBeg, double winEnd, double &offset);
    bool leftMostObj(Color enColor, double winBeg, double winEnd, double &offset);
    bool leftMostObj(Color enColor, double winBeg, double winEnd, CamObject3D &obj);
    bool leftMostObj_blk(Color enColor, double winBeg, double winEnd, CamObject3D &obj);
    bool smallestZObj(Color enColor, double winBeg, double winEnd, CamObject3D &obj);
    bool smallestZObj_blk(Color enColor, double winBeg, double winEnd, CamObject3D &obj);
    void setMiddleRange(double mr){camMidRegionArea=mr;}
    bool isEmpty();
    bool is2DImgReady(){return is2DImgAvailable;}
    bool isImgDataReady(){return (is2DImgAvailable && is3DImgAvailable);}
    bool setImgDataDirty(){is2DImgAvailable = false; is3DImgAvailable = false;}
    // print attribute of the objet
    void printObjects();
    void eliminateInvalidData();

private:
    // ros node handle
    image_transport::ImageTransport it;
    // ros::NodeHandle nh;
    // image subscriber
    image_transport::Subscriber imSub;
    // point cloud subscriber
    ros::Subscriber pcSub;
    ros::Rate Cam_rate;
    // image data
    cv::Mat mImage;
    int mLength;
    int mWidth;
    int mNumPixels;
    // first initialize
    bool mFirstInitialization;
    // point cloud
    pcl::PointCloud<pcl::PointXYZ> mPointCloud;

    // mutex
    std::mutex mMutexImg;
    std::mutex mMutexPointCloud;

    // trackbar for configuring the RGB ranges
    Trackbar *mTrackbar;
    Trackbar *mTrackbar1;

    // middle region of 2D image
    double camMidRegionArea;

    // flags: image data available
    bool is2DImgAvailable;
    bool is3DImgAvailable;

    // threshold to get 3D position
    float mMaxDistance;
    // calculate 3D position of the objects
    void getPointPose(const std::vector<std::vector<cv::Point> >& contours,
                      std::vector<pcl::PointXYZ>& pointPose, std::vector<int>& aviNumPoints,
                      std::vector<int> &aviTotalNumPoints);
    void getMedianPointPose(const std::vector<std::vector<cv::Point> >& contours,
                            std::vector<pcl::PointXYZ>& pointPose, std::vector<cv::Point>& aTopPoint);
    void getTopPoint2D(const std::vector<std::vector<cv::Point> >& acontours,std::vector<cv::Point>& aTopPoint);

};

#endif
