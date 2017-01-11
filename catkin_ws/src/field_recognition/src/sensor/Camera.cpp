#include "Camera.h"
#include "trackbar.h"

#define CAMERA_IN_SIMULATOR

using namespace cv;
using namespace std;
turtlebotCamera::turtlebotCamera():it(nh),camMidRegionArea(CAM_MID_REGION),Cam_rate(10)
{
    //nh.setCallbackQueue(&camera_queue);
#ifndef SIMULATION_MODE
    imSub = it.subscribe("/camera/rgb/image_raw", CAM_FIFO_LENGTH, &turtlebotCamera::imageCb, this);
#else
    imSub = it.subscribe("/image", CAM_FIFO_LENGTH, &turtlebotCamera::imageCb, this);
#endif
    isCamDataAvailable = false;
}


void turtlebotCamera::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    unique_lock<mutex> lock(mMutexImg);
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    isCamDataAvailable = true;
    mImage = cv_ptr->image.clone();
    picWidth = double(mImage.cols);
    picHeight = double(mImage.rows);
}

std::vector<CamObject> turtlebotCamera::detectObject(Color enColor, cv::Scalar &lowRange,cv::Scalar &highRange,
                                                  cv::Scalar &lowRange1, cv::Scalar &highRange1)
{
    cv::Mat hsvImg;
    cv::Mat threshold;
    cv::Mat origImg;
    origImg = mImage.clone();
    cv::cvtColor(origImg, hsvImg, cv::COLOR_BGR2HSV);

    //cv::Scalar lowRange;
    //cv::Scalar highRange;
    // find object using given ranges
    if(enColor==red)
    {
        cv::Mat threshold1;
        cv::inRange(hsvImg, lowRange, highRange, threshold);
        cv::inRange(hsvImg, lowRange1, highRange1, threshold1);
        threshold = threshold + threshold1;
    }
    else
        cv::inRange(hsvImg, lowRange, highRange, threshold);
    cv::Mat erodeElement = cv::getStructuringElement(MORPH_RECT, Size(3,3));
    cv::Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8,8));

    // suppress noise
    erode(threshold,threshold, erodeElement);
    erode(threshold,threshold, erodeElement);

    dilate(threshold, threshold, dilateElement);
    dilate(threshold, threshold, dilateElement);

    // find object contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(threshold, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    // calculate the moments
    vector<Moments> mu(contours.size());
    for(size_t i=0, iend=contours.size(); i<iend; i++)
    {
        mu[i]=moments(contours[i]);
    }
    // get mass centers
    vector<Point2f> mc(contours.size());
    for(size_t i=0, iend=contours.size(); i<iend; i++)
    {
        mc[i]=Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
    }
    // create detected object
    vector<CamObject> objects(contours.size());
    for(size_t i=0, iend=contours.size(); i<iend; i++)
    {
        objects[i].mColor = enColor;
        objects[i].mPosition = mc[i];
        objects[i].mType = puck;
    }

    // draw the objects
    RNG rng(12345);
    for(size_t i=0, iend=objects.size(); i<iend; i++)
    {
        Scalar drawColor = Scalar(rng.uniform(0,255),rng.uniform(0,255), rng.uniform(0,255) );
        drawContours(origImg, contours, i, drawColor, 2, 8, hierarchy, 0, Point());
        circle(origImg, objects[i].mPosition, 10, drawColor);
    }

    //imwrite("Image.jpg", mImage);
    imshow("Image from camera", origImg);
    waitKey(10);
    return objects;

}


void turtlebotCamera::detectObject(Color enColor, vector<CamObject> &obj)
{
    unique_lock<mutex> lock(mMutexImg);
    cv::Mat hsvImg;
    cv::Mat threshold;
    cv::Mat origImg;
    cv::Mat threshold1;
    origImg = mImage.clone();
    cv::cvtColor(origImg, hsvImg, cv::COLOR_BGR2HSV);

    //cv::Scalar lowRange;
    //cv::Scalar highRange;
    // find object using given ranges
#ifndef SIMULATION_MODE
    switch(enColor)
    {
    case blue:
        cv::inRange(hsvImg, Scalar(113,121,34), Scalar(130,255,130), threshold);
        break;
    case yellow:
        cv::inRange(hsvImg, Scalar(20, 32, 113), Scalar(33,255,255), threshold);
        break;
    case green:
        cv::inRange(hsvImg, Scalar(45, 100,53), Scalar(82, 255,255), threshold);
        break;
    case red:
        cv::inRange(hsvImg, Scalar(0, 90, 45), Scalar(10, 255, 255), threshold);
        cv::inRange(hsvImg, Scalar(170, 255, 255), Scalar(180, 255, 255), threshold1);
        threshold = threshold + threshold1;
        break;
    case black:
        cv::inRange(hsvImg, Scalar(0,50,0), Scalar(30, 147, 120), threshold);
        break;
    default:
        break;
    }
#else
    switch(enColor)
    {
    case red:
        cv::inRange(origImg, Scalar(0,0,30), Scalar(0,0,255), threshold);
        break;
    case yellow:
        cv::inRange(origImg, Scalar(0, 40,40), Scalar(10, 255,255), threshold);
        break;
    case green:
        cv::inRange(origImg, Scalar(0, 40,0), Scalar(10, 255,10), threshold);
        break;
    case blue:
        cv::inRange(origImg, Scalar(40, 0, 0), Scalar(255, 10, 10), threshold);
        break;
    case black:
        cv::inRange(origImg, Scalar(0,0,0), Scalar(10, 10, 10), threshold);
        break;
    default:
        break;
    }
#endif

    cv::Mat erodeElement = cv::getStructuringElement(MORPH_RECT, Size(3,3));
    cv::Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8,8));
#ifndef SIMULATION_MODE
    // suppress noise
    erode(threshold,threshold, erodeElement);
    erode(threshold,threshold, erodeElement);

    dilate(threshold, threshold, dilateElement);
    dilate(threshold, threshold, dilateElement);
#endif
    // find object contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(threshold, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    // calculate the moments
    vector<Moments> mu(contours.size());
    for(size_t i=0, iend=contours.size(); i<iend; i++)
    {
        mu[i]=moments(contours[i]);
    }
    // get mass centers
    vector<Point2f> mc(contours.size());
    for(size_t i=0, iend=contours.size(); i<iend; i++)
    {
        mc[i]=Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
    }
    // create detected object
    obj.resize(contours.size());
    for(size_t i=0, iend=contours.size(); i<iend; i++)
    {
        obj[i].mColor = enColor;
        obj[i].mPosition = mc[i];
        obj[i].mMoment = mu[i];
    }

    // draw the objects
    RNG rng(12345);
    for(size_t i=0, iend=obj.size(); i<iend; i++)
    {
        Scalar drawColor = Scalar(rng.uniform(0,255),rng.uniform(0,255), rng.uniform(0,255));
        drawContours(origImg, contours, i, drawColor, 2, 8, hierarchy, 0, Point());
        circle(origImg, obj[i].mPosition, 10, drawColor);
    }

    //imwrite("Image.jpg", mImage);
    imshow("Image from camera", origImg);
    waitKey(10);
}

void turtlebotCamera::detectObject(Color enColor){
    detectObject(enColor, cObjects);
}

bool turtlebotCamera::objectInMiddle(Color enColor, double &offset){
    bool isFound = false;
    int midIndexBegin = floor(picWidth * (1 - camMidRegionArea) / 2);
    int midIndexEnd = floor(picWidth * (1 + camMidRegionArea) / 2);
    for(vector<CamObject>::iterator it=cObjects.begin(); it!=cObjects.end(); ++it){
        if(it->mPosition.x > midIndexBegin && it->mPosition.x < midIndexEnd && it->mColor == enColor){
            offset = it->mPosition.x - (picWidth - 1)/2;
            isFound = true;
            return isFound;
        }
    }
    return isFound;
}

bool turtlebotCamera::rightMostObj(Color enColor, double winBeg, double winEnd, double &offset){
    bool isFound = false;
    float posMid = (picWidth-1)/2;
    float posBeg = (picWidth-1) * winBeg;
    float posEnd = (picWidth-1) * winEnd;
    float rightMostObjPos = posBeg;
    for(vector<CamObject>::iterator it=cObjects.begin(); it!=cObjects.end(); ++it){
        if(it->mColor == enColor && it->mPosition.x < posEnd && it->mPosition.x > posBeg && it->mPosition.x > rightMostObjPos){
            offset = it->mPosition.x - posMid;
            rightMostObjPos = it->mPosition.x;
            isFound = true;
        }
    }
    return isFound;
}

bool turtlebotCamera::rightMostObj_blk(Color enColor, double winBeg, double winEnd, double &offset){
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
#ifdef MULTI_FIFO_DATAREQ
        Cam2D_queue.callOne();
#endif
        if(isCamDataAvailable){
            isCamDataAvailable=false;
            detectObject(enColor);
            return rightMostObj(enColor, winBeg, winEnd, offset);
        }else{
            Cam_rate.sleep();
        }
    }
}

bool turtlebotCamera::leftMostObj(Color enColor, double winBeg, double winEnd, double &offset){
    bool isFound = false;
    float posMid = (picWidth-1)/2;
    float posBeg = (picWidth-1) * winBeg;
    float posEnd = (picWidth-1) * winEnd;
    float leftMostObjPos = posEnd;
    for(vector<CamObject>::iterator it=cObjects.begin(); it!=cObjects.end(); ++it){
        if(it->mColor == enColor && it->mPosition.x < posEnd && it->mPosition.x > posBeg && it->mPosition.x < leftMostObjPos){
            offset = it->mPosition.x - posMid;
            leftMostObjPos = it->mPosition.x;
            isFound = true;
        }
    }
    return isFound;
}
