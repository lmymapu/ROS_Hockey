#include "Camera.h"
#include "trackbar.h"

#define CAMERA_IN_SIMULATOR

using namespace cv;
using namespace std;
turtlebotCamera::turtlebotCamera():it(nh)
{
    //nh.setCallbackQueue(&camera_queue);
#ifndef SIMULATION_MODE
    imSub = it.subscribe("/camera/rgb/image_raw", 10, &turtlebotCamera::imageCb, this);
#else
    imSub = it.subscribe("/image", 10, &turtlebotCamera::imageCb, this);
#endif
}

turtlebotCamera::turtlebotCamera(const turtlebotCamera &tC):it(nh),cv_ptr(tC.cv_ptr){
    //this->cv_ptr = tC.cv_ptr;
}

void turtlebotCamera::getCameraData(){
    camera_queue.callAvailable(ros::WallDuration(0));
}

void turtlebotCamera::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mImage = cv_ptr->image.clone();
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


void turtlebotCamera::detectObject(Color enColor, std::vector<CamObject> &cObjects)
{
    cv::Mat hsvImg;
    cv::Mat threshold;
    cv::Mat origImg;
    cv::Mat threshold1;
    origImg = mImage.clone();
    cv::cvtColor(origImg, hsvImg, cv::COLOR_BGR2HSV);

    //cv::Scalar lowRange;
    //cv::Scalar highRange;
    // find object using given ranges
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
    cObjects.resize(contours.size());
    for(size_t i=0, iend=contours.size(); i<iend; i++)
    {
        cObjects[i].mColor = enColor;
        cObjects[i].mPosition = mc[i];
        cObjects[i].mMoment = mu[i];
    }

    // draw the objects
    RNG rng(12345);
    for(size_t i=0, iend=cObjects.size(); i<iend; i++)
    {
        Scalar drawColor = Scalar(rng.uniform(0,255),rng.uniform(0,255), rng.uniform(0,255) );
        drawContours(origImg, contours, i, drawColor, 2, 8, hierarchy, 0, Point());
        circle(origImg, cObjects[i].mPosition, 10, drawColor);
    }

    //imwrite("Image.jpg", mImage);
    imshow("Image from camera", origImg);
    waitKey(10);
}

bool turtlebotCamera::objectInMiddle(Color enColor){
    return true;
}
