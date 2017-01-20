#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <math.h>
#include <numeric>
#include <algorithm>
#include "3DCamera.h"
#include "trackbar.h"
#include "sensor_msgs/PointCloud2.h"

using namespace cv;
using namespace std;
turtlebotCamera3D::turtlebotCamera3D():it(nh), mLength(0), mWidth(0), mNumPixels(0), mFirstInitialization(true),
    mMaxDistance(10),Cam_rate(10),camMidRegionArea(CAM_MID_REGION),is2DImgAvailable(false),
    is3DImgAvailable(false)
{
    imSub = it.subscribe("/camera/rgb/image_raw", CAM3D_IMG_FIFO_LENGTH, &turtlebotCamera3D::imageCb, this);
    pcSub = nh3D.subscribe("/camera/depth/points", CAM3D_PTCLOUD_FIFO_LENGTH, &turtlebotCamera3D::pointCloudCb, this);
    /*sensor_msgs::PointCloud2 a;
    a.data.reverse_iterator
    /*mTrackbar = new Trackbar("HSV Trackbar1");
    mTrackbar1 = new Trackbar("HSV Trackbar2");
    mTrackbar->startTrackbar();
    mTrackbar1->startTrackbar();*/
}

void turtlebotCamera3D::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::unique_lock<std::mutex> lock(mMutexImg);
    mImage = cv_ptr->image.clone();
    if(mFirstInitialization)
    {
        mLength = mImage.cols;
        mWidth = mImage.rows;
        mNumPixels = mLength*mWidth;
        mFirstInitialization=false;
    }
    is2DImgAvailable = true;
}

void turtlebotCamera3D::pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::unique_lock<std::mutex> lock(mMutexPointCloud);
    pcl::fromROSMsg(*msg,mPointCloud);
    //printf("Number of points: %lu\n",mPointCloud.points.size());
    is3DImgAvailable = true;
}

void turtlebotCamera3D::detectObject2D(Color enColor, std::vector<CamObject3D> &cObjects)
{
    std::unique_lock<std::mutex> lock(mMutexImg);
    std::unique_lock<std::mutex> lock1(mMutexPointCloud);
    cv::Mat hsvImg;
    cv::Mat threshold;
    cv::Mat origImg;
    cv::Mat threshold1;
    origImg = mImage.clone();
    //printf("Image size: %d, %d\n", origImg.cols, origImg.rows);
    cv::cvtColor(origImg, hsvImg, cv::COLOR_BGR2HSV);
    imgWidth = origImg.cols; imgHeight = origImg.rows;

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
    vector<Point> mc(contours.size());
    for(size_t i=0, iend=contours.size(); i<iend; i++)
    {
        mc[i]=Point(round(mu[i].m10/mu[i].m00), round(mu[i].m01/mu[i].m00));
    }

    // get top points of each contours
    vector<cv::Point> pTopPoints;
    getTopPoint2D(contours, pTopPoints);
    // create detected object
    cObjects.resize(contours.size());
    for(size_t i=0, iend=contours.size(); i<iend; i++)
    {
        cObjects[i].mColor = enColor;
        cObjects[i].mMassCenter = mc[i];
        cObjects[i].mMoment = mu[i];
        cObjects[i].mTopPoint = pTopPoints[i];
        if(pTopPoints[i].y > CAM3D_FLOOR_THRESHOLD*mWidth)
            cObjects[i].mOnFloor = true;
        else
            cObjects[i].mOnFloor = false;
    }
#ifdef DEBUG_ON
    // draw the objects
    RNG rng(12345);
    string text;

    for(size_t i=0, iend=cObjects.size(); i<iend; i++)
    {
        Scalar drawColor = Scalar(rng.uniform(0,255),rng.uniform(0,255), rng.uniform(0,255) );
        drawContours(origImg, contours, i, drawColor, CV_FILLED);
        circle(origImg, cObjects[i].mMassCenter, 10, drawColor);
    }

    imshow("Image from camera", origImg);
    waitKey(10);
#endif
    is2DImgAvailable = false;
}

void turtlebotCamera3D::detectObject2D(Color enColor){
#ifdef DEBUG_ON
    ros::Time start = ros::Time::now();
#endif
    detectObject2D(enColor, cObjects3D);
#ifdef DEBUG_ON
    ros::Time end = ros::Time::now();
    ROS_INFO_STREAM("3D image processing has taken time: "<< end-start);
#endif
}

void turtlebotCamera3D::detectObject3D(Color enColor, std::vector<CamObject3D> &cObjects)
{
    std::unique_lock<std::mutex> lock(mMutexImg);
    std::unique_lock<std::mutex> lock1(mMutexPointCloud);
    cv::Mat hsvImg;
    cv::Mat threshold;
    cv::Mat origImg;
    cv::Mat threshold1;
    origImg = mImage.clone();
    //printf("Image size: %d, %d\n", origImg.cols, origImg.rows);
    cv::cvtColor(origImg, hsvImg, cv::COLOR_BGR2HSV);
    imgWidth = origImg.cols; imgHeight = origImg.rows;

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
    vector<Point> mc(contours.size());
    for(size_t i=0, iend=contours.size(); i<iend; i++)
    {
        mc[i]=Point(round(mu[i].m10/mu[i].m00), round(mu[i].m01/mu[i].m00));
    }

    // get objects 3D position
    std::vector<pcl::PointXYZ> pointPose;
    std::vector<pcl::PointXYZ> pointMedianPose;
    vector<int> vNumPoints;
    vector<int> vTotalNumPoints;
    vector<cv::Point> pTopPoint;
    getPointPose(contours, pointPose, vNumPoints, vTotalNumPoints);
    getMedianPointPose(contours, pointMedianPose, pTopPoint);

    // create detected object
    cObjects.resize(contours.size());
    for(size_t i=0, iend=contours.size(); i<iend; i++)
    {
        cObjects[i].mColor = enColor;
        cObjects[i].mMassCenter = mc[i];
        cObjects[i].mMoment = mu[i];
        cObjects[i].m3DPosition = pointPose[i];
        cObjects[i].m3DPositionMedian = pointMedianPose[i];
        cObjects[i].mEulerDistance = sqrt(pointPose[i].x*pointPose[i].x
                                          +pointPose[i].z*pointPose[i].z);
        cObjects[i].mEulerDistanceMedian = sqrt(pointMedianPose[i].x*pointMedianPose[i].x
                                          +pointMedianPose[i].z*pointMedianPose[i].z);
        cObjects[i].mNumPoints = vNumPoints[i];
        cObjects[i].mTotalNumPoints = vTotalNumPoints[i];
        cObjects[i].mAreaInImage = float(vTotalNumPoints[i]) / float(mNumPixels);
        cObjects[i].mTopPoint = pTopPoint[i];
        if(pTopPoint[i].y > CAM3D_FLOOR_THRESHOLD*mWidth)
            cObjects[i].mOnFloor = true;
        else
            cObjects[i].mOnFloor = false;
    }

    // sort the objects using 2D distances
    std::sort(cObjects.begin(), cObjects.end());
#ifdef DEBUG_ON
    // draw the objects
    RNG rng(12345);
    string text;

    for(size_t i=0, iend=cObjects.size(); i<iend; i++)
    {
        Scalar drawColor = Scalar(rng.uniform(0,255),rng.uniform(0,255), rng.uniform(0,255) );
        drawContours(origImg, contours, i, drawColor, CV_FILLED);
        circle(origImg, cObjects[i].mMassCenter, 10, drawColor);
    }

    imshow("Image from camera", origImg);
    waitKey(10);
#endif
    setImgDataDirty();
}

void turtlebotCamera3D::detectObject3D(Color enColor){
#ifdef DEBUG_ON
    ros::Time start = ros::Time::now();
#endif
    detectObject3D(enColor, cObjects3D);
#ifdef DEBUG_ON
    ros::Time end = ros::Time::now();
    ROS_INFO_STREAM("3D image processing has taken time: "<< end-start);
#endif
}

bool turtlebotCamera3D::isEmpty()
{
    return mImage.empty() || mPointCloud.empty();
}

void turtlebotCamera3D::getPointPose(const std::vector<std::vector<cv::Point> >& contours,
                        std::vector<pcl::PointXYZ>& pointPose, std::vector<int>& aviNumPoints,
                                   std::vector<int>& aviTotalNumPoints)
{
    //std::unique_lock<std::mutex> lock(mMutexPointCloud);
    // get pixel coordinates within the contours
    vector<vector<Point>> locations;
    locations.resize(contours.size());

    for(size_t i=0, iend=contours.size(); i<iend; i++)
    {
        Mat cImg = Mat::zeros(mWidth, mLength, CV_8UC1);
        drawContours(cImg, contours, i, cv::Scalar(255), CV_FILLED);
        threshold(cImg, cImg, 100, 255, THRESH_BINARY);
        findNonZero(cImg, locations[i]);
    }
    // extract point position from point cloud
    for(size_t i=0, iend=locations.size(); i<iend; i++)
    {
        const vector<Point>& contour = locations[i];

        aviTotalNumPoints.push_back(int(contour.size()));
        // calculate mean and standard deviation
        float mean=0, stdDeviation=0;
        vector<float> ithContourDistance;
        vector<int> pointIndices;

        // discard nan points
        for(size_t j=0, jend=contour.size(); j<jend; j++)
        {
            int n;
            n = contour[j].x + contour[j].y*mLength;
            float jthDistance = sqrt(mPointCloud.points[n].x*mPointCloud.points[n].x+
                                     mPointCloud.points[n].y*mPointCloud.points[n].y+
                                     mPointCloud.points[n].z*mPointCloud.points[n].z);

            if(jthDistance < mMaxDistance)
            {
                ithContourDistance.push_back(jthDistance);
                pointIndices.push_back(n);
            }
        }

        // mean value
        size_t arrLength = ithContourDistance.size();
        double sumDistance=accumulate(ithContourDistance.begin(),ithContourDistance.end(), 0.0);
        mean = float(sumDistance/arrLength);

        // standard deviation
        vector<float> diff(arrLength);
        transform(ithContourDistance.begin(),ithContourDistance.end(), diff.begin(),
                  [mean](float x){return x-mean;});
        double sqSum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        stdDeviation = sqrt(sqSum/arrLength);

        // discard those points which outside the [mean-stdDeviation,mean+stdDeviation]
        int numPoints=0;
        vector<double> ithPoint{0.0,0.0,0.0};

        for(size_t i=0, iend=ithContourDistance.size(); i<iend; i++)
        {
            float leftBorder = mean - stdDeviation/4;
            float rightBorder = mean + stdDeviation/4;
            if(leftBorder<ithContourDistance[i] < rightBorder)
            {
                numPoints++;
                ithPoint[0] += mPointCloud.points[pointIndices[i]].x;
                ithPoint[1] += mPointCloud.points[pointIndices[i]].y;
                ithPoint[2] += mPointCloud.points[pointIndices[i]].z;
            }
        }

        if(numPoints > 0)
        {
            ithPoint[0] = ithPoint[0] / numPoints;
            ithPoint[1] = ithPoint[1] / numPoints;
            ithPoint[2] = ithPoint[2] / numPoints;
        }

        pcl::PointXYZ ithPointPose{float(ithPoint[0]),float(ithPoint[1]),float(ithPoint[2])};
        pointPose.push_back(ithPointPose);
        aviNumPoints.push_back(numPoints);
    }
}

void turtlebotCamera3D::getMedianPointPose(const std::vector<std::vector<cv::Point> >& contours,
                        std::vector<pcl::PointXYZ>& pointPose, std::vector<Point> &aTopPoint)
{
    //std::unique_lock<std::mutex> lock(mMutexPointCloud);
    // get pixel coordinates within the contours
    vector<vector<Point>> locations;
    locations.resize(contours.size());
    aTopPoint.resize(contours.size());
    for(size_t i=0, iend=contours.size(); i<iend; i++)
    {
        Mat cImg = Mat::zeros(mWidth, mLength, CV_8UC1);
        drawContours(cImg, contours, i, cv::Scalar(255), CV_FILLED);
        threshold(cImg, cImg, 100, 255, THRESH_BINARY);
        findNonZero(cImg, locations[i]);
    }
    // extract point position from point cloud
    for(size_t i=0, iend=locations.size(); i<iend; i++)
    {
        const vector<Point>& contour = locations[i];

        // find point with median distance
        vector<float> ithContourDistance;
        vector<int> pointIndices;
        vector<float> ithPoint{0.0,0.0,0.0};
        cv::Point topPoint(mLength, mWidth);

        for(size_t j=0, jend=contour.size(); j<jend; j++)
        {
            int n;
            n = contour[j].x + contour[j].y*mLength;
            float jthDistance = sqrt(mPointCloud.points[n].x*mPointCloud.points[n].x+
                                     mPointCloud.points[n].y*mPointCloud.points[n].y+
                                     mPointCloud.points[n].z*mPointCloud.points[n].z);

            if(jthDistance < mMaxDistance)
            {
                ithContourDistance.push_back(jthDistance);
                pointIndices.push_back(n);
            }

            if(topPoint.y > contour[j].y)
                topPoint = contour[j];
        }
        aTopPoint[i] = topPoint;


        // find median distance
        if(ithContourDistance.size()>0)
        {
            vector<float> medianDistance=ithContourDistance;
#ifdef _DEBUG_MODE_
            ROS_INFO("Distance befor sorting: %f", medianDistance[medianDistance.size()/2]);
#endif
            std::nth_element(medianDistance.begin(),medianDistance.begin()+medianDistance.size()/2,
                             medianDistance.end());
#ifdef _DEBUG_MODE_
            ROS_INFO("Distance after sorting: %f", medianDistance[medianDistance.size()/2]);
#endif
            vector<float>::iterator itOfMedian = std::find(ithContourDistance.begin(),ithContourDistance.end(),
                                                           medianDistance[medianDistance.size()/2]);
            int indexPt = itOfMedian - ithContourDistance.begin();
            ithPoint[0] = mPointCloud.points[pointIndices[indexPt]].x;
            ithPoint[1] = mPointCloud.points[pointIndices[indexPt]].y;
            ithPoint[2] = mPointCloud.points[pointIndices[indexPt]].z;
        }

        pcl::PointXYZ ithPointPose{ithPoint[0],ithPoint[1],ithPoint[2]};
        pointPose.push_back(ithPointPose);
    }
}

void turtlebotCamera3D::getTopPoint2D(const std::vector<std::vector<cv::Point> >& acontours,std::vector<cv::Point>& aTopPoint)
{
    // get pixel coordinates within the contours
    vector<vector<Point>> locations;
    locations.resize(acontours.size());
    aTopPoint.resize(acontours.size());
    for(size_t i=0, iend=acontours.size(); i<iend; i++)
    {
        Mat cImg = Mat::zeros(mWidth, mLength, CV_8UC1);
        drawContours(cImg, acontours, i, cv::Scalar(255), CV_FILLED);
        threshold(cImg, cImg, 100, 255, THRESH_BINARY);
        findNonZero(cImg, locations[i]);
    }

    // get top point
    for(size_t i=0, iend=locations.size(); i<iend; i++)
    {
        const vector<Point>& contour = locations[i];
        cv::Point topPoint(mLength, mWidth);

        for(size_t j=0, jend=contour.size(); j<jend; j++)
        {
            if(topPoint.y > contour[j].y)
                topPoint = contour[j];
        }
        aTopPoint[i] = topPoint;
    }
}


void turtlebotCamera3D::printObjects()
{
    cartesianCoordinate objPose;
    ROS_INFO("Number of detected object: %lu\n", cObjects3D.size());
    for(size_t i=0, iend=cObjects3D.size(); i<iend; i++)
    {
        cartesianCoordinate objPose;
        ROS_INFO("Object %lu:", i);
        ROS_INFO("    Mass center x:%d, y:%d", cObjects3D[i].mMassCenter.x,
               cObjects3D[i].mMassCenter.y);
        ROS_INFO("    3D position x:%f, y:%f, z:%f", cObjects3D[i].m3DPosition.x,
               cObjects3D[i].m3DPosition.y, cObjects3D[i].m3DPosition.z);
        ROS_INFO("    3D median position x:%f, y:%f, z:%f", cObjects3D[i].m3DPositionMedian.x,
               cObjects3D[i].m3DPositionMedian.y, cObjects3D[i].m3DPositionMedian.z);
        ROS_INFO("    Number of points: %d", cObjects3D[i].mNumPoints);
        ROS_INFO("    Total number of points: %d", cObjects3D[i].mTotalNumPoints);
        ROS_INFO("    Area percentage of object: %f", cObjects3D[i].mAreaInImage);
        ROS_INFO("    Top point: x=%d, y=%d", cObjects3D[i].mTopPoint.x, cObjects3D[i].mTopPoint.y);
        if(cObjects3D[i].mOnFloor)
            ROS_INFO("    Object on floor: yes");
        else
            ROS_INFO("    Object on floor: no");
        /*objPose = cObjects3D[i].getWorldPose(0);
        ROS_INFO("    World Coordinate: [%f, %f]", objPose.x, objPose.y);*/
        /*objPose = cObjects3D[i].getOdomPose(0);
        ROS_INFO("    Odom Coordinate: [%f, %f]", objPose.x, objPose.y);*/
    }
}

bool turtlebotCamera3D::objectInMiddle(Color enColor, double &offset){
    bool isFound = false;
    int midIndexBegin = floor(imgWidth * (1 - camMidRegionArea) / 2);
    int midIndexEnd = floor(imgWidth * (1 + camMidRegionArea) / 2);
    for(vector<CamObject3D>::iterator it=cObjects3D.begin(); it!=cObjects3D.end(); ++it){
        if(it->mMassCenter.x > midIndexBegin && it->mMassCenter.x < midIndexEnd && it->mColor == enColor && !(it->mOnFloor)){
            offset = it->mMassCenter.x - (imgWidth - 1)/2;
            isFound = true;
            return isFound;
        }
    }
    return isFound;
}

bool turtlebotCamera3D::rightMostObj(Color enColor, double winBeg, double winEnd, double &offset){
    bool isFound = false;
    float posMid = (imgWidth-1)/2;
    float posBeg = (imgWidth-1) * winBeg;
    float posEnd = (imgWidth-1) * winEnd;
    float rightMostObjPos = posBeg;
    for(vector<CamObject3D>::iterator it=cObjects3D.begin(); it!=cObjects3D.end(); ++it){
        if(it->mColor == enColor && it->mMassCenter.x < posEnd && it->mMassCenter.x > posBeg &&
                it->mMassCenter.x > rightMostObjPos && !(it->mOnFloor)){
            offset = it->mMassCenter.x - posMid;
            rightMostObjPos = it->mMassCenter.x;
            isFound = true;
        }
    }
    return isFound;
}

bool turtlebotCamera3D::rightMostObj(Color enColor, double winBeg, double winEnd, CamObject3D &obj){
    bool isFound = false;
    float posMid = (imgWidth-1)/2;
    float posBeg = (imgWidth-1) * winBeg;
    float posEnd = (imgWidth-1) * winEnd;
    float rightMostObjPos = posBeg;
    for(vector<CamObject3D>::iterator it=cObjects3D.begin(); it!=cObjects3D.end(); ++it){
        if(it->mColor == enColor && it->mMassCenter.x < posEnd && it->mMassCenter.x > posBeg &&
                it->mMassCenter.x > rightMostObjPos && !(it->mOnFloor)){
            obj = *it;
            rightMostObjPos = it->mMassCenter.x;
            isFound = true;
        }
    }
    return isFound;
}

bool turtlebotCamera3D::rightMostObj_blk(Color enColor, double winBeg, double winEnd, CamObject3D &obj){
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
        if(isImgDataReady()){
            detectObject3D(enColor);
            return rightMostObj(enColor, winBeg, winEnd, obj);
        }else{
            Cam_rate.sleep();
        }
    }
}

bool turtlebotCamera3D::rightMostObj_blk(Color enColor, double winBeg, double winEnd, double &offset){
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
        if(isImgDataReady()){
            detectObject3D(enColor);
            return rightMostObj(enColor, winBeg, winEnd, offset);
        }else{
            Cam_rate.sleep();
        }
    }
}

bool turtlebotCamera3D::leftMostObj(Color enColor, double winBeg, double winEnd, double &offset){
    bool isFound = false;
    float posMid = (imgWidth-1)/2;
    float posBeg = (imgWidth-1) * winBeg;
    float posEnd = (imgWidth-1) * winEnd;
    float leftMostObjPos = posEnd;
    for(vector<CamObject3D>::iterator it=cObjects3D.begin(); it!=cObjects3D.end(); ++it){
        if(it->mColor == enColor && it->mMassCenter.x < posEnd && it->mMassCenter.x > posBeg &&
                it->mMassCenter.x < leftMostObjPos && !(it->mOnFloor)){
            offset = it->mMassCenter.x - posMid;
            leftMostObjPos = it->mMassCenter.x;
            isFound = true;
        }
    }
    return isFound;
}

bool turtlebotCamera3D::leftMostObj(Color enColor, double winBeg, double winEnd, CamObject3D &obj){
    bool isFound = false;
    float posMid = (imgWidth-1)/2;
    float posBeg = (imgWidth-1) * winBeg;
    float posEnd = (imgWidth-1) * winEnd;
    float leftMostObjPos = posEnd;
    for(vector<CamObject3D>::iterator it=cObjects3D.begin(); it!=cObjects3D.end(); ++it){
        if(it->mColor == enColor && it->mMassCenter.x < posEnd && it->mMassCenter.x > posBeg &&
                it->mMassCenter.x < leftMostObjPos && !(it->mOnFloor)){
            obj = *it;
            leftMostObjPos = it->mMassCenter.x;
            isFound = true;
        }
    }
    return isFound;
}

bool turtlebotCamera3D::leftMostObj_blk(Color enColor, double winBeg, double winEnd, CamObject3D &obj){
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
        if(isImgDataReady()){
            detectObject3D(enColor);
            return leftMostObj(enColor, winBeg, winEnd, obj);
        }else{
            Cam_rate.sleep();
        }
    }
}

bool turtlebotCamera3D::smallestZObj(Color enColor, double winBeg, double winEnd, CamObject3D &obj){
    float minZ = 100;
    bool isFound = false;
    float posBeg = (imgWidth-1) * winBeg;
    float posEnd = (imgWidth-1) * winEnd;
    for(vector<CamObject3D>::iterator it=cObjects3D.begin(); it!=cObjects3D.end(); ++it){
        if(it->mColor == enColor && it->mMassCenter.x < posEnd && it->mMassCenter.x > posBeg &&
                it->m3DPositionMedian.z < minZ && it->m3DPositionMedian.z != 0 && !(it->mOnFloor)){
            obj = *it;
            minZ = it->m3DPositionMedian.z;
            isFound = true;
        }
    }
    return isFound;
}

bool turtlebotCamera3D::smallestZObj_blk(Color enColor, double winBeg, double winEnd, CamObject3D &obj){
    while(ros::ok()){
#ifdef SINGLE_FIFO_TH_DATAREQ
        ros::spinOnce();
#endif
        if(isImgDataReady()){
            detectObject3D(enColor);
            return smallestZObj(enColor, winBeg, winEnd, obj);
        }else{
            Cam_rate.sleep();
        }
    }
}

void turtlebotCamera3D::eliminateInvalidData(){
    vector<CamObject3D> objs(0);
    for(vector<CamObject3D>::iterator it=cObjects3D.begin(); it!=cObjects3D.end(); ++it){
        if(it->m3DPositionMedian.z!=0 && !(it->mOnFloor)){
            objs.push_back(*it);
        }
    }
    cObjects3D.clear();
    for(vector<CamObject3D>::iterator it=objs.begin(); it!=objs.end(); ++it){
        if(it->m3DPositionMedian.z!=0){
            cObjects3D.push_back(*it);
        }
    }
}
