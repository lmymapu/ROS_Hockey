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
turtlebotCamera3D::turtlebotCamera3D():it(nh), mLength(0), mWidth(0), mMaxDistance(10),
    is2DImgAvailable(false),is3DImgAvailable(false)
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
    mLength = mImage.cols;
    mWidth = mImage.rows;

}

void turtlebotCamera3D::pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::unique_lock<std::mutex> lock(mMutexPointCloud);
    pcl::fromROSMsg(*msg,mPointCloud);
    printf("Number of points: %lu\n",mPointCloud.points.size());

}

std::vector<CamObject3D> turtlebotCamera3D::detectObject(Color enColor, cv::Scalar &lowRange,cv::Scalar &highRange,
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
    vector<CamObject3D> objects(contours.size());
    for(size_t i=0, iend=contours.size(); i<iend; i++)
    {
        objects[i].mColor = enColor;
        objects[i].mMassCenter = mc[i];
        objects[i].mType = puck;
    }

    // draw the objects
    RNG rng(12345);
    for(size_t i=0, iend=objects.size(); i<iend; i++)
    {
        Scalar drawColor = Scalar(rng.uniform(0,255),rng.uniform(0,255), rng.uniform(0,255) );
        drawContours(origImg, contours, i, drawColor, 2, 8, hierarchy, 0, Point());
        circle(origImg, objects[i].mMassCenter, 10, drawColor);
    }

    //imwrite("Image.jpg", mImage);
    imshow("Image from camera", origImg);
    waitKey(10);
    return objects;

}


void turtlebotCamera3D::detectObject(Color enColor, std::vector<CamObject3D> &cObjects)
{
    std::unique_lock<std::mutex> lock(mMutexImg);
    std::unique_lock<std::mutex> lock1(mMutexPointCloud);
    cv::Mat hsvImg;
    cv::Mat threshold;
    cv::Mat origImg;
    cv::Mat threshold1;
    origImg = mImage.clone();
//    ROS_INFO("Image size: %d, %d\n", origImg.cols, origImg.rows);
    imgWidth = origImg.cols; imgHeight = origImg.rows;
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
    vector<Point> mc(contours.size());
    for(size_t i=0, iend=contours.size(); i<iend; i++)
    {
        mc[i]=Point(round(mu[i].m10/mu[i].m00), round(mu[i].m01/mu[i].m00));
    }

    // get objects 3D position
    std::vector<pcl::PointXYZ> pointPose;
    vector<int> vNumPoints;
    vector<int> vTotalNumPoints;
    getPointPose(contours, pointPose, vNumPoints, vTotalNumPoints);

    // create detected object
    cObjects.resize(contours.size());
    for(size_t i=0, iend=contours.size(); i<iend; i++)
    {
        cObjects[i].mColor = enColor;
        cObjects[i].mMassCenter = mc[i];
        cObjects[i].mMoment = mu[i];
        cObjects[i].m3DPosition = pointPose[i];
        cObjects[i].mEulerDistance = sqrt(pointPose[i].x*pointPose[i].x
                                          +pointPose[i].z*pointPose[i].z);
        cObjects[i].mNumPoints = vNumPoints[i];
        cObjects[i].mTotalNumPoints = vTotalNumPoints[i];
    }

    // draw the objects
    RNG rng(12345);
    string text;

    for(size_t i=0, iend=cObjects.size(); i<iend; i++)
    {
        Scalar drawColor = Scalar(rng.uniform(0,255),rng.uniform(0,255), rng.uniform(0,255) );
        drawContours(origImg, contours, i, drawColor, CV_FILLED);
        circle(origImg, cObjects[i].mMassCenter, 10, drawColor);
    }
    /*
    // print 3D position of the object
    ptPosition =cObjects[i].mPosition.x + cObjects[i].mPosition.y*mLength;
    text = "x:"+to_string(mPointCloud.points[ptPosition].x)+
            "y:"+to_string(mPointCloud.points[ptPosition].y)+
            "z:"+to_string(mPointCloud.points[ptPosition].z) +"\n";
    //putText(origImg,text, cObjects[i].mPosition, FONT_HERSHEY_SIMPLEX, 0.5, drawColor);
    printf("x: %f, y:%f, z:%f\n",mPointCloud.points[ptPosition].x,
           mPointCloud.points[ptPosition].y, mPointCloud.points[ptPosition].z);
    printf("center position: x: %d, y:%d\n",cObjects[i].mPosition.x,
           cObjects[i].mPosition.y);
    printf("Point Number: %d\n",ptPosition);
    printf("length of image: %d\n", mLength);*/
    //imwrite("Image.jpg", mImage);
    imshow("Image from camera", origImg);
    waitKey(10);
}

void turtlebotCamera3D::detectObject(Color enColor){
    detectObject(enColor, cObjects3D);
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

void turtlebotCamera3D::printObjects()
{
    ROS_INFO("Number of detected object: %lu\n", cObjects3D.size());
    for(size_t i=0, iend=cObjects3D.size(); i<iend; i++)
    {
        ROS_INFO("Object %lu:\n", i);
        ROS_INFO("    Mass center x:%d, y:%d\n", cObjects3D[i].mMassCenter.x,
               cObjects3D[i].mMassCenter.y);
        ROS_INFO("    3D position x:%f, y:%f, z:%f\n", cObjects3D[i].m3DPosition.x,
               cObjects3D[i].m3DPosition.y, cObjects3D[i].m3DPosition.z);
        ROS_INFO("    Number of points: %d\n", cObjects3D[i].mNumPoints);
        ROS_INFO("    Total number of points: %d\n", cObjects3D[i].mTotalNumPoints);
    }
}

