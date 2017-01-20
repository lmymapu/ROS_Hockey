#include "trackbar.h"

using namespace cv;
using namespace std;
/*int Trackbar::hMin=0;
int Trackbar::hMax=0;
int Trackbar::sMin=0;
int Trackbar::sMax=0;
int Trackbar::vMin=0;
int Trackbar::vMax=0;
cv::Scalar Trackbar::lowRange=Scalar(hMin,sMin,vMin);
cv::Scalar Trackbar::highRange=Scalar(hMax,sMax,vMax);*/

Trackbar::Trackbar(string name):hMin(0),hMax(0),sMin(0),sMax(0),
    vMin(0),vMax(0),threshold(255), name(name)
{
}

void Trackbar::startTrackbar()
{
    namedWindow(name);

    // create trackbar
    createTrackbar("Hue Min", name, &hMin, threshold, NULL);
    createTrackbar("Hue Max", name, &hMax, threshold, NULL);
    createTrackbar("Sat Min", name, &sMin, threshold, NULL);
    createTrackbar("Sat Max", name, &sMax, threshold, NULL);
    createTrackbar("Val Min", name, &vMin, threshold, NULL);
    createTrackbar("Val Max", name, &vMax, threshold, NULL);
}
/*
void Trackbar::onTrackbar(int, void*)
{
  cv::Scalar a = Scalar(hMin,sMin,vMin);
    //lowRange=Scalar(hMin,sMin,vMin);
    highRange=a;//Scalar(hMax,sMax,vMax);
}*/
