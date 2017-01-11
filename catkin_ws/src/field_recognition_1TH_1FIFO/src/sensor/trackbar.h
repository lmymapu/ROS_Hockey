#ifndef TRACKBAR_H
#define TRACKBAR_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class Trackbar
{
public:
    Trackbar(std::string);

    //static cv::Scalar lowRange;
    //static cv::Scalar highRange;

    void startTrackbar();
    int hMin;
    int hMax;
    int sMin;
    int sMax;
    int vMin;
    int vMax;
    int threshold;

private:
    std::string name;

    //static void onTrackbar(int, void *);
};

#endif // TRACKBAR_H
