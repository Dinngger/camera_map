#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include "cam_map.hpp"

using namespace cv;

int main()
{
    Map map;
    string fileName = "../map.xml";
    ReadXmlFile(fileName, map);
    Mat image = Mat::zeros(800, 800, CV_8UC3);
    imshow("map", image);
    waitKey(0);
    destroyWindow("map");
    return 0;
}
