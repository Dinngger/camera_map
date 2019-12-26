#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include "cam_map.hpp"
#include <stdio.h>
#include <iostream>
#include <string>
#include <fstream>

using namespace cv;
using namespace std;

int read_text(char *filename, string (&texts)[10])
{
    ifstream in(filename);
    string line;
    int count = 0;
    if (in) {
        while (getline(in, line)) {
            texts[count] = line;
            count++;
        }
    }
    in.close();
    return 0;
}

int main()
{
    Map map;
    string fileName = "../map.xml";
    ReadXmlFile(fileName, map);

    char prefix[1] = {'3'};
    Mat img, img600;
    Mat map_img = imread("../map.png");
    for (int i=1; i<2083; i++) {
        char name[54] = {};
        char name2[50] = {};
        sprintf(name, "/home/yi03/darknet/pic3/%s_%08d.jpg", prefix, i);
    	img = imread(name);
        resize(img, img600, cv::Size(600, 600));

        string texts[10];
        strncpy(name2, name, 34);
        read_text(name2, texts);

        Mat comb_img(600, 1200, CV_8UC3);
        Mat left(comb_img, Rect(0, 0, 600, 600));
        img600.copyTo(left);
        Mat right(comb_img, Rect(600, 0, 600, 600));
        map_img.copyTo(right);
        imshow("video", comb_img);
        waitKey(1);
    }
    destroyWindow("video");
    return 0;
}
