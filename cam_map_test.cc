#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include "cam_map.hpp"
#include <stdio.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

using namespace cv;
using namespace std;

int main()
{
    Map map;
    string fileName = "../map.xml";
    ReadXmlFile(fileName, map);

    char prefix[1] = {'4'};
    Mat img, img600, pointed_map;
    Mat map_img = imread("../map.png");
    VideoWriter writer("../output.avi", CV_FOURCC('M','J','P','G'),
                60, Size(1400, 600), true);
    for (int i=1; i<2083; i++) {
        char name[54] = {};
        char name2[50] = {};
        sprintf(name, "/home/yi03/darknet/car4/%s_%08d.jpg", prefix, i);
    	img = imread(name);
        resize(img, img600, cv::Size(800, 600));

        map_img.copyTo(pointed_map);
        strncpy(name2, name, 34);
        ifstream in(name2);
        string line;
        if (in) {
            while (getline(in, line)) {
                int p[4];
                stringstream ss;
                ss << line;
                for (int i=0; i<4; i++)
                    ss >> p[i];
                _Point p_map;
                if (map.draw_point(p, p_map)) {
                    printf("\r%d,%d  %d/2082", p_map.x, p_map.y, i);
                    circle(pointed_map, Point(p_map.x, p_map.y), 1, Scalar(255, 0, 0), -1);
                }
            }
        }
        in.close();

        Mat comb_img(600, 1400, CV_8UC3);
        Mat left(comb_img, Rect(0, 0, 800, 600));
        img600.copyTo(left);
        Mat right(comb_img, Rect(800, 0, 600, 600));
        pointed_map.copyTo(right);
        writer.write(comb_img);
        // imshow("video", comb_img);
        // waitKey(1);
    }
    writer.release();
    printf("\noutput finished!\n");
    // destroyWindow("video");
    return 0;
}
