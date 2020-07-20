#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;

const Vector3d armor_module[4] = {
    Vector3d(-0.065, -0.0285, 0),
    Vector3d(-0.065,  0.0285, 0),
    Vector3d( 0.065, -0.0285, 0),
    Vector3d( 0.065,  0.0285, 0)
};

Matrix3d K;

int main() {
    K << 1776.67168581218, 0, 0,
         0, 1778.59375346543, 0,
         0, 0, 1;
    ifstream fi("/home/dinger/mine/camera_map/USELESS/data.txt", ios::in);
    ofstream fo("/home/dinger/mine/camera_map/USELESS/data2.txt", ios::out);
    // rgb[27], dist, degree, r/b, l/r, x0, y0, x1, y1
    Eigen::Quaterniond r15(Eigen::AngleAxisd(M_PI / 12.0, Eigen::Vector3d(-1, 0, 0)));
    for (int i=0; i<54022; i++) {
        Mat m = Mat(Size(500, 500), CV_32FC3, Scalar(0, 0, 0));
        int rgb, distI, degreeI;
        for (int j=0; j<3; j++) {
            for (int q=0; q<3; q++) {
                Scalar a(0, 0, 0);
                for (int k=0; k<3; k++) {
                    fi >> rgb;
                    fo << rgb;
                    fo << " ";
                    a(k) = 1.0 * rgb / 255.0;
                }
                rectangle(m, Rect(235+j*10, 235+q*10, 10, 10), a, -1);
            }
        }
        fi >> distI >> degreeI;
        fo << distI << " " << degreeI << " ";
        double dist = distI;
        double degree = degreeI;
        Eigen::Quaterniond _r(Eigen::AngleAxisd(M_PI * degree / 180.0, Eigen::Vector3d(0, -1, 0)));
        double x0, y0;
        fi >> x0 >> y0;
        fi >> rgb;
        fo << rgb << " ";
        int x1I, y1I;
        fi >> x1I >> y1I;
        double x1 = x1I;
        double y1 = y1I;
        int lbp_id = x1 > x0 ? 1 : 0;
        fo << lbp_id << " ";
        Vector2d l[2];
        Vector2i li[2];
        for (int i=0; i<2; i++) {
            Vector3d p = _r.matrix() * (r15.matrix() * armor_module[lbp_id * 2 + i]);
            p(2) += dist / 1000;
            p = K * p;
            p /= p(2);
            l[i](0) = p(0) + x0 - x1 + 25;
            l[i](1) = p(1) + y0 - y1 + 25;
            fo << p(0) + x0 - x1 << " ";
            fo << p(1) + y0 - y1;
            if (i == 0)
                fo << " ";
            li[i] = (l[i] * 10).cast<int>();
        }
        fo << "\n";
        line(m, cv::Point(li[0](0), li[0](1)), cv::Point(li[1](0), li[1](1)), Scalar(0.5, 0.5, 0.5), 1, 16, 0);
        circle(m, Point(250, 250), 1, Scalar(1, 1, 1), -1);
        imshow("m", m);
        waitKey(0);
    }
    return 0;
}