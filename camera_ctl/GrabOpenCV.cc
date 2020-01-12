#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <time.h>

using namespace cv;

#include "CameraCtl.hpp"


double getDistance(Point pointO, Point pointA) {
	double distance;
	distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
	distance = sqrtf(distance);
	return distance;
}

void get_trapezoids(Point2f corners[4], std::vector<std::vector<Point2f>> &trapezoids) {
	std::vector<Point2f> left_trapezoid;
	std::vector<Point2f> right_trapezoid;
	double d1 = getDistance(corners[0], corners[1]);
	double d2 = getDistance(corners[1], corners[2]);
	Point2f trapezoid_points[4];
	Point2f direction_vectors[2];
	Point2f vertical_vector;
	Point2f midpoints[2];
	if (d1 > d2) {
		midpoints[0] = (corners[0] + corners[1]) / 2;
		midpoints[1] = (corners[2] + corners[3]) / 2;
		Point2f vertical_vector = (midpoints[1]) - (midpoints[0]);
		vertical_vector = d1 * 5 * vertical_vector / sqrt(vertical_vector.x*vertical_vector.x + vertical_vector.y*vertical_vector.y);
		direction_vectors[0] = corners[1] - corners[0];
		direction_vectors[1] = corners[2] - corners[3];
		left_trapezoid.push_back(corners[0]);
		left_trapezoid.push_back(corners[1]);
		left_trapezoid.push_back(midpoints[0] + vertical_vector - direction_vectors[0]);
		left_trapezoid.push_back(midpoints[0] + vertical_vector + direction_vectors[0]);
		right_trapezoid.push_back(corners[2]);
		right_trapezoid.push_back(corners[3]);
		right_trapezoid.push_back(midpoints[1] - vertical_vector + direction_vectors[1]);
		right_trapezoid.push_back(midpoints[1] - vertical_vector - direction_vectors[1]);
		trapezoids.push_back(left_trapezoid);
		trapezoids.push_back(right_trapezoid);
	}
	else {
		midpoints[0] = (corners[1] + corners[2]) / 2;
		midpoints[1] = (corners[3] + corners[0]) / 2;
		Point2f vertical_vector = (midpoints[1]) - (midpoints[0]);
		vertical_vector = d1 * 5 * vertical_vector / sqrt(vertical_vector.x*vertical_vector.x + vertical_vector.y*vertical_vector.y);
		direction_vectors[0] = corners[2] - corners[1];
		direction_vectors[1] = corners[3] - corners[0];
		left_trapezoid.push_back(corners[1]);
		left_trapezoid.push_back(corners[2]);
		left_trapezoid.push_back(midpoints[0] + vertical_vector - direction_vectors[0]);
		left_trapezoid.push_back(midpoints[0] + vertical_vector + direction_vectors[0]);
		right_trapezoid.push_back(corners[3]);
		right_trapezoid.push_back(corners[0]);
		right_trapezoid.push_back(midpoints[1] - vertical_vector - direction_vectors[1]);
		right_trapezoid.push_back(midpoints[1] - vertical_vector + direction_vectors[1]);
		trapezoids.push_back(left_trapezoid);
		trapezoids.push_back(right_trapezoid);
	}
}

bool in_trapezoid(Point2f corners[4], std::vector<Point2f> trapezoid) {
	int count = 0;
	for (int i = 0; i<4; i++) {
		if (pointPolygonTest(trapezoid, corners[i], false)>0)
			count++;
	}
    //std::cout<<count<<std::endl;
	if (count == 4) {
		return true;
	}
	return false;
}

void get_result(std::vector<RotatedRect> rrects, std::vector<std::vector<Point2f>> trapezoids, std::vector<RotatedRect> &results, int size) {
	bool flag[size][size];
	for (int i = 0; i < size; i++)
        for(int j = 0;j < size; j++)
		    flag[i][j] = false;
	for (int i = 0; i<size; i++) {
		Point2f corners[4];
		rrects[i].points(corners);
		for (int j = 0; j<size*2; j++) {
			if (in_trapezoid(corners, trapezoids[j])) {
				flag[i][j/2] = true;
                //std::cout<<i<<" "<<j<<std::endl;
			}
		}
	}
	for (int i = 0; i<size; i++) {
		for (int j = 0; j<size; j++) {
			if (flag[i][j] == true && flag[j][i] == true) {
				results.push_back(rrects[i]);
				results.push_back(rrects[j]); 
			}
		}
	}
}



int main(int argc, char* argv[]) {
	CameraCtl camCtl;
	camCtl.startGrabbing();
	printf("Press any key to exit.\n");
	bool lowExposureTime = true;
	std::vector<RotatedRect> rrects;
	std::vector<RotatedRect> results;
	float mean_mean = 40;
	while (true) {
		Mat img = camCtl.getOpencvMat();
		camCtl.setExposureTime(lowExposureTime ? 7000 : 50);
		lowExposureTime = !lowExposureTime;
		Mat grayImage;
		Mat binary = img.clone();
		cvtColor(img, grayImage, COLOR_BGR2GRAY);
		Scalar meanValue;
		meanValue = mean(grayImage);
		printf("\rmean: %f\t", meanValue.val[0]);
		bool lowExposure = meanValue.val[0] < mean_mean;
		mean_mean = mean_mean * 0.95 + 0.05 * meanValue.val[0];

		if (lowExposure) {
			rrects.clear();
			results.clear();
			threshold(grayImage, binary, 243, 255, CV_THRESH_BINARY);

			std::vector<std::vector<Point>> contours;
			std::vector<Vec4i> hierarchy;
			findContours(grayImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());

			for (int i = 0; i < contours.size(); i++) {
				//绘制轮廓的最小外结矩形
				double area = contourArea(contours[i]);
				if (area > 5) {
					RotatedRect rrect = minAreaRect(contours[i]);
					rrects.push_back(rrect);
				}
			}
			// imshow("lowExposureImage", img);
			// if (waitKey(1) > 0)
			//     break;
			//Point2f trapezoids[size * 2][4];

			std::vector<std::vector<Point2f>> trapezoids;
			for (int i = 0; i < rrects.size(); i++) {
				Point2f corners[4];
				rrects[i].points(corners);
				get_trapezoids(corners, trapezoids);
			}
			get_result(rrects, trapezoids, results, rrects.size());
		}
		else {
			for (int i = 0; i < results.size(); i++) {
				Point2f corners[4];
				results[i].points(corners);
				for (int j = 0; j < 4; j++) {
					line(img, corners[j], corners[(j + 1) % 4], Scalar(0, 0, 255), 2);
				}
			}
            imshow("Image",img);
            for (int i = 0; i < rrects.size(); i++) {
                 Point2f corners[4];
                 rrects[i].points(corners);
                 for (int j = 0; j < 4; j++) {
                     line(img, corners[j], corners[(j + 1) % 4], Scalar(0, 0, 255), 2);
                 }
             }
			imshow("src", img);
			if (waitKey(1) > 0)
				break;
		}
	}
	return 0;
}
