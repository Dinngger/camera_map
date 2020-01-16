#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>

using namespace cv;

#include "CameraCtl.hpp"

double getDistance(Point pointO, Point pointA) {
	double distance;
	distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
	distance = sqrtf(distance);
	return distance;
}

#define PARAM1 10
#define PARAM2 6

void get_trapezoids(Point2f corners[4], std::vector<std::vector<Point2f> > &trapezoids) {
	std::vector<Point2f> left_trapezoid;
	std::vector<Point2f> right_trapezoid;
	double d1 = getDistance(corners[0], corners[1]);
	double d2 = getDistance(corners[1], corners[2]);
	Point2f trapezoid_points[4];
	Point2f direction_vectors[2];
	Point2f midpoints[2];

	int i0 = 0;
	if (d1 < d2)
		i0 = 1;

	midpoints[0] = (corners[i0] + corners[i0 + 1]) / 2;
	midpoints[1] = (corners[i0 + 2] + corners[(i0 + 3) % 4]) / 2;
	Point2f vertical_vector = (midpoints[1]) - (midpoints[0]);
	vertical_vector = (i0 == 0 ? d1 : d2) * PARAM1 * vertical_vector / sqrt(vertical_vector.x*vertical_vector.x + vertical_vector.y*vertical_vector.y);
	direction_vectors[0] = corners[i0 + 1] - corners[i0];
	direction_vectors[1] = corners[(i0 + 3) % 4] - corners[i0 + 2];
	left_trapezoid.push_back(corners[i0]);
	left_trapezoid.push_back(corners[i0 + 1]);
	left_trapezoid.push_back(midpoints[0] - vertical_vector + PARAM2 * direction_vectors[0]);
	left_trapezoid.push_back(midpoints[0] - vertical_vector - PARAM2 * direction_vectors[0]);
	right_trapezoid.push_back(corners[i0 + 2]);
	right_trapezoid.push_back(corners[(i0 + 3) % 4]);
	right_trapezoid.push_back(midpoints[1] + vertical_vector + PARAM2 * direction_vectors[1]);
	right_trapezoid.push_back(midpoints[1] + vertical_vector - PARAM2 * direction_vectors[1]);
	trapezoids.push_back(left_trapezoid);
	trapezoids.push_back(right_trapezoid);
}

bool in_trapezoid(Point2f corners[4], const std::vector<Point2f> &trapezoid) {
	int count = 0;
	for (int i = 0; i<4; i++) {
		if (pointPolygonTest(trapezoid, corners[i], false) >= 0)
			count++;
	}
	//std::cout<<count<<std::endl;
	if (count == 4) {
		return true;
	}
	return false;
}

void draw_armor(RotatedRect rrect1, RotatedRect rrect2, Mat &img) {
	Point2f corners[8];
	rrect1.points(corners);
	rrect2.points(corners + 4);
	Point2f center = Point2f(0, 0);
	for (int i = 0;i < 8;i++) {
		center.x += corners[i].x;
		center.y += corners[i].y;
	}
	center.x = center.x / 8;
	center.y = center.x / 8;
	double distance_to_center[8];
	for (int i = 0;i < 8;i++) {
		distance_to_center[i] = getDistance(corners[i], center);
	}
	int index;
	double temp1;
	Point2f temp2;
	for (int i = 0;i < 8;i++) {
		index = i;
		for (int j = i + 1;j < 8;j++) {
			if (distance_to_center[index] > distance_to_center[j]) {
				index = j;
			}
		}
		if (index != i) {
			temp1 = distance_to_center[index];
			distance_to_center[index] = distance_to_center[i];
			distance_to_center[i] = temp1;

			temp2 = corners[index];
			corners[index] = corners[i];
			corners[i] = temp2;
		}
	}
	Point2f armor_points[4] = { corners[0], corners[1], corners[7], corners[6] };
	for (int i = 0; i < 4; i++) {
		line(img, armor_points[i], armor_points[(i + 1) % 4], Scalar(0, 0, 255), 2);
	}
}

void get_result(const std::vector<RotatedRect> &rrects, const std::vector<std::vector<Point2f> > &trapezoids, std::vector<RotatedRect> &results, int size, Mat &img) {
	bool used[size];
	bool flag[size][size];
	for (int i = 0; i < size; i++) {
		used[i] = false;
		for (int j = 0; j < size; j++)
			flag[i][j] = false;
	}
	for (int i = 0; i < size; i++) {
		Point2f corners[4];
		rrects[i].points(corners);
		for (int j = 0; j<size * 2; j++) {
			if (in_trapezoid(corners, trapezoids[j])) {
				flag[i][j / 2] = true;
				//std::cout<<i<<" "<<j<<std::endl;
			}
		}
	}
	for (int i = 0; i<size; i++) {
		//if (used[i])
		//	continue;
		for (int j = 0; j<size; j++) {
			//if (used[j])
			//	continue;
			if (flag[i][j] == true && flag[j][i] == true) {
				results.push_back(rrects[i]);
				results.push_back(rrects[j]);
				used[i] = true;
				used[j] = true;
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
		Mat src_channels[3];
		split(img, src_channels);
		grayImage = src_channels[0];
		Scalar meanValue;
		meanValue = mean(grayImage);
		bool lowExposure = meanValue.val[0] < mean_mean;
		mean_mean = mean_mean * 0.95 + 0.05 * meanValue.val[0];

		if (lowExposure) {
			rrects.clear();
			results.clear();
			threshold(grayImage, binary, 128, 255, CV_THRESH_BINARY);

			std::vector<std::vector<Point> > contours;
			std::vector<Vec4i> hierarchy;
			findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());

			for (int i = 0; i < contours.size(); i++) {
				//绘制轮廓的最小外结矩形
				double area = contourArea(contours[i]);
				if (area > 10) {
					RotatedRect rrect = minAreaRect(contours[i]);
					rrects.push_back(rrect);
				}
			}

			std::vector<std::vector<Point2f> > trapezoids;
			for (int i = 0; i < rrects.size(); i++) {
				Point2f corners[4];
				rrects[i].points(corners);
				get_trapezoids(corners, trapezoids);
			}
			get_result(rrects, trapezoids, results, rrects.size());
		} else {
			for (int i = 0; i < rrects.size(); i++) {
				Point2f corners[4];
				rrects[i].points(corners);
				for (int j = 0; j < 4; j++) {
					line(img, corners[j], corners[(j + 1) % 4], Scalar(0, 0, 255), 2);
				}
			}
			imshow("gray", img);
			if (waitKey(1) > 0)
				break;
		}
		else {
			for (int i = 0; i < results.size(); i++) {
				Point2f corners[4];
				results[i].points(corners);
				for (int j = 0; j < 4; j++) {
					line(img, corners[j], corners[(j + 1) % 4], Scalar(0, 0, 255), 2);
				}
			}
			imshow("Image", img);
            if (waitKey(1) == 'q')
				break;
		}
	}
	return 0;
}
