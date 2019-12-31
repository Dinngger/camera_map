#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <time.h>
#include <string>
#include <fstream>

using namespace cv;
using namespace std;

struct Proposal {
	double w, h;
	double x, y;
	double maxy, miny;
	RotatedRect rect;
	double area;
	double angle;
	double dx, dy;
};

void setLabel(cv::Mat& im, string label, Proposal re, Proposal re2, int i, int j)
{
	//转化为轮廓
	Point2f corners[4];
	re.rect.points(corners);
	Point2f *lastItemPointer = (corners + sizeof corners / sizeof corners[0]);
	vector<Point2f> contour(corners, lastItemPointer);

	Point2f corners2[4];
	re2.rect.points(corners2);
	Point2f *lastItemPointer2 = (corners2 + sizeof corners2 / sizeof corners2[0]);
	vector<Point2f> contour2(corners2, lastItemPointer2);

	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;
	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2) + (j - 1) * 15);
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255), cv::FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);

	r = cv::boundingRect(contour2);

	cv::Point pt2(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2) + i * 15);
	cv::rectangle(im, pt2 + cv::Point(0, baseline), pt2 + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255), cv::FILLED);
	cv::putText(im, label, pt2, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

double getDistance(Point pointO, Point pointA)
{
	double distance;
	distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
	distance = sqrtf(distance);

	return distance;
}

void get_contours_rect(vector<RotatedRect>& rects, vector<vector<Point>> contours, int flag) {
	for (int i = 0;i < contours.size();i++)
	{
		double area = contourArea(contours[i]);
		if (flag == 1 && area < 20)
			continue;
		if (area > 2000)
			continue;
		//绘制轮廓的最小外结矩形
		RotatedRect white_rect = minAreaRect(contours[i]);
		rects.push_back(white_rect);
	}
	if (rects.size() == 0)
		cout << "error!--------------------" << endl;
}

Mat equalizeChannelHist(const Mat & inputImage)
{
	if (inputImage.channels() >= 3)
	{
		vector<Mat> channels;
		split(inputImage, channels);
		Mat B, G, R;
		equalizeHist(channels[0], B);
		//B = channels[0];
		equalizeHist(channels[1], G);
		equalizeHist(channels[2], R);
		vector<Mat> combined;
		combined.push_back(B);
		combined.push_back(G);
		combined.push_back(R);
		Mat result;
		merge(combined, result);
		return result;
	}

	return Mat();
}

bool DoesRectangleContainPoint(RotatedRect rectangle, Point2f point) {

	//Get the corner points.
	Point2f corners[4];
	rectangle.points(corners);

	//Convert the point array to a vector.
	Point2f* lastItemPointer = (corners + sizeof corners / sizeof corners[0]);
	vector<Point2f> contour(corners, lastItemPointer);
	//Check if the point is within the rectangle.
	double indicator = pointPolygonTest(contour, point, false);
	bool rectangleContainsPoint = (indicator >= 0);
	return rectangleContainsPoint;
}

void rects2_in_rects1(vector<RotatedRect> rects1, vector<RotatedRect> rects2, vector<RotatedRect>& final_rects, int flag) {
	for (int i = 0;i < rects1.size();i++)
		for (int j = 0;j < rects2.size();j++)
			if (DoesRectangleContainPoint(rects1[i], rects2[j].center))
				if (flag == 1) {
					final_rects.push_back(rects1[i]);
					break;
				}
				else {
					final_rects.push_back(rects2[j]);
				}
}
void show_RotatedRects(vector<RotatedRect> rects, Mat img, string name) {
	for (int i = 0;i < rects.size();i++) {
		Point2f four_cornor[4];
		rects[i].points(four_cornor);
		for (int j = 0;j < 4;j++)  //把RotatedRect画出来
		{
			line(img, four_cornor[j], four_cornor[(1 + j) % 4], Scalar(0, 255, 0), 1, 8, 0);
		}
	}
	imshow(name, img);
}

int main() {
	int begintime, endtime, middletime1, middletime2;
	begintime = clock();
	string basepath = "F:/car2/";
	int totalFrameNumber = 2082;
	/*VideoCapture cap("C:\\Users\\allegra\\source\\repos\\Project6\\Project6\\task2\\armor1.mkv");
	double totalFrameNumber = cap.get(CAP_PROP_FRAME_COUNT);*/
	string jpgsname = basepath + "jpgs.txt";
	ifstream f1(jpgsname);
	string line1;
	vector<int> points;
	if (f1) {
		while (getline(f1, line1)) // line中不包括每行的换行符
		{
			middletime1 = clock();
			string picname = basepath + line1 + ".jpg";
			Mat src = imread(picname);
			string labelname = basepath + line1 + ".txt";
			cout << picname << endl;
			ifstream f(labelname);
			string line0;
			vector<int> points;
			if (f) {
				while (getline(f, line0)) // line中不包括每行的换行符
				{
					points.push_back(stoi(line0));
				}
			}
			else
			{
				cout << "no " + labelname + " file" << endl;
			}
			Rect rect(Point(points[0], points[1]), Point(points[2], points[3]));
			Mat roi = src(rect);
			
			Mat roi2 = roi.clone();
			Mat roi3 = roi.clone();
			Mat roi4 = roi.clone();
			Mat roi5 = roi.clone();
			Mat roi6 = roi.clone();
			vector<Mat> src_channels(3);
			split(roi, src_channels);
			Mat blue = (src_channels[0] - 0.4 * src_channels[1] - 0.7 * src_channels[2] > 80);//提蓝色

			Mat white = (src_channels[0] >200 & src_channels[1] >200 & src_channels[2] >200);//提白色

			Mat plus = blue + white;//蓝色和白色加一起

									//寻找最外层轮廓
			vector<vector<Point>> contours;
			vector<vector<Point>> contours2;
			vector<vector<Point>> contours3;
			vector<Vec4i> hierarchy;
			vector<Vec4i> hierarchy2;
			findContours(plus, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());
			findContours(blue, contours2, hierarchy2, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());
			findContours(white, contours3, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());

			vector<RotatedRect> white_rects;
			vector<RotatedRect> plus_rects;
			vector<RotatedRect> blue_rects;
			get_contours_rect(blue_rects, contours2, 0);
			get_contours_rect(white_rects, contours3, 1);
			get_contours_rect(plus_rects, contours, 0);

			vector<RotatedRect> final_plus_rects;
			vector<RotatedRect> final_white_rects;

			rects2_in_rects1(plus_rects, blue_rects, final_plus_rects, 1);

			rects2_in_rects1(final_plus_rects, white_rects, final_white_rects, 2);

			vector<Proposal> pro_rects;
			for (int i = 0;i < final_white_rects.size();i++)
			{
				//绘制轮廓的最小外结矩形
				RotatedRect final_white_rect = final_white_rects[i];
				array<Point2f, 4> P2;
				final_white_rect.points(P2.data());
				double min_rectangle_area = contourArea(P2);
				if (min_rectangle_area > 10) {
					double h1 = getDistance(P2[0], P2[1]);
					double h2 = getDistance(P2[1], P2[2]);
					double h, w, dx, dy, maxy, miny;
					if (h1 > h2) {
						h = h1;
						w = h2;
						dx = P2[0].x - P2[1].x;
						dy = P2[0].y - P2[1].y;
					}
					else {
						h = h2;
						w = h1;
						dx = P2[1].x - P2[2].x;
						dy = P2[1].y - P2[2].y;
					}
					double x = (P2[0].x + P2[2].x) / 2;
					double y = (P2[0].y + P2[2].y) / 2;

					if (fabs(P2[0].y - P2[2].y) > fabs(P2[1].y - P2[3].y)) {
						maxy = max(P2[0].y, P2[2].y);
						miny = min(P2[0].y, P2[2].y);
					}
					else {
						maxy = max(P2[1].y, P2[3].y);
						miny = min(P2[1].y, P2[3].y);
					}
					struct Proposal pro_rect;
					pro_rect.h = h;
					pro_rect.w = w;
					//cout << "高：" << h << "," << "宽：" << w << endl;
					pro_rect.x = x;
					pro_rect.y = y;
					pro_rect.dx = dx;
					pro_rect.dy = dy;
					pro_rect.maxy = maxy;
					pro_rect.miny = miny;
					pro_rect.rect = final_white_rect;
					pro_rect.area = min_rectangle_area;
					pro_rect.angle = -final_white_rect.angle < 80 ? -final_white_rect.angle : 90 + final_white_rect.angle;
					pro_rects.push_back(pro_rect);
				}
			}

			for (int i = 0;i < final_white_rects.size();i++) {
				RotatedRect rrect = final_white_rects[i];
				// matrices we'll use
				Mat M, rotated, cropped;
				// get angle and size from the bounding box
				float angle = rrect.angle;
				Size rect_size = rrect.size;
				// thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
				if (rrect.angle < -45.) {
					angle += 90.0;
					swap(rect_size.width, rect_size.height);
				}
				// get the rotation matrix
				M = getRotationMatrix2D(rrect.center, angle, 1.0);
				// perform the affine transformation
				warpAffine(roi, rotated, M, roi.size(), INTER_CUBIC);
				// crop the resulting image
				getRectSubPix(rotated, rect_size, rrect.center, cropped);
				imshow("cropped" + to_string(i), cropped);
			}

			Point2f corners0s[10][4];
			Proposal pro_rects2[10][2];
			int pro_c = 0;
			vector<double> cosine0s;
			for (int i = 0;i < pro_rects.size();i++) {
				for (int j = i + 1;j < pro_rects.size();j++) {
					if (pro_rects[i].area + pro_rects[j].area < 50) continue;
					double ratio = (2 * getDistance(Point(pro_rects[j].x, pro_rects[j].y), Point(pro_rects[i].x, pro_rects[i].y))) / (pro_rects[j].h + pro_rects[i].h);
					int precisionVal = 2;
					string trimmedString = std::to_string(ratio).substr(0, std::to_string(ratio).find(".") + precisionVal + 1);

					if ((ratio > 1.8 && ratio < 3) || (pro_rects[i].w >12 && pro_rects[j].w > 12 && ratio >= 2.7 && ratio < 3.5 && pro_rects[i].area > 50 && pro_rects[j].area > 50 && (fabs(pro_rects[i].angle - pro_rects[j].angle) < 5))) {
						Proposal pi = pro_rects[i];
						Proposal pj = pro_rects[j];

						double dx2 = pi.x - pj.x;
						double dy2 = pi.y - pj.y;
						double cosine1 = fabs(dx2 *pi.dx + dy2 * pi.dy) / sqrt(dx2*dx2 + dy2 * dy2) / sqrt(pi.dx*pi.dx + pi.dy*pi.dy);
						double cosine2 = fabs(dx2 *pj.dx + dy2 * pj.dy) / sqrt(dx2*dx2 + dy2 * dy2) / sqrt(pj.dx*pj.dx + pj.dy*pj.dy);
						if (cosine1 > 0.7 || cosine2 > 0.7)
							continue;
						if (pi.w > 2.5*pj.w && pj.area<50 || pj.w > 2.5*pi.w && pi.area<50)
							continue;
						if ((pi.area < 50 && pj.area>100 && pj.area / pi.area > 3) || (pj.area < 50 && pi.area>100 && pi.area / pj.area > 3))
							continue;
						double cosine0 = fabs(pi.dx *pj.dx + pi.dy * pj.dy) / sqrt(pi.dx*pi.dx + pi.dy * pi.dy) / sqrt(pj.dx*pj.dx + pj.dy*pj.dy);

						cosine0s.push_back(acos(cosine0) * 180 / 3.14159);
						Point2f cornersi[4];
						pi.rect.points(cornersi);
						Point2f cornersj[4];
						pj.rect.points(cornersj);
						
						
						Point max_x = Point(0, 0);
						Point max_y = Point(0, 0);
						Point min_x = Point(10000, 10000);
						Point min_y = Point(10000, 10000);
						Point corner0[8];
						for (int k = 0;k < 4;k++) {
							corner0[2 * k] = cornersi[k];
							corner0[2 * k + 1] = cornersj[k];
						}
						Point max_y1 = Point(0, 0);
						Point max_y2 = Point(0, 0);
						Point min_y1 = Point(10000, 10000);
						Point min_y2 = Point(10000, 10000);
						for (int k = 0;k < 4;k++) {
							if (cornersi[k].y > max_y1.y)
								max_y1 = cornersi[k];
							if (cornersj[k].y > max_y2.y)
								max_y2 = cornersj[k];
							if (cornersi[k].y < min_y1.y)
								min_y1 = cornersi[k];
							if (cornersj[k].y < min_y2.y)
								min_y2 = cornersj[k];
						}
						//cout << i << "  " << j << "  " << ratio << "  " << pro_c << endl;
						double delta_y = min(fabs(max_y1.y - max_y2.y), fabs(min_y1.y - min_y2.y));
						if (max_y1.y < min_y2.y || max_y2.y < min_y1.y || max_y1.y - min_y1.y>3 * (max_y2.y - min_y2.y) || max_y2.y - min_y2.y>3 * (max_y1.y - min_y1.y))
							continue;
						double sum_x=0, sum_y=0;
						for (int jj = 0;jj < 4;jj++) {
							line(roi, cornersi[jj], cornersi[(jj + 1) % 4], Scalar(0, 0, 255), 2);
							line(roi, cornersj[jj], cornersj[(jj + 1) % 4], Scalar(0, 0, 255), 2);
							sum_x += cornersi[jj].x + cornersj[jj].x;
							sum_y += cornersi[jj].y + cornersj[jj].y;
						}
						cout << "x=" << sum_x/8 << " y=" << sum_y/8 << endl;
						/*

						for (int k = 0;k < 8;k++) {
							if (corner0[k].x > max_x.x)
								max_x = corner0[k];
							if (corner0[k].y > max_y.y)
								max_y = corner0[k];
							if (corner0[k].x < min_x.x)
								min_x = corner0[k];
							if (corner0[k].y < min_y.y)
								min_y = corner0[k];
						}
						Point2f corners0[4] = { max_x,max_y,min_x ,min_y };
						memcpy(corners0s[pro_c], corners0, sizeof(corners0s[pro_c]));
						Proposal t[2] = { pro_rects[i], pro_rects[j] };
						memcpy(pro_rects2[pro_c], t, sizeof(pro_rects2[pro_c]));
						//setLabel(src3, trimmedString, pro_rects[i], pro_rects[j], i, j);
						pro_c++;
						//cout << i << "  " << j << "  " << ratio <<"  "<<pro_c<< endl;
						//cout << max_x << "  " << max_y << "  " << min_x << "  " << min_y << endl;
						//goto stop;*/
					}
					//cout << ratio << endl;
					//setLabel(src3, trimmedString, pro_rects[i], pro_rects[j], i, j);
				}
			}
			//stop:
			//cout << pro_c << "----------45444444" << endl;
			/*
			Point2f corners0[4];
			int p = 0;
			double min_delta_angle = 45;
			Point2f t_corners0s[4];
			if (pro_c == 3) {
				//cout << "三个" << endl;
				for (int k = 0;k < pro_c;k++) {
					double tx1 = pro_rects2[k][0].x + pro_rects2[k][1].x;
					for (int kk = k + 1;kk < pro_c;kk++) {
						double tx2 = pro_rects2[kk][0].x + pro_rects2[kk][1].x;
						if (tx2 > tx1) {
							memcpy(t_corners0s, corners0s[kk], sizeof(t_corners0s));
							memcpy(corners0s[kk], corners0s[k], sizeof(corners0s[kk]));
							memcpy(corners0s[k], t_corners0s, sizeof(corners0s[k]));
						}
					}
				}
				p = 1;
			}
			else if (pro_c > 1) {
				for (int k = 0;k < pro_c;k++) {
					//cout << pro_rects2[k][0].angle << "  " << pro_rects2[k][1].angle << " angel" << endl;
					double delta_angle = fabs(pro_rects2[k][0].angle - pro_rects2[k][1].angle);
					if (delta_angle < min_delta_angle) {
						min_delta_angle = delta_angle;
						p = k;
					}
				}
			}
			//for (int p = 0;p < pro_c;p++) {
			/*if (pro_c > 1) {
			if (cosine0s[p] > 8)
			continue;
			else {
			memcpy(corners0, corners0s[p], sizeof(corners0));
			for (int jj = 0;jj < 4;jj++) {
				line(roi, corners0[jj], corners0[(jj + 1) % 4], Scalar(0, 0, 255), 2);
			}
			/*
			double delta_angle = fabs(pro_rects2[p][0].rect.angle - pro_rects2[p][1].rect.angle);
			//cout << min_delta_angle << "===================66666666666**************" << p << "  " << pro_c << endl;

			Point2f *lastItemPointer2 = (corners0 + sizeof corners0 / sizeof corners0[0]);
			vector<Point2f> contour3(corners0, lastItemPointer2);
			Rect rr = boundingRect(contour3);
			rectangle(roi, rr, Scalar(0, 255, 0), 1, 8, 0);*/

			/*}
			else {
			memcpy(corners0, corners0s[p], sizeof(corners0));
			double delta_angle = fabs(pro_rects2[p][0].rect.angle - pro_rects2[p][1].rect.angle);
			//cout << min_delta_angle << "===================66666666666**************" << p << "  " << pro_c << endl;

			Point2f *lastItemPointer2 = (corners0 + sizeof corners0 / sizeof corners0[0]);
			vector<Point2f> contour3(corners0, lastItemPointer2);
			Rect rr = boundingRect(contour3);
			rectangle(src, rr, Scalar(0, 255, 0), 1, 8, 0);
			}*/

			
			show_RotatedRects(final_white_rects, roi2, "final_white_rects");
			show_RotatedRects(final_plus_rects, roi3, "final_plus_rects");
			show_RotatedRects(blue_rects, roi4, "blue_rects");
			show_RotatedRects(plus_rects, roi5, "plus_rects");
			show_RotatedRects(white_rects, roi6, "white_rects");
			imshow("blue", blue);
			imshow("white", white);
			imshow("plus", plus);
			//imshow("result", roi);

			//imshow("src", src);
			//cout << endl << endl << endl << ii << endl;
			if (waitKey(1) == 27)
				return 0;
			//waitKey(0);
			//imwrite("./task2/0new_" + to_string(ii) + ".jpg", src);

			//imshow("blue", blue);

			namedWindow("ROI", 0);
			resizeWindow("ROI", 640, 480);
			imshow("ROI", roi);

			namedWindow("src", 0);
			resizeWindow("src", 640, 480);
			imshow("src", src);

			waitKey(0);
			middletime2 = clock();
			cout << "middletime=  " << middletime2 - middletime1 << endl;
		}
	}
	else
	{
		cout << "no " + jpgsname + " file" << endl;
	}
	endtime = clock();
	cout << "finaltime=  " << endtime - begintime << endl;
	system("pause");
	return 0;
}


