#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>
#include "opencv2/objdetect.hpp"
#include "ArmorPlate.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <queue>
//#define DARK
#define RECORD					//录像标识

using namespace std;

double getDistance (CvPoint pointO,CvPoint pointA )
{
    double distance;
    distance = powf((pointO.x - pointA.x),2) + powf((pointO.y - pointA.y),2);
    distance = sqrtf(distance);
    return distance;
}
class Frame
{
public:
    int Frame_count;
    bool is_ok;
    cv::RotatedRect max_rect;
    cv::Mat img;
    cv::Point2f left_up_point;
    std::vector<cv::RotatedRect> result;
    cv::Point2f short_point;
    cv::Point2f long_point;
    cv::Point2f duijiao_point;
    Frame(int x)
    {
        Frame_count=x;
    }
    Frame()
    {
        //cout<<1<<endl;
        return;
    }
    void find_max()
    {
        float max=0;
        if (result.size()==0)
            return;
        for(int i=0;i<result.size();i++)
        {
            if (result[i].size.area()>max)
            {
                max=result[i].size.area();
                max_rect=result[i];
            }
        }
        cv::Point2f P[4];
        max_rect.points(P);
        float min=10000;
        for(int i=0;i<4;i++)
        {
            double dist=getDistance(P[i],cv::Point(0,0));
            if (dist<min)
            {
                min=dist;
                left_up_point=P[i];
            }
        }

        double rect_short,rect_long;
        double rect_height=getDistance(P[0],P[1]);
        double rect_width=getDistance(P[1],P[2]);
        rect_short=rect_height>rect_width?rect_width:rect_height;
        rect_long=rect_height>rect_width?rect_height:rect_width;
        for(int i=0;i<4;i++)
        {
            if (abs(getDistance(P[i],left_up_point)-rect_short)<3)
            {
                //cout<<"find short"<<endl;
                short_point=P[i];
            }
            else if (abs(getDistance(P[i],left_up_point)-rect_long)<3)
            {
                //cout<<"find long"<<endl;
                long_point=P[i];
                is_ok=true;
            }
            else if (abs(getDistance(P[i],left_up_point)==0))
                continue;
            else
                duijiao_point=P[i];
        }

        cout<<"x="<<left_up_point.x<<"y="<<left_up_point.y<<endl;
    }

};

Frame frames[1000];
double sumTime = 0, startTime = 0, endTime = 0;
cv::Mat frame,proced;
ArmorPlate amp;

int main(){
    std::vector<cv::RotatedRect> result;
    cv::VideoCapture cap("../test.avi");
    double totalFrameNumber = cap.get(cv::CAP_PROP_FRAME_COUNT);

    for (int w = 0; w < totalFrameNumber; w++)
    {
        startTime = cv::getTickCount();
        std::vector<cv::RotatedRect> result;
        cap.read(frame);
        //draw armor
        Frame my_frame(w);
        amp.lowExposure(frame, proced);
        amp.findPossible(proced);
        amp.drawArmorPlate(frame);
        amp.getArmorRect(my_frame.result);
        //draw end
        frames[w]=my_frame;
        my_frame.find_max();
        cv::imshow("frame", frame);
        cv::waitKey(0);
        std::vector<cv::Point2f> points1,points2;

        if (!my_frame.is_ok)
        {
            cout<<"no armor"<<endl;
            continue;
        }
        points1.push_back(my_frame.left_up_point);
        points1.push_back(my_frame.short_point);
        points1.push_back(my_frame.long_point);
        points1.push_back(my_frame.duijiao_point);
        points2.push_back(cv::Point2f(0,0));
        points2.push_back(cv::Point2f(0,5.6));
        points2.push_back(cv::Point2f(13.2,0));
        points2.push_back(cv::Point2f(13.2,5.6));
        cv::Mat homography_matrix = cv::findHomography ( points1, points2, cv::RANSAC, 3 );
        //cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;
        //cout<<homography_matrix.rows<<endl;
        if (homography_matrix.rows==0)
        {
            cout<<"no homography matrix"<<endl;
            continue;
        }
        //-- 分解单应矩阵
        std::vector<cv::Mat> r, t, n;
        double intrinsic[9] = { 1770.71,       0,          738.37,
                                0,       1768.77,   610.32,
                                0,              0,          1}; //Xtion内参
        cv::Mat K(3, 3, CV_64FC1, intrinsic); //相机内参
        cv::decomposeHomographyMat(homography_matrix, K, r, t, n);
        //cout << "========Homography========" << endl;

        std::vector<cv::Mat>  point_answer;
        for(int i=0; i<r.size(); ++i) {
            //cout << "======== " << i << " ========" << endl;
            //cout << "rotation" << i << " = " << endl;
            //cout << r[i] << endl;
            //cout << "translation" << i << " = " << endl;
            //cout << t[i] << endl;

            for (int j=0;j<4;j++)
            {
                double frame_point[3]={points2[j].x,points2[j].y,0};
                cv::Mat Point_Mat(3,1,CV_64FC1,frame_point);
                //cout<<"the point is "<<Point_Mat<<endl;
                cv::Mat answer_temp_Mat=r[i]*Point_Mat+t[i];
                //cout<<"answer is"<<answer_temp_Mat<<endl;
                point_answer.push_back(answer_temp_Mat);
                //cout<<"point=="<<t[i]<<endl;
            }
        }
        endTime = cv::getTickCount();
        cout<<endTime-startTime<<endl;

    }

    return 0;
}
