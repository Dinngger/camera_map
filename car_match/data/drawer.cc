#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
struct lightBarP
{
    cv::Point2f Center;
    cv::Point2f P; // the vector from center to the upper point.
    int index;
    lightBarP(cv::Point2f _center, cv::Point2f _p, int ind)
    {
        Center = _center;
        P = _p;
        index = ind;
    }
};
void drawLightBar(lightBarP _l, cv::Mat &src)
{
    cv::Point2f Up;
    cv::Point2f Down;
    char str[2];
    Up.x = _l.Center.x + _l.P.x;
    Up.y = _l.Center.y + _l.P.y;
    Down.x = _l.Center.x - _l.P.x;
    Down.y = _l.Center.y - _l.P.y;
    cv::line(src, Up, Down, cv::Scalar(255, 0, 0), 2);        //red
    std::snprintf(str, 4, "%d", _l.index);
    cv::putText(src, str, _l.Center + cv::Point2f(6, 6), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255));
}
void drawLight_car1(lightBarP _l, cv::Mat &src)
{
    cv::Point2f Up;
    cv::Point2f Down;
    char str[2];
    Up.x = _l.Center.x + _l.P.x;
    Up.y = _l.Center.y + _l.P.y;
    Down.x = _l.Center.x - _l.P.x;
    Down.y = _l.Center.y - _l.P.y;
    cv::line(src, Up, Down, cv::Scalar(0, 255, 0), 3);      //green
}
void drawLight_car2(lightBarP _l, cv::Mat &src)
{
    cv::Point2f Up;
    cv::Point2f Down;
    char str[2];
    Up.x = _l.Center.x + _l.P.x;
    Up.y = _l.Center.y + _l.P.y;
    Down.x = _l.Center.x - _l.P.x;
    Down.y = _l.Center.y - _l.P.y;
    cv::line(src, Up, Down, cv::Scalar(255,255, 255), 3);        //blue
}
int main()
{
    int count = 0;
    std::ifstream inFile;
    inFile.open("/home/zhao/transformer.csv", std::ios::in);
    std::string line;
    while (getline(inFile, line))
    {
        cv::Mat frame = cv::Mat(1600, 1200, CV_8UC3,cv::Scalar(0,0,0));
        std::istringstream sin(line);
        std::vector<std::string> fields;
        std::string field;
        std::vector<lightBarP> lightvec;
        while (getline(sin, field, ','))
        {
            fields.push_back(field);
        }
        for (int i = 0; i < 52; i+=4)
        {
            if(fields[i].c_str()-'0'==0)
                break;
            lightBarP light(cv::Point2f(atof(fields[i].c_str()), atof(fields[i+1].c_str())), cv::Point2f(atof(fields[i+2].c_str()), atof(fields[i+3].c_str())), i/4 );
            drawLightBar(light, frame);
            lightvec.push_back(light);
        }
        for(int i=0;i<13;i++)
        {
            if(atof(fields[i+65].c_str())!=0)
            {
                if(atof(fields[i+65].c_str())==1)
                {
                    drawLight_car1(lightvec[i],frame);
                }
                if(atof(fields[i+65].c_str())==2)
                {
                    drawLight_car2(lightvec[i],frame);
                }
                
            }
        }
        cv::imshow("dis", frame);
        if(cv::waitKey(0)==27)
            break;
        frame.release();
        fields.clear();
        field.clear();
    }
    return 0;
}