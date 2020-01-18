#include "frame.h"

float get_distance(cv::Point p1, cv::Point p2){
	double distance;
	distance = powf((p1.x - p2.x), 2) + powf((p1.y - p2.y), 2);
	distance = sqrtf(distance);
	return distance;
}

LLC_Frame::LLC_Frame(){
    return;
}

LLC_Frame::LLC_Frame(int x)
    {
        Frame_count=x;
        is_ok = false;
    }

LLC_Frame::~LLC_Frame(){
    ;
}

void LLC_Frame::find_max(){
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
        double dist=get_distance(P[i],cv::Point(0,0));
        if (dist<min)
        {
            min=dist;
            left_up_point=P[i];
        }
    }

    double rect_short,rect_long;
    double rect_height=get_distance(P[0],P[1]);
    double rect_width=get_distance(P[1],P[2]);
    rect_short=rect_height>rect_width?rect_width:rect_height;
    rect_long=rect_height>rect_width?rect_height:rect_width;
    for(int i=0;i<4;i++)
    {
        if (abs(get_distance(P[i],left_up_point)-rect_short)<3)
        {
            //cout<<"find short"<<endl;
            short_point=P[i];
        }
        else if (abs(get_distance(P[i],left_up_point)-rect_long)<3)
        {
            //cout<<"find long"<<endl;
            long_point=P[i];
            is_ok=true;
        }
        else if (abs(get_distance(P[i],left_up_point)==0))
            continue;
        else
            duijiao_point=P[i];
    }

    //cout<<"x="<<left_up_point.x<<"y="<<left_up_point.y<<endl;
}
