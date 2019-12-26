#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include "cam_map.hpp"

#define SHAPE_LINE 1
#define SHAPE_ERASER 2

int g_style;
CvRect rect;

Cv_Point g_Start_Point;
Cv_Point g_End_Point;
Cv_Point p_Start;        //记录鼠标位置左上方点
Cv_Point p_End;          //记录鼠标位置右下方点
bool drawing = false;   //是否处于画图状态
bool erasering = false; //是否处于擦除状态
void callback(int event, int x, int y, int flags, void *param);
void DrawLine(IplImage *img);

//简易绘图工具，实现了画线和橡皮擦功能，绘图功能还有待往里添加
int main(int argc, char **argv)
{
    Map map;
    Aera aera(0);
    aera.setT(1, 0, 0, 1, 0, 0);
    aera.points.push_back(_Point(11, 11));
    aera.points.push_back(_Point(22, 11));
    aera.points.push_back(_Point(33, 22));
    aera.points.push_back(_Point(44, 33));
    map.areas.push_back(aera);
    aera.id = 1;
    aera.setT(0.8, 0.2, -0.2, 0.8, 0, 0);
    map.areas.push_back(aera);
    string fileName = "../map.xml";
    CreateXmlFile(fileName, map);

    IplImage *img = cvCreateImage(cvSize(512, 512), IPL_DEPTH_8U, 3);
    cvSet(img, cvScalar(255, 255, 255));
    IplImage *temp = cvCloneImage(img);
    cvCopy(img, temp);
    cvNamedWindow("简易绘图工具");
    cvSetMouseCallback("简易绘图工具", callback, img);
    printf("请键入要选择执行的操作:‘l’——”画线“,‘e’——”橡皮“\n");
    char select = 'l';
    while (1)
    {
        cvCopy(img, temp);
        //始终在原图image上画结果(和鼠标事件绑定)，先将image复制给temp,然后在临时图temp画出画图的过程(在main函数中画过程图形),然后用temp来显示图像
        switch (select)
        {
        case 'l':
            g_style = SHAPE_LINE;
            break;
        case 'e':
            g_style = SHAPE_ERASER;
            break;
        case 27:
            return 0;
        }
        if (g_style == SHAPE_LINE && drawing)
            //画鼠标在按住移动过程中画线
            cvDrawLine(temp, g_Start_Point, g_End_Point, cvScalar(0, 0, 0));
        if (g_style == SHAPE_ERASER)
        {
            cvRectangle(temp, p_Start, p_End, cvScalar(0, 0, 0));
            //画出橡皮矩形边框
        }
        cvShowImage("简易绘图工具", temp);
        select = cvWaitKey(30);
    }
    cvReleaseImage(&img);
    cvReleaseImage(&temp);
    cvDestroyWindow("简易绘图工具");
    return 0;
}

void callback(int event, int x, int y, int flags, void *param)
{
    IplImage *img = (IplImage *)param;
    switch (event)
    {
    case CV_EVENT_LBUTTONDOWN:
    {
        if (SHAPE_LINE == g_style)
        {
            drawing = true;
            g_Start_Point = cv_Point(x, y);
            g_End_Point = g_Start_Point;
            //此处将终点坐标设为同起始点避免记住前一条直线的终点坐标
        }
        if (SHAPE_ERASER == g_style)
        {
            erasering = true;
            //注意边界问题要适当修改ROI有效区域
            rect.x = x - 20;
            rect.y = y - 20;
            //注意坐标的计算,一般原点在窗口的左上角(这和操作系统等因素有关,IplImage结构中有个origin属性可以设置图像的原点)
            rect.width = 40;
            rect.height = 40;
            p_Start.x = x - 20;
            p_Start.y = y - 20;
            p_End.x = x + 20;
            p_End.y = y + 20;
            if (x > -20 && x < 532 && y > -20 && y < 532)
            {
                cvSetImageROI(img, rect);
                cvSet(img, cvScalar(255, 255, 255));
                cvResetImageROI(img);
            }
        }
    }
    break;
    case CV_EVENT_MOUSEMOVE:
    {
        p_Start.x = x - 20;
        p_Start.y = y - 20;
        //随时定位鼠标位置画橡皮矩形
        p_End.x = x + 20;
        p_End.y = y + 20;
        if (SHAPE_LINE == g_style)
        {
            if (drawing)
            {
                g_End_Point = cv_Point(x, y);
            }
        }
        if (SHAPE_ERASER == g_style)
        {
            rect.x = x - 20;
            rect.y = y - 20;
            rect.width = 40;
            rect.height = 40;
            if (erasering)
            {
                if (x > -20 && x < 532 && y > -20 && y < 532)
                {
                    cvSetImageROI(img, rect);
                    cvSet(img, cvScalar(255, 255, 255));
                    cvResetImageROI(img);
                }
            }
        }
    }
    break;
    case CV_EVENT_LBUTTONUP:
    {
        if (SHAPE_LINE == g_style)
        {
            drawing = false;
            cvDrawLine(img, g_Start_Point, g_End_Point, cvScalar(255, 0, 0));
            //(255,0,0)此处画出的是蓝色,即BGR
        }
        if (SHAPE_ERASER == g_style)
        {
            erasering = false;
        }
    }
    }
}
