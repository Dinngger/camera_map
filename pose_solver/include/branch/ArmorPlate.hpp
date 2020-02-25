/**============新的MatchLight(明暗交替)============
 * 作者：hqy
 * 创建时间：2020.2.9 00:27
 * 主要思路：设置几个判断条件：（2020.1.18日写）
 * 9/2/2020 修改：
 *      drawArmorPlate需要通过数字判断其是否valid
 * 25/2/2020 修改：
 *      将Armorplate加入雷达组算法
 */
#ifndef _ARMOR_PLATE_HPP
#define _ARMOR_PLATE_HPP

#include <vector>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "AimDeps.cc"
#define ANGLE_THRESH        19.0             //装甲板灯条角度差阈值(12.0)
//#define ARMORPLATE_DEBUG 
#ifdef ARMORPLATE_DEBUG
    #define print printf
#else
    #define print(...)
#endif

#define TEST_ARMOR_MODE_ACCURATE            //如果需要大阈值，则注释此行
#ifdef TEST_ARMOR_MODE_ACCURATE
    #define OPS_RATIO_HEIGHT 10.24          //对边宽比例
    #define OPS_RATIO_WIDTH 1.21            //对边长比例
    #define NEAR_RATIO 8.0                  //邻边装甲板比例
#else
    #define OPS_RATIO_HEIGHT 16.0           //对边宽比例
    #define OPS_RATIO_WIDTH 4.0             //对边长比例
    #define NEAR_RATIO 25.0                 //邻边装甲板比例
#endif

class ArmorPlate{
public:
    ArmorPlate();
    ~ArmorPlate();
public:
    /**
     * @brief 根据与匹配情况匹配所有的灯条
     * @param matches cv::Point2f 与匹配对，两个值是两个灯条在lights中的索引
     * @param lights 灯条容器，由LightMatch类传入
     * @param tar_list 入参/输出，由更高层的AimDistance类传入
     */
    void matchAll(
        std::vector<cv::Point> matches,
        std::vector<aim_deps::Light> lights, 
        std::vector<aim_deps::Armor> &tar_list);
    /**
     * @brief 绘制装甲板于图像
     * @param src 输入/输出,用于绘制的图像
     * @param tar_list 装甲板列表
     * @param optimal 最优装甲板的位置(使用绿色绘制)
     */
    void drawArmorPlates(cv::Mat &src, 
        const std::vector<aim_deps::Armor> tar_list, const int optimal = 0);
public:
    std::vector<cv::Mat> rMats, tMats;                                  //临时的加入
private:
    bool isRatioValid();                                                //中点连线（平方）的比是否合适
    bool isEdgesValid();                                                //两对边(平方)的比是否合适

    /**
     * @brief 对两个灯条的RotatedRect进行匹配，直接在tar_list中emplace_back符合条件的装甲板
     * @param l1 灯条1
     * @param l2 灯条2
    */
    bool isMatch(aim_deps::Light l1, aim_deps::Light l2);
    void getMidPoints(cv::RotatedRect rect,
         cv::Point2f &p1, cv::Point2f &p2);                             //获取一个灯条的短边两中点
    bool getArmorPlate(cv::RotatedRect r1, cv::RotatedRect r2);         //灯条匹配装甲板  

    inline static bool isAngleMatch(const cv::RotatedRect r1, const cv::RotatedRect r2);   //灯条角度差合适吗    
    inline static float angleDiff(const cv::RotatedRect r1, const cv::RotatedRect r2);//求两灯条的角度差
    inline static float getPointDist(const cv::Point2f p1, const cv::Point2f p2);          //返回两点距离的平方
private:
    cv::Point2f points[4];                                              //装甲板点列的临时容器
};

ArmorPlate::ArmorPlate(){
    for(int i=0;i<4;++i) points[i]=aim_deps::NULLPOINT2f;
}

ArmorPlate::~ArmorPlate(){;}

void ArmorPlate::matchAll(
    std::vector<cv::Point> matches,
    std::vector<aim_deps::Light> lights,
    std::vector<aim_deps::Armor> &tar_list
)
{
    tar_list.clear();
    for( int i = 0 ; i<matches.size() ; ++i){
        if(isMatch(lights[matches[i].x],lights[matches[i].y])){
            tar_list.emplace_back(aim_deps::Armor(points, 0, lights[matches[i].x], lights[matches[i].y]));
            print("Matched:(%d), with matches(%d, %d).\n", i, matches[i].x, matches[i].y);
        }
    }
    //=============debug:计算出最后有多少个valid的装甲板==================
    #ifdef DEBUG
    int _cnt = 0;
    for(int i=0;i<tar_list.size();++i){
        if(tar_list[i].valid) ++_cnt;
    }
    #endif
    //print("Target list size(%d), valid size(%d).\n", tar_list.size(), _cnt);
}

bool ArmorPlate::isMatch(aim_deps::Light l1, aim_deps::Light l2)
{
    if(!isAngleMatch(l1.box, l2.box)){
        print("Angle dismatch.(%d, %d).\n", l1.index, l2.index);
        return false;
    }
    bool judge = true;                                  //灯条角度过大（与x轴成的夹角小）时退出
    if(l1.box.center.x < l2.box.center.x)               //r1灯条在左侧
        judge = getArmorPlate(l1.box, l2.box);
    else
        judge = getArmorPlate(l2.box, l1.box);                  //始终保持第一个入参是x轴坐标小的灯条
    if(!judge) return false;
    if(isRatioValid() && isEdgesValid()){
        print("Pushed in.(%d, %d).\n", l1.index, l2.index);
        return true;
    }
    else{
        print("Ratio dismatch.(%d, %d).\n", l1.index, l2.index);
    }
    return false;
}

void ArmorPlate::drawArmorPlates(cv::Mat &src, 
    const std::vector<aim_deps::Armor> tar_list, const int optimal){
	char str[2];
    cv::line(src, cv::Point(720, 0), cv::Point(720, 1080), cv::Scalar(255, 0, 0));
	cv::line(src, cv::Point(0, 540), cv::Point(1440, 540), cv::Scalar(255, 0, 0));
    for (int i = 0; i< tar_list.size(); ++i) {
        ///关于valid的意义：可能需要删除，默认valid(装甲板有效)
        if(tar_list[i].armor_number != -1 && tar_list[i].valid){   //有意义的数字
            if(i != optimal){       //非最佳装甲板使用黄色绘制
                for (int j = 0; j < 4; ++j){
                    cv::line(src, tar_list[i].vertex[j], 
                    tar_list[i].vertex[(j + 1) % 4], cv::Scalar(0, 255, 255), 2);   
                }
            }
            else{                   //最佳装甲板使用绿色绘制
                for (int j = 0; j < 4; ++j){
                    cv::line(src, tar_list[i].vertex[j],
                    tar_list[i].vertex[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
                }
            }
            ///snprintf(str, 2, "%d", j);      //最佳装甲板位置x
	        ///cv::putText(src, str, tar_list[i].vertex[j]+cv::Point2f(2, 2),
	        ///    cv::FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(0, 100, 255));
            snprintf(str, 2, "%d", tar_list[i].armor_number);
            cv::putText(src, str, tar_list[i].center+cv::Point2f(25, 10),
                cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255));
            cv::circle(src, tar_list[i].center, 3, cv::Scalar(0, 0, 255), -1);
        }
    }
}

void ArmorPlate::getMidPoints(cv::RotatedRect rect, cv::Point2f &p1, cv::Point2f &p2){
    cv::Point2f tmp_p1, tmp_p2, corners[4];                                     //找出角点
    rect.points(corners);
    float d1 = getPointDist(corners[0], corners[1]);            //0/1点距离的平方
	float d2 = getPointDist(corners[1], corners[2]);            //1/2点距离的平方
	int i0 = d1 > d2? 1 : 0;								    //长所在边第一个顶点的位置
    tmp_p1 = (corners[i0] + corners[i0 + 1]) / 2;			    //获得旋转矩形两条短边上的中点
	tmp_p2 = (corners[i0 + 2] + corners[(i0 + 3) % 4]) / 2;
    if(tmp_p1.y > tmp_p2.y){                                    //保证输出点的顺序
        p2 = tmp_p1;    
        p1 = tmp_p2;
    }
    else{                                                       //必须是p1是处于上方的点，p2处于下方（y轴更大）
        p1 = tmp_p1;
        p2 = tmp_p2;
    }
}

bool ArmorPlate::getArmorPlate(cv::RotatedRect r1, cv::RotatedRect r2){
    getMidPoints(r1, points[0], points[1]);             
    getMidPoints(r2, points[3], points[2]);
    cv::Point2f diff = points[0]-points[1];
    return diff.x/diff.y < 1.5;
}

bool ArmorPlate::isRatioValid(){                    //对边中点连线的长度平方比值是否合适
    float len1 = getPointDist((points[0]+points[1])/2, (points[2]+points[3])/2),
         len2 = getPointDist((points[0]+points[3])/2, (points[1]+points[2])/2);
    return (len1/len2 < NEAR_RATIO && len1/len2 > 1.0/NEAR_RATIO);
}

//从最左上角开始的点，逆时针方向标号是0,1,2,3
bool ArmorPlate::isEdgesValid(){  //对边长度平方比值是否合适
    float edges[4];
    for(int i = 0; i<4;++i){
        edges[i]=getPointDist(points[i], points[(i+1)%4]);
    }
    bool judge1 = (edges[0]/edges[2]<OPS_RATIO_HEIGHT &&
        edges[0]/edges[2] > 1.0 / OPS_RATIO_HEIGHT),     //宽对边比值范围大
        judge2 = (edges[1]/edges[3]<OPS_RATIO_WIDTH &&
        edges[1]/edges[3] > 1.0 / OPS_RATIO_WIDTH);   //长对边比值范围小
    return judge1 && judge2;
}

bool ArmorPlate::isAngleMatch(const cv::RotatedRect r1, const cv::RotatedRect r2){
    float diff = angleDiff(r1, r2);
    if(diff<ANGLE_THRESH){
        return true;
    }
    print("Angle difference is %f, not matched.\n", diff);
    return false;
}

float ArmorPlate::angleDiff(const cv::RotatedRect r1, const cv::RotatedRect r2){
    float ang1 = r1.size.width > r1.size.height ? r1.angle : -r1.angle - 90;
	float ang2 = r2.size.width > r2.size.height ? r2.angle : -r2.angle - 90;
    return abs(ang1-ang2);
}

float ArmorPlate::getPointDist(const cv::Point2f p1, const cv::Point2f p2){
    return ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

#endif     //_ARMOR_PLATE_HPP


