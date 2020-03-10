/**============新的MatchLight(明暗交替)============
 * 作者：hqy
 * 创建时间：2020.2.9 00:27
 * 主要思路：设置几个判断条件：（2020.1.18日写）
 * 9/2/2020 修改：
 *      drawArmorPlate需要通过数字判断其是否valid
 */
#ifndef _ARMOR_PLATE_HPP
#define _ARMOR_PLATE_HPP

#include <vector>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "../aim_deps/AimDeps.cc"
#include "../../../serial_com/include/serial_com/LOG.hpp"
//#define ARMORPLATE_DEBUG 
#ifdef ARMORPLATE_DEBUG
    #define amp_debug rmlog::LOG::printc        //彩色输出
#else
    #define amp_debug(...)
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
        const std::vector<aim_deps::Armor> tar_list, const int optimal);                                            //消息发布
private:
    bool isRatioValid();                                                //中点连线（平方）的比是否合适
    bool isEdgesValid();                                                //两对边(平方)的比是否合适

    /**
     * @brief 对两个灯条的RotatedRect进行匹配，直接在tar_list中emplace_back符合条件的装甲板
     * @param l1 灯条1
     * @param l2 灯条2
    */
    bool isMatch(aim_deps::Light l1, aim_deps::Light l2);
    bool getArmorPlate(cv::RotatedRect r1, cv::RotatedRect r2);         //灯条匹配装甲板
    void getMidPoints(cv::RotatedRect rect,
         cv::Point2f &p1, cv::Point2f &p2);                             //获取一个灯条的短边两中点
    void filter(std::vector<aim_deps::Armor> &tar_list);                //进一步过滤装甲板容器


    /**
     * @brief 判断灯条1角度是否合适
     * 此函数在points被解算出来以后才会执行，因为point[0],[1]位置表征的是左方灯条，[3],[2]则是右方灯条
     * 所以依据这两个点之间的位置来判断灯条角度，不使用RotatedRect
     */
    bool isAngleMatch();                       
    /** 获得两个点对应直线（灯条的简化表征）的角度 */
    inline static float getAngle(const cv::Point2f p1, const cv::Point2f p2);
    inline static float getPointDist(const cv::Point2f p1, const cv::Point2f p2);          //返回两点距离的平方
    inline static float getRatio(const float l);                        //计算自适应装甲板长宽比
private:
    bool _is_enemy_blue;                                                //敌人颜色
    float _average_ang;                                                 //计算的平局角度储存在这里
    cv::Point2f points[4];                                              //装甲板点列的临时容器
    aim_deps::Distance_Params params;                                   //装甲板匹配参数
};

ArmorPlate::ArmorPlate(){
    for(int i=0; i<4 ;++i) points[i] = aim_deps::NULLPOINT2f;
    _average_ang = 0.0;
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
            tar_list.back().ang_aver = abs(_average_ang);            //给新push的元素的平均灯条角度赋值
            amp_debug(rmlog::F_GREEN, "Matched:(", i, "), with matches(", matches[i].x, ", ", matches[i].y,")");
        }
    }
    filter(tar_list);                   //过滤无效装甲板
    //amp_debug("Target list size(%d), valid size(%d).\n", tar_list.size(), _cnt);
}

bool ArmorPlate::isMatch(aim_deps::Light l1, aim_deps::Light l2)
{
    bool judge = true;                                  //灯条角度过大（与x轴成的夹角小）时退出
    if(l1.box.center.x < l2.box.center.x)               //r1灯条在左侧
        judge = getArmorPlate(l1.box, l2.box);
    else
        judge = getArmorPlate(l2.box, l1.box);                  //始终保持第一个入参是x轴坐标小的灯条
    if(!judge) return false;
    if(!isAngleMatch()){
        amp_debug(rmlog::F_RED, "Angle mismatch:(", l1.index, ", ", l2.index, ")");
        return false;
    }
    if(isRatioValid() && isEdgesValid()){
        amp_debug(rmlog::F_BLUE, "Push in:(", l1.index, ", ", l2.index, ")");
        return true;
    }
    else{
        amp_debug(rmlog::F_RED, "Ratio mismatch:(", l1.index, ", ", l2.index, ")");
    }
    return false;
}

void ArmorPlate::drawArmorPlates(cv::Mat &src, 
    const std::vector<aim_deps::Armor> tar_list, const int optimal){
	char str[2];
    cv::line(src, cv::Point(720, 0), cv::Point(720, 1080), cv::Scalar(255, 0, 0));
	cv::line(src, cv::Point(0, 540), cv::Point(1440, 540), cv::Scalar(255, 0, 0));
    for (int i = 0; i< tar_list.size(); ++i) {
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

void ArmorPlate::filter(std::vector<aim_deps::Armor> &tar_list){
    for(int i = 0; i<tar_list.size(); ++i){
        if(tar_list[i].valid){
            for(int j = i+1; j<tar_list.size(); ++j){
                if(tar_list[j].valid){
                    if(tar_list[i] == tar_list[j]){                         //装甲板有共灯条冲突
                        tar_list[i].ang_aver > tar_list[j].ang_aver ?       //灯条平均角度大的被过滤掉
                            tar_list[i].valid = false :
                            tar_list[j].valid = false;
                        amp_debug(rmlog::F_BLUE, "One false armorplate is filtered.");
                    }
                }
            }
        }
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
         len2 = getPointDist((points[0]+points[3])/2, (points[1]+points[2])/2),
         ratio = len1/len2, 
         thresh = getRatio(len2);
    ///CHANGES IN HERE
    if(len2 < 25.0){                                //灯条高度平方小于25时，过小的两灯条需要进行一个判断
        if (ratio < thresh && ratio > thresh / 4){  //如果过小的两个灯条过于接近（ratio<= thresh/4）,就是错的
            return true;
        }
    }
    else{
        if (ratio < thresh){
            return true;
        }
    }
    amp_debug(rmlog::F_RED, "Ratio failed: thresh", thresh, ", ", len1/len2, ", ", len2/len1);
    amp_debug(rmlog::F_PURPLE, "Correspond to: len1, len2:", sqrt(len1), ", ", sqrt(len2));
    return false;
}

//从最左上角开始的点，逆时针方向标号是0,1,2,3
bool ArmorPlate::isEdgesValid(){  //对边长度平方比值是否合适
    float edges[4];
    for(int i = 0; i<4;++i){
        edges[i]=getPointDist(points[i], points[(i+1)%4]);
    }
    bool judge1 = (edges[0]/edges[2] < params.OPS_RATIO_HEIGHT &&
        edges[0]/edges[2] > 1.0 / params.OPS_RATIO_HEIGHT),     //宽对边比值范围大
        judge2 = (edges[1]/edges[3] < params.OPS_RATIO_WIDTH &&
        edges[1]/edges[3] > 1.0 / params.OPS_RATIO_WIDTH);   //长对边比值范围小
    if (judge1 && judge2){
        return true;
    }
    ///DEBUG
    if(!judge1) amp_debug(rmlog::F_RED, "Judge 1 failed:", edges[0]/edges[2], ", ", edges[2]/edges[0]);
    if(!judge2) amp_debug(rmlog::F_RED, "Judge 2 failed:", edges[1]/edges[3], ", ", edges[3]/edges[1]);
    return false;
}

bool ArmorPlate::isAngleMatch(){
    //输入按照装甲板上点的顺序: 从左上角开始逆时针,
    // |0        3|
    // |1        2|
    float ang1 = getAngle(points[0], points[1]),
        ang2 = getAngle(points[3], points[2]);      
    if (abs(ang1-ang2) < params.ANGLE_THRESH){
        _average_ang = (abs(ang1) + abs(ang2))/2;
        return true;
    }
    return false;
}

float ArmorPlate::getAngle(const cv::Point2f p1, const cv::Point2f p2){
    //默认是p1是处于上方的点，p2是处于下方的点
    float dy = p2.y - p1.y, dx = p2.x - p1.x, ang = 0.0;
    ang =  atan2f(dx, dy) * aim_deps::RAD2DEG;
    ///DEBUG
    amp_debug(rmlog::F_WHITE, "Angle for this light: ", ang);
    return ang;
}

float ArmorPlate::getPointDist(const cv::Point2f p1, const cv::Point2f p2){
    return ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

float ArmorPlate::getRatio(const float l){
    // 默认输入的len2是灯条长度平方的平均值
    if(l > 56.25) return aim_deps::distance_params.NEAR_RATIO_MIN;            //12.5   (56.25是7.5的平方)
    else if(l <= 14.44) return aim_deps::distance_params.NEAR_RATIO_MAX;      //17.64是3.8（像素）的平方，30.0
    float len = sqrt(l);
    return aim_deps::coeffs[0] * powf(len, 4) + aim_deps::coeffs[1] * powf(len, 3) + 
            aim_deps::coeffs[2] * len * len + aim_deps::coeffs[3] * len + aim_deps::coeffs[4];
}

#endif     //_ARMOR_PLATE_HPP


