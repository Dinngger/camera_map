/**============新的MatchLight(明暗交替)============
 * 作者：hqy
 * 创建时间：2020.2.9 00:27
 * 主要思路：设置几个判断条件：（2020.1.18日写）
 * 9/2/2020 修改：
 *      drawArmorPlate需要通过数字判断其是否valid
 */
#ifndef _ARMOR_PLATE_HPP
#define _ARMOR_PLATE_HPP

#include <map>
#include <vector>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ml.hpp>
#include "LOG.hpp"
#include "AimDeps.hpp"
// #define ARMORPLATE_DEBUG     // ArmorPlate   装甲板匹配模块       装甲板配对信息显示
#ifdef ARMORPLATE_DEBUG
    #define amp_debug rmlog::LOG::printc        //彩色输出
#else
    #define amp_debug(...)
#endif
#define _MAX_CAR_NUM 2
#define _FEAT_NUM 5

typedef aim_deps::Light* LightPtr;
typedef std::vector<LightPtr> CarLight;

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
        const std::vector<int>& car,
        const std::vector<cv::Point>& matches,
        std::vector<aim_deps::Light> &lights, 
        std::vector<aim_deps::Armor> &tar_list);
    /**
     * @brief 绘制装甲板于图像
     * @param src 输入/输出,用于绘制的图像
     * @param tar_list 装甲板列表
     * @param optimal 最优装甲板的位置(使用绿色绘制)
     */
    void drawArmorPlates(cv::Mat &src, 
        const std::vector<aim_deps::Armor>& tar_list, const int optimal) const;                                            //消息发布
private:
    /**
     * @brief 对两个灯条的RotatedRect进行匹配，直接在tar_list中emplace_back符合条件的装甲板
     * @param l1 灯条1
     * @param l2 灯条2
    */

    void filter(
        const std::vector<int> &car,
        std::vector<aim_deps::Armor> &tar_list,
        std::vector<aim_deps::Light> &lights
    );                      //进一步过滤装甲板容器

    float armorScore(
        const aim_deps::Light& l1,
        const aim_deps::Light& l2
    ) const;

    // 2v2 完全二分图最大权算法
    void simpleKM(const CarLight& lights, int* lut);

    template <bool use_angle = false>
    void GrubbsIteration(CarLight& lights);
private:
    bool _is_enemy_blue;                                                //敌人颜色
    cv::Point2f points[4];                                              //装甲板点列的临时容器
    cv::Ptr<cv::ml::ANN_MLP> mlp;                                       // MLP装甲板判定
    aim_deps::Distance_Params params;                                   //装甲板匹配参数
};
#endif     //_ARMOR_PLATE_HPP