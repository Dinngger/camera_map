/**============新的MatchLight(明暗交替)============
 * 作者：hqy
 * 创建时间：2020.2.9 00:27
 * 修改时间：2021.3.18 ANN_MLP
 */
#ifndef _ARMOR_PLATE_HPP
#define _ARMOR_PLATE_HPP

#include <map>
#include <queue>
#include <vector>
#include <memory>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <torch/torch.h>
#include <torch/script.h>
#include "LOG.hpp"
#include "AimDeps.hpp"
// #define ARMORPLATE_DEBUG     // ArmorPlate   装甲板匹配模块       装甲板配对信息显示
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

    void filter(
        std::vector<aim_deps::Armor> &tar_list,
        size_t lights_size
    ) const;

    /**
     * @brief 绘制装甲板于图像
     * @param src 输入/输出,用于绘制的图像
     * @param tar_list 装甲板列表
     * @param optimal 最优装甲板的位置(使用绿色绘制)
     */
    void drawArmorPlates(cv::Mat &src, 
        const std::vector<aim_deps::Armor>& tar_list, const int optimal) const;                                            //消息发布
private:
    bool _is_enemy_blue;                                                //敌人颜色
    cv::Point2f points[4];                                              //装甲板点列的临时容器
    aim_deps::Distance_Params params;                                   //装甲板匹配参数
    torch::jit::script::Module clf;
};
#endif     //_ARMOR_PLATE_HPP