/**============新的MatchLight(明暗交替)============
 * 作者：hqy
 * 创建时间：2020.2.9 00:27
    输入按照装甲板上点的顺序: 从左上角开始逆时针,
    |0        3|
    |1        2|   
 */

#include "ArmorPlate.hpp"
#include "params.hpp"

/// @ref https://wenku.baidu.com/view/0f3c083a172ded630a1cb6c8.html
std::map<int, float> grubbs_length = {
    {3, 1.329}, {4, 2.140}, {5, 2.796}, {6, 3.320}, {7, 3.756}, {8, 4.129}, {9, 4.452}, {10, 4.735}
};

std::map<int, float> grubbs_angle = {
    {3, 1.500}, {4, 2.238}, {5, 3.112}, {6, 3.893}, {7, 4.575}, {8, 5.171}, {9, 5.698}, {10, 6.160}
};

ArmorPlate::ArmorPlate(){
    for(int i=0; i<4 ;++i) points[i] = aim_deps::NULLPOINT2f;
    clf = torch::jit::load(std::string(proj_path) + "pose_solver/traced_model.pt");
}

ArmorPlate::~ArmorPlate(){
    ;
}

void ArmorPlate::matchAll(
    const std::vector<int>& car,
    const std::vector<cv::Point> &matches,
    std::vector<aim_deps::Light> &lights,
    std::vector<aim_deps::Armor> &tar_list
)
{
    tar_list.clear();
    std::vector<torch::jit::IValue> inputs;
    for(size_t i = 0 ; i < matches.size() ; ++i){
        const aim_deps::Light& l1 = lights[matches[i].x];
        const aim_deps::Light& l2 = lights[matches[i].y];
        std::vector<float> feature(5);
        feature[0] = l1.box.length;
        feature[1] = l2.box.length;
        cv::Point2f vec = l2.box.center - l1.box.center;
        float cosa = std::cos(-l1.box.angle), sina = std::sin(-l1.box.angle);
        feature[2] = cosa * vec.x - sina * vec.y;
        feature[3] = sina * vec.x + cosa * vec.y;
        feature[4] = l2.box.angle - l1.box.angle;
        at::Tensor ts = torch::tensor(feature).reshape({1, 5});
        at::Tensor tmp = torch::softmax(clf.forward({ts}).toTensor(), 1);
        float out = tmp.index({0, 1}).item().toFloat();
        if (out > 0.5) {
            const aim_deps::Light& l1 = lights[matches[i].x];
            const aim_deps::Light& l2 = lights[matches[i].y];
            points[0] = l1.box.vex[0];
            points[1] = l1.box.vex[1];
            points[2] = l2.box.vex[1];
            points[3] = l2.box.vex[0];
            tar_list.emplace_back(points, 0, l1, l2);
            tar_list.back().valid = true;
        }
    }
    filter(tar_list, lights.size());
}

void ArmorPlate::drawArmorPlates(cv::Mat &src, 
    const std::vector<aim_deps::Armor>& tar_list, const int optimal) const{
	char str[2];
    cv::line(src, cv::Point(720, 0), cv::Point(720, 1080), cv::Scalar(255, 0, 0));
	cv::line(src, cv::Point(0, 540), cv::Point(1440, 540), cv::Scalar(255, 0, 0));
    for (size_t i = 0; i< tar_list.size(); ++i) {
        if(tar_list[i].armor_number != -1){   //有意义的数字
            if(!tar_list[i].valid){       //非最佳装甲板使用黄色绘制
                for (int j = 0; j < 4; ++j){
                    cv::line(src, tar_list[i].vertex[j], 
                    tar_list[i].vertex[(j + 1) % 4], cv::Scalar(0, 255, 255), 1);   
                }
            }
            else{                   //最佳装甲板使用绿色绘制
                for (int j = 0; j < 4; ++j){
                    cv::line(src, tar_list[i].vertex[j],
                    tar_list[i].vertex[(j + 1) % 4], cv::Scalar(0, 255, 0), 1);
                }
            }
            ///snprintf(str, 2, "%d", j);      //最佳装甲板位置x
	        ///cv::putText(src, str, tar_list[i].vertex[j]+cv::Point2f(2, 2),
	        ///    cv::FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(0, 100, 255));
            snprintf(str, 2, "%d", tar_list[i].armor_number);
            cv::putText(src, str, tar_list[i].vertex[2] + cv::Point2f(8, 4),
                cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255));
        }
    }
}

void ArmorPlate::filter(
    std::vector<aim_deps::Armor> &tar_list,
    size_t lights_size
) const {
    std::vector<std::vector<aim_deps::Armor*> > arms(lights_size);
    for (aim_deps::Armor& arm: tar_list) {                        // 共灯条，如果没有共灯条，那么一个内vector最多有一个Armor
        arms[arm.left_light.index].emplace_back(&arm);
        arms[arm.right_light.index].emplace_back(&arm);
    }
    for (std::vector<aim_deps::Armor*>& ptrs: arms) {
        float mini = 90.0;
        size_t mini_pos = 0;
        for (size_t i = 0; i < ptrs.size(); i++) {
            if (ptrs[i]->valid == false) continue; 
            float val = std::abs(ptrs[i]->left_light.box.angle - ptrs[i]->right_light.box.angle);
            if (val < mini) {
                mini = val;
                mini_pos = i;
            }
        }
        for (size_t i = 0; i < ptrs.size(); i++) {
            if (ptrs[i]->valid == false) continue; 
            ptrs[i]->valid = (i == mini_pos);
        }
    }
}