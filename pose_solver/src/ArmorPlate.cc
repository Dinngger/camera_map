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
    using namespace cv::ml;
    for(int i=0; i<4 ;++i) points[i] = aim_deps::NULLPOINT2f;
    mlp = ANN_MLP::load(std::string(proj_path) + "pose_solver/mlp.xml");
    printf("MLP status: trained(%d), empty(%d)\n", mlp->isTrained(), mlp->empty());
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
    for(size_t i = 0 ; i < matches.size() ; ++i){
        const aim_deps::Light& l1 = lights[matches[i].x];
        const aim_deps::Light& l2 = lights[matches[i].y];
        // float score = armorScore(l1, l2);       // 注意，直接predict时输出0为是灯条
        // if (std::round(score) == 1) {
        //         points[0] = l1.box.vex[0];
        //         points[1] = l1.box.vex[1];
        //         points[2] = l2.box.vex[1];
        //         points[3] = l2.box.vex[0];
        //         tar_list.emplace_back(points, 0, l1, l2);
        // }
        points[0] = l1.box.vex[0];
        points[1] = l1.box.vex[1];
        points[2] = l2.box.vex[1];
        points[3] = l2.box.vex[0];
        tar_list.emplace_back(points, 0, l1, l2);
        tar_list.back().valid = true;
    }
    // if (car.size()) {
    //     filter(car, tar_list, lights);                   //过滤无效装甲板
    // }
}

float ArmorPlate::armorScore(
    const aim_deps::Light& l1,
    const aim_deps::Light& l2
) const {
    float feature[5] = {0.0};
    feature[0] = l1.box.length;
    feature[1] = l2.box.length;
    cv::Point2f vec = l1.box.center - l1.box.center;
    float cosa = std::cos(-l1.box.angle), sina = std::sin(-l1.box.angle);
    feature[2] = cosa * vec.x - sina * vec.y;
    feature[3] = sina * vec.x + cosa * vec.y;
    feature[4] = l2.box.angle - l1.box.angle;
    cv::Mat dst;
    mlp->predict(cv::Mat(1, _FEAT_NUM, CV_32FC1, feature), dst);
    return dst.at<float>(0);
}

void ArmorPlate::drawArmorPlates(cv::Mat &src, 
    const std::vector<aim_deps::Armor>& tar_list, const int optimal) const{
	char str[2];
    cv::line(src, cv::Point(720, 0), cv::Point(720, 1080), cv::Scalar(255, 0, 0));
	cv::line(src, cv::Point(0, 540), cv::Point(1440, 540), cv::Scalar(255, 0, 0));
    for (size_t i = 0; i< tar_list.size(); ++i) {
        if(tar_list[i].armor_number != -1){   //有意义的数字
            if((int)i != optimal){       //非最佳装甲板使用黄色绘制
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

template <bool use_angle>
void ArmorPlate::GrubbsIteration(CarLight& lights) {
    if (lights.size() < 3) return;
    for (size_t k = 0; k < lights.size(); k++) {
        size_t size = 0, max_pos = 0, max_dist = 0;
        float mean = 0.0, var = 0.0;
        for (size_t i = 0; i < lights.size(); i++) {
            if (lights[i]->valid == true) continue;
            mean += use_angle ? lights[i]->box.angle : lights[i]->box.length;
            size ++;
        }
        mean /= float(size);
        for (size_t i = 0; i < lights.size(); i++) {
            if (lights[i]->valid == true) continue;
            float d2 = use_angle ? std::pow(lights[i]->box.angle - mean, 2) : std::pow(lights[i]->box.length - mean, 2);
            if (d2 > max_dist) {
                max_dist = d2;
                max_pos = i;
            }
            var += d2;
        }
        var /= (float(size) - 1.0);
        float threshold = use_angle ? grubbs_angle[size] * var : grubbs_length[size] * var;
        if (max_dist > threshold) {
            lights[max_pos]->valid = false;
            if (size - 1 < 3) break;
        }
        else {                              // Grubbs准则没有找到outlier
            break;
        }
    }
}

void ArmorPlate::filter(const std::vector<int> &car, std::vector<aim_deps::Armor> &tar_list, std::vector<aim_deps::Light> &lights){
    std::vector<CarLight> car_lights(_MAX_CAR_NUM);
    std::vector<CarLight> valid_lights(_MAX_CAR_NUM);
    for (size_t i = 0; i < lights.size(); i++) {
        int index = car[i];
        if (index > 0) {
            car_lights[index - 1].emplace_back(&lights[i]);
        }
    }
    #pragma omp parallel for num_threads(4)
    for (size_t i = 0; i < car_lights.size(); i++) {
        GrubbsIteration<true>(car_lights[i]);       // 每辆车进行 长度 outlier detection （长度更加严格）
        GrubbsIteration<false>(car_lights[i]);      // 角度 outlier detection （角度没有那么严格）
        for (const LightPtr& lt: car_lights[i]) {
            if (lt->valid == true) {
                valid_lights[i].emplace_back(lt);
            }
        }
    }
    int match_lut[lights.size()];            // lookup table for matching
    memset(match_lut, -1, sizeof(int) * lights.size());
    for (size_t i = 0; i < valid_lights.size(); i++) {
        const CarLight& lts = valid_lights[i];
        size_t size = lts.size();
        if (size > 4) {                 // 5个及以上灯条
            if (lts.front()->box.length >= lts.back()->box.length) {  // 首灯条更长
                simpleKM(lts, match_lut);
            }
            else {
                float angle_score[3];
                for (int j = 1; j < 4; j++) {
                    angle_score[j - 1] = armorScore(*lts[size - j], *lts[size - j - 1]);
                }
                int start_pos = angle_score[0] + angle_score[2] > angle_score[1] + 1.0 ? size - 1 : size - 2;
                for (int j = start_pos; j > 0; j -= 2) {
                    int this_index = lts[j]->index,
                        next_index = lts[j - 1]->index;
                    match_lut[this_index] = next_index;
                    match_lut[next_index] = this_index;
                }
            }
        }
        else if (size == 4) {           // 4个灯条
            simpleKM(lts, match_lut);
        }
        else if (size == 3) {           // 3个灯条
            float score1 = 1 - armorScore(*lts[0], *lts[1]),
                score2 = 1 - armorScore(*lts[1], *lts[2]);
            if (score1 < score2){
                match_lut[lts[0]->index] = lts[1]->index;
                match_lut[lts[1]->index] = lts[0]->index;
            }
            else {
                match_lut[lts[1]->index] = lts[2]->index;
                match_lut[lts[2]->index] = lts[1]->index;
            }
        }
        else if (size == 2) {
            match_lut[lts[0]->index] = lts[1]->index;
            match_lut[lts[1]->index] = lts[0]->index;
        }
    }
    // 根据match_lut进行匹配
    for(aim_deps::Armor& tar: tar_list){
        int left_i = tar.left_light.index, right_i = tar.right_light.index;
        if (match_lut[left_i] != right_i || match_lut[right_i] != left_i) {
            tar.valid = false;
        }
        else {
            tar.valid = true;
            tar.left_light.isLeft = 1;
            lights[left_i].isLeft = 1;
            tar.right_light.isLeft = 0;
            lights[right_i].isLeft = 0;
        }
    }
}

void ArmorPlate::simpleKM(const CarLight& lights, int* lut) {
    float angle_score[3];
    for (int j = 0; j < 3; j++) {
        angle_score[j] = armorScore(*lights[j], *lights[j + 1]);
    }
    int start_pos = angle_score[0] + angle_score[2] > angle_score[1] + 1.0 ? 0 : 1;
    for (size_t j = start_pos; j < lights.size() - 1; j+= 2) {
        int this_index = lights[j]->index,
            next_index = lights[j + 1]->index;
        lut[this_index] = next_index;
        lut[next_index] = this_index;
    }
}