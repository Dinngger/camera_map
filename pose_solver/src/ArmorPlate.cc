/**============新的MatchLight(明暗交替)============
 * 作者：hqy
 * 创建时间：2020.2.9 00:27
 * 主要思路：设置几个判断条件：（2020.1.18日写）
 * 9/2/2020 修改：
 *      drawArmorPlate需要通过数字判断其是否valid
 */

#include "../include/ArmorPlate.hpp"

ArmorPlate::ArmorPlate(){
    for(int i=0; i<4 ;++i) points[i] = aim_deps::NULLPOINT2f;
}

ArmorPlate::~ArmorPlate(){;}

void ArmorPlate::matchAll(
    std::vector<cv::Point> matches,
    std::vector<aim_deps::Light> lights,
    std::vector<aim_deps::Armor> &tar_list
)
{
    tar_list.clear();
    for(size_t i = 0 ; i<matches.size() ; ++i){
        if(isMatch(lights[matches[i].x],lights[matches[i].y])){
            if(lights[matches[i].x].box.center.x < lights[matches[i].y].box.center.x)
            {
                tar_list.emplace_back(aim_deps::Armor(points, 0, lights[matches[i].x], lights[matches[i].y]));
            }
            else
            {
                tar_list.emplace_back(aim_deps::Armor(points, 0, lights[matches[i].y], lights[matches[i].x]));
            }
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
        judge = getArmorPlate(l2.box, l1.box);          //始终保持第一个入参是x轴坐标小的灯条
    if(!judge) return false;                        
    if(!isAngleMatch(l1.box.angle, l2.box.angle)){
        amp_debug(rmlog::F_RED, "Angle mismatch:(", l1.index, ", ", l2.index, ")");
        return false;
    }
    if(isRatioValid() && isEdgesValid() && isAreaGood()){
        amp_debug(rmlog::F_BLUE, "Push in:(", l1.index, ", ", l2.index, ")");
        return true;
    }
    else{
        amp_debug(rmlog::F_RED, "Ratio mismatch or area too small:(", l1.index, ", ", l2.index, ")");
    }
    return false;
}

void ArmorPlate::drawArmorPlates(cv::Mat &src, 
    const std::vector<aim_deps::Armor> tar_list, const int optimal){
	char str[2];
    cv::line(src, cv::Point(720, 0), cv::Point(720, 1080), cv::Scalar(255, 0, 0));
	cv::line(src, cv::Point(0, 540), cv::Point(1440, 540), cv::Scalar(255, 0, 0));
    for (size_t i = 0; i< tar_list.size(); ++i) {
        if(tar_list[i].armor_number != -1 && tar_list[i].valid){   //有意义的数字
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
            cv::putText(src, str, tar_list[i].center + cv::Point2f(25, 10),
                cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255));
            cv::circle(src, tar_list[i].center, 3, cv::Scalar(0, 0, 255), -1);
        }
    }
}

void ArmorPlate::filter(std::vector<aim_deps::Armor> &tar_list){
    for(size_t i = 0; i<tar_list.size(); ++i){
        if(tar_list[i].valid){
            for(size_t j = i+1; j<tar_list.size(); ++j){
                if(tar_list[j].valid){
                    /// TODO: 此处是否判断失效？
                    int judge = tar_list[i].collide(tar_list[j]);
                    if(judge >= 0){                  //装甲板有共灯条冲突
                        int start1 = 1, start2 = 1;
                        // 是左灯条，则从位置3开始算夹角, 是右灯条，则从位置1开始算夹角
                        if(judge == tar_list[i].left_light.index) start1 = 3;         
                        else start1 = 1;         
                        if(judge == tar_list[j].left_light.index) start2 = 3;
                        else start2 = 1;
                        double cos1 = cornerAngle(tar_list[i].vertex, start1),
                            cos2 = cornerAngle(tar_list[j].vertex, start2); 
                        float diff1 = tar_list[i].left_light.box.angle - 
                                    tar_list[i].right_light.box.angle,
                            diff2 = tar_list[j].left_light.box.angle - 
                                    tar_list[j].right_light.box.angle;
                        diff1 *= aim_deps::DEG2RAD;
                        diff2 *= aim_deps::DEG2RAD;
                        if( 0.5*(1 - cos(diff1)) + cos1 > 0.5*(1 - cos(diff2)) + cos2){
                            tar_list[i].valid = false;
                            amp_debug(rmlog::F_RED, "Light ", tar_list[i].left_light.index, " & ",
                                     tar_list[i].right_light.index, " are invalid:", 
                                     0.5*(1 - cos(diff1)) + cos1, ", ", 0.5*(1 - cos(diff2)) + cos2);
                            break;
                        }
                        else{
                            tar_list[j].valid = false;
                             amp_debug(rmlog::F_RED, "Light ", tar_list[j].left_light.index, " & ",
                                     tar_list[j].right_light.index, " are invalid:", 
                                     0.5*(1 - cos(diff1)) + cos1, ", ", 0.5*(1 - cos(diff2)) + cos2);
                        }
                    }
                }
            }
        }
    }
    for(size_t i = 0; i<tar_list.size(); ++i){
        if(tar_list[i].valid){
            lightCompensate(tar_list[i].left_light.box, tar_list[i].right_light.box, &tar_list[i]);
            //printf("Now the Light %d: %f\n", tar_list[i].left_light.index, tar_list[i].left_light.box.length);
            //printf("Now the Light %d: %f\n", tar_list[i].right_light.index, tar_list[i].right_light.box.length);    
        }
    }
}

bool ArmorPlate::getArmorPlate(aim_deps::LightBox b1, aim_deps::LightBox b2){
    lightCompensate(b1, b2);
    points[0] = b1.vex[0];
    points[1] = b1.vex[1];
    points[2] = b2.vex[1];
    points[3] = b2.vex[0];
    cv::Point2f diff = points[0]-points[1];
    /// 这个地方的意思是：需要灯条有合适的角度（cot值必须小于1.5）
    return diff.x/diff.y < 1.5;                 
}

bool ArmorPlate::isRatioValid(){                    //对边中点连线的长度平方比值是否合适
    float len1 = aim_deps::getPointDist((points[0]+points[1])/2, (points[2]+points[3])/2),
         len2 = aim_deps::getPointDist((points[0]+points[3])/2, (points[1]+points[2])/2),
         ratio = len1/len2, 
         thresh = getRatio(len2);
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
    for(int i = 0; i<4; ++i){
        edges[i] = aim_deps::getPointDist(points[i], points[(i+1)%4]);
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

bool ArmorPlate::isAngleMatch(const float ang1, const float ang2){
    //输入按照装甲板上点的顺序: 从左上角开始逆时针,
    // |0        3|
    // |1        2|   
    if (std::abs(ang1-ang2) < params.ANGLE_THRESH){
        return true;
    }
    return false;
}

bool ArmorPlate::isAreaGood(){
    std::vector<cv::Point2f> tmp = {points[0], points[1], points[2], points[3]};
    /// float res = cv::contourArea(tmp);
    return cv::contourArea(tmp) >= aim_deps::MIN_ARMOR_AREA;
}

float ArmorPlate::getRatio(const float l){                 
    // 默认输入的len2是灯条长度平方的平均值
    if(l > 56.25) return aim_deps::distance_params.NEAR_RATIO_MIN;            //12.5   (56.25是7.5的平方)
    else if(l <= 14.44) return aim_deps::distance_params.NEAR_RATIO_MAX;      //17.64是3.8（像素）的平方，30.0
    float len = sqrt(l);                    //最后函数的输入是sqrt(l),是一个关于sqrt(l)的四次函数
    return aim_deps::coeffs[0] * powf(len, 4) + aim_deps::coeffs[1] * powf(len, 3) +
            aim_deps::coeffs[2] * len * len + aim_deps::coeffs[3] * len + aim_deps::coeffs[4];
}

void ArmorPlate::lightCompensate(
    aim_deps::LightBox &l1,
    aim_deps::LightBox &l2, 
    aim_deps::Armor *_a
){
    float max_len = cv::max(l1.length, l2.length);
    if(_a != nullptr){
        if(max_len >= 14.0 && std::max( std::abs(l1.angle), std::abs(l2.angle)) >= 2.0){
            aim_deps::LightBox *min_box, *max_box;
            if(l1.length > l2.length){
                min_box = &l2;
                max_box = &l1;
            }
            else{
                min_box = &l1;
                max_box = &l2;
            }
            //printf("Lights %d and %d\n", _a->left_light.index, _a->right_light.index);
            //printf("Max len : %f, min_len : %f\n", max_len, min_box->length);
            if(max_len / min_box->length >= 1.3){
                if(min_box->ref == aim_deps::NULLPOINT2f){          //参考点没有被赋值成功
                    cv::Point2f diff_up = min_box->center - max_box->vex[0];
                    cv::Point2f diff_center = min_box->center - max_box->center;
                    if( std::abs( diff_up.y ) < std::abs( diff_center.y ) ){              // 需补偿box的中点离正确box的上顶点更接近
                        min_box->add((max_len - min_box->length) * 0.5, false); // 补偿下方，下顶点下移 
                    }           
                    else{                                                       // 需补偿box的中点离正确box的中点更接近
                        min_box->add((max_len - min_box->length) * 0.5, true);  // 补偿上方（上顶点上移）
                    }
                    //printf("No reference point.\n");
                }else{
                    min_box->rebuild(max_box->vex, max_len);
                }
                if(min_box == &(_a->left_light.box)){       // [0][1]点重新赋值
                    _a->vertex[0] = min_box->vex[0];
                    _a->vertex[1] = min_box->vex[1];
                    // printf("Left\n");
                }
                else if(min_box == &(_a->right_light.box)){ // [2][3]点重新赋值
                    _a->vertex[3] = min_box->vex[0];
                    _a->vertex[2] = min_box->vex[1];
                    // printf("Right\n");
                }
            }
        }
        return;
    }
    float len_diff = std::abs(l1.length - l2.length);
    if(max_len <= 14.0 && len_diff > 0.5){
        if( (std::abs(l1.angle) < 1.0 || std::abs(l1.angle + 90.0) < 1.0) &&
            (std::abs(l2.angle) < 1.0 || std::abs(l2.angle + 90.0) < 1.0) ){
            // 灯条过小时(长度14.0以下).(远处装甲板一般都是正对的,稍侧转将在低曝光下无法被捕捉)
            // 此时如果两灯条长度存在差别，则需要修正，让两灯条长度一致
            if( l1.length < l2.length){             
                l1.add(len_diff * 0.5);
            }
            else{
                l2.add(len_diff * 0.5);
            }
        }
        if(max_len <= 5.0) {       //可能略大
            l1.add(2.1);
            l2.add(2.1);
        }
        else{
            float comp = compensation(max_len);
            l1.add(comp * 0.8);
            l2.add(comp * 0.8);
            // printf("Comp added to the lights:%f\n", comp);
        }
    }
}

float ArmorPlate::cornerAngle(const cv::Point2f *pts, const int start){
    // 计算四边形四个角的大小，与90度偏差越小，说明与矩形越接近，则可以判断两个共灯条装甲板哪一个才是正确的匹配
    float _sum = 0.0;
    for(int i = 0; i < 4; ++i){
        // 内积角度(两相邻边)
        float inner_pro = (pts[(i+1)%4] - pts[i%4]).dot((pts[(i+2)%4] - pts[(i+1)%4]));
        inner_pro /= sqrt(aim_deps::getPointDist(pts[(i+1)%4], pts[i%4]));
        inner_pro /= sqrt(aim_deps::getPointDist(pts[(i+2)%4], pts[(i+1)%4]));
        // 权重，共灯条处的角度权重大
        if(i != start % 4 || i != (start + 1)%4 )
            _sum += 0.5 * std::abs(inner_pro);              //直接加近似的cos theta
        else
            _sum += std::abs(inner_pro);
    }
    return _sum;
}

float ArmorPlate::compensation(const float mean){
    return 0.001564f * powf(mean, 3) - 0.042191f * mean * mean + 0.029147f * mean + 3.5707f;
}
