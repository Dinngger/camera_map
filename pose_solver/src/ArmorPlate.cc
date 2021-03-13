/**============新的MatchLight(明暗交替)============
 * 作者：hqy
 * 创建时间：2020.2.9 00:27
 * 主要思路：设置几个判断条件：
 */

#include "ArmorPlate.hpp"

ArmorPlate::ArmorPlate(){
    for(int i=0; i<4 ;++i) points[i] = aim_deps::NULLPOINT2f;
}

ArmorPlate::~ArmorPlate(){
    ;
}

void ArmorPlate::matchAll(
    const std::vector<cv::Point> &matches,
    std::vector<aim_deps::Light> &lights,
    std::vector<aim_deps::Armor> &tar_list
)
{
    tar_list.clear();
    for(size_t i = 0 ; i < matches.size() ; ++i){
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
    filter(tar_list, lights);                   //过滤无效装甲板
    for (aim_deps::Armor& tar: tar_list) {
        if (tar.valid) {
            aim_deps::Light& l = tar.left_light, r = tar.right_light;
            l.isLeft = 1;
            lights[l.index].isLeft = 1;
            r.isLeft = 0;
            lights[r.index].isLeft = 0;
        }
    }
}

bool ArmorPlate::isMatch(const aim_deps::Light &l1, const aim_deps::Light &l2)
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
    // printf("Arm(%d, %d): length_ratio: ", l1.index, l2.index);

    if(isRatioValid(l1.box.length, l2.box.length) && isEdgesValid() && isAreaGood()){
        amp_debug(rmlog::F_BLUE, "Push in:(", l1.index, ", ", l2.index, ")");
        return true;
    }
    else{
        amp_debug(rmlog::F_RED, "Ratio mismatch or area too small:(", l1.index, ", ", l2.index, ")");
    }
    return false;
}

void ArmorPlate::drawArmorPlates(cv::Mat &src, 
    const std::vector<aim_deps::Armor>& tar_list, const int optimal) const{
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
            cv::putText(src, str, tar_list[i].vertex[2] + cv::Point2f(8, 4),
                cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255));
        }
    }
}

void ArmorPlate::filter(std::vector<aim_deps::Armor> &tar_list, std::vector<aim_deps::Light> &lights){
    for(size_t i = 0; i<tar_list.size(); ++i){
        if(tar_list[i].valid){
            for(size_t j = i+1; j<tar_list.size(); ++j){
                if(tar_list[j].valid){
                    int judge = tar_list[i].collide(tar_list[j]);
                    if(judge >= 0){                  //装甲板有共灯条冲突
                        int start1 = 1, start2 = 1;
                        // 是左灯条，则从位置3开始算夹角, 是右灯条，则从位置1开始算夹角
                        aim_deps::Light &li = tar_list[i].left_light,
                            &lj = tar_list[j].left_light,
                            &ri = tar_list[i].right_light,
                            &rj = tar_list[j].right_light;
                        if(judge == li.index) start1 = 3;         
                        if(judge == lj.index) start2 = 3;
                        if(start1 == start2){           // 冲突的灯条对于两个装甲板都是左灯条或者右灯条
                            if(start1 == 3){            // 冲突灯条均为左灯条
                                if(ri.box.center.x < rj.box.center.x){
                                    tar_list[j].valid = false; continue; //内层判断为无效则继续内层循环
                                }else{
                                    tar_list[i].valid = false; break;    //外层判断无效则退出内层循环
                                }
                            }
                            else{                       // 冲突灯条均为右灯条
                                if(li.box.center.x > lj.box.center.x){
                                    tar_list[j].valid = false; continue;
                                }else{
                                    tar_list[i].valid = false; break;
                                }
                            }
                        }
                        double cos1 = cornerAngle(tar_list[i].vertex, start1),
                            cos2 = cornerAngle(tar_list[j].vertex, start2); 
                        float diff1 = li.box.angle - ri.box.angle,
                            diff2 = lj.box.angle - rj.box.angle;
                        diff1 *= aim_deps::DEG2RAD;
                        diff2 *= aim_deps::DEG2RAD;
                        // 加权判断共灯条的两装甲哪一个是真实装甲板(需要考虑装甲板接近矩形的程度以及灯条角度差)

                        if( 0.5*(1 - cos(diff1)) + cos1 > 0.5*(1 - cos(diff2)) + cos2){
                            tar_list[i].valid = false; 
                            amp_debug(rmlog::F_RED, "Light ", li.index, " & ",
                                    ri.index, " are invalid:", 
                                    0.5*(1 - cos(diff1)) + cos1, ", ", 0.5*(1 - cos(diff2)) + cos2);
                            break;
                        }
                        else{
                            tar_list[j].valid = false;
                            amp_debug(rmlog::F_RED, "Light ", lj.index, " & ",
                                    rj.index, " are invalid:", 
                                    0.5*(1 - cos(diff1)) + cos1, ", ", 0.5*(1 - cos(diff2)) + cos2);
                        }
                    }
                }
            }
        }
    }
    for(size_t i = 0; i<tar_list.size(); ++i){
        if(tar_list[i].valid){
            tar_list[i].left_light.valid = true;
            tar_list[i].right_light.valid = true;
            lights[tar_list[i].left_light.index].valid = true;
            lights[tar_list[i].right_light.index].valid = true;
        }
    }
}

bool ArmorPlate::getArmorPlate(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2){
    //lightCompensate(b1, b2);
    points[0] = b1.vex[0];
    points[1] = b1.vex[1];
    points[2] = b2.vex[1];
    points[3] = b2.vex[0];
    cv::Point2f diff = points[0] - points[1];
    /// 这个地方的意思是：需要灯条有合适的角度（cot值必须小于1.5）
    return diff.x/diff.y < 1.5;                 
}

bool ArmorPlate::isRatioValid(float l1, float l2) const{                    //对边中点连线的长度平方比值是否合适
    float len1 = aim_deps::getPointDist((points[0]+points[1])/2, (points[2]+points[3])/2),
         len2 = (l1 + l2) / 2,
         ratio = len1/(len2 * len2), 
         thresh = lengthThreshold(ratio);
    // DEBUG
    // printf("%f and sqrt(ratio): %f, with threshold: %f\n", ratio, sqrt(ratio), thresh);
    if(len2 < 5.0){                                 //灯条高度平方小于25时，过小的两灯条需要进行一个判断
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
bool ArmorPlate::isEdgesValid() const{  //对边长度平方比值是否合适
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
    /// DEBUG
    //if(!judge1) amp_debug(rmlog::F_RED, "Judge 1 failed:", edges[0]/edges[2], ", ", edges[2]/edges[0]);
    //if(!judge2) amp_debug(rmlog::F_RED, "Judge 2 failed:", edges[1]/edges[3], ", ", edges[3]/edges[1]);
    return false;
}

bool ArmorPlate::isAngleMatch(float ang1, float ang2) const{
    //输入按照装甲板上点的顺序: 从左上角开始逆时针,
    // |0        3|
    // |1        2|   
    return (std::abs(ang1-ang2) < params.ANGLE_THRESH);
}

bool ArmorPlate::isAreaGood() const{
    std::vector<cv::Point2f> tmp = {points[0], points[1], points[2], points[3]};
    return cv::contourArea(tmp) >= aim_deps::MIN_ARMOR_AREA;
}

float ArmorPlate::lengthThreshold(const float ratio){                 
    if(ratio >= 19.36) return 20.0;
    else if(ratio >= 14.44) return 1.13971147 * ratio - 2.10587699;
    else return 16.0;
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