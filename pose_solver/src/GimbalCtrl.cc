/**=================官方开源弹道模型修改==================
 * 创建时间：2020.1.17 10:57
 * 存在的问题：
 *    1.参数调整：pitch，yaw偏置未调整
 *    2.摩擦系数未确定
 */

#include "GimbalCtrl.hpp"

namespace ballistic {

void GimbalCtrl::Init(float x,float y,float z,float pitch,float yaw, float init_v, float init_k) {
    offset_.x = x;
    offset_.y = y;
    offset_.z = z;
    offset_pitch_ = pitch;
    offset_yaw_ = yaw;
    init_v_ = init_v;
    init_k_ = init_k;
}

#ifdef SINGLE_DIRECTION
//air friction is considered
float GimbalCtrl::BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
    float t;
    t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
    return (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
}
#else
//(air friction) of x/y directions is considered
//直接返回y
float GimbalCtrl::BulletModel(float x, float v, float angle) const {
    return x * GRAVITY/(init_k_ * v * cos(angle)) + tan(angle) * x +
        1/(init_k_ * init_k_) * GRAVITY * log(1-init_k_ * x/ (v * cos(angle)));
}
#endif


//x:distance , y: height
float GimbalCtrl::GetPitch(float x, float y, float v, float &t) {
    float y_temp, y_actual, dy;
    float a;
    y_temp = y;
    // by iteration
    for (int i = 0; i < 20; ++i) {
        a = (float) atan2(y_temp, x);
        y_actual = BulletModel(x, v, a);
        dy = y - y_actual;
        y_temp = y_temp + dy;
        if (fabsf(dy) < 0.001) {
            break;
        }
    }
    t = calcTime(x, v, a);           //计算shoot_delay
    return a*RAD2DEG;             //转为角度
}

void GimbalCtrl::Transform(cv::Point3f postion, float &pitch, float &yaw, float &delay) {
    //y取负数是因为，图像中tVec为正时，装甲板在摄像头正对处下方
    pitch = GetPitch((postion.z + offset_.z) / 1000, -(postion.y + offset_.y) / 1000, init_v_, delay)
                + (float)(offset_pitch_);           //加上一个初始的摄像头角度偏置
    //yaw轴是逆时针为正
    yaw = -(float) (atan2(postion.x + offset_.x, postion.z + offset_.z))*RAD2DEG + (float)(offset_yaw_);
}

float GimbalCtrl::calcTime(const float x, const float v, const float angle){
    #ifdef SINGLE_DIRECTION
        return (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
    #else
        return (float)(-1/(init_k_) * log(1-init_k_ * x/ (v * cos(angle))));
    #endif  //SINGLE_DIRECTION
}

} // ballistic
