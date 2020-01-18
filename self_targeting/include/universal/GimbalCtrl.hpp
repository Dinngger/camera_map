/**=================官方开源弹道模型============
 * 创建时间：2020.1.17 10:57
 * 存在的问题：
 *  1.参数调整：pitch，yaw偏置未调整
 *  2.摩擦系数未确定
 */
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#define DEBUG
#ifdef DEBUG
  #define print printf
#else
  #define print(...)
#endif

namespace ballistic{                  //命名空间：弹道模型

const double PI = 3.1415926536;
const float GRAVITY = 9.7944;
const float RAD2DEG = 57.29578;       //180/PI

class GimbalCtrl
{
 private:
  /**
   * @brief Calculate the actual y value with air resistance
   * @param x the distanc
   * @param v Projectile velocity
   * @param angle Pitch angle
   * @return The actual y value in the ballistic coordinate
   */
  float BulletModel(float x,float v,float angle);
  /**
   * @brief Get the ballistic control angle
   * @param x Distance from enemy(the armor selected to shoot) to ballistic
   * @param y Value of y in ballistic coordinate.
   * @param v Projectile velocity
   * @return Gimbal pitch angle
   */
  float GetPitch(float x,float y,float v);

 public:
  /**
   * @brief Init the Transformation matrix from camera to ballistic //TODO: write in ros tf
   * @param x Translate x, 单位mm
   * @param y Translate y, 单位mm
   * @param z Translate z, 单位mm
   * @param pitch Rotate pitch, 单位角度（deg）
   * @param yaw Rotate yaw, 单位角度（deg）
   * @param init_v 初速度，单位m/s
   * @param init_K 空气摩擦因数，默认为0.1
   */
  void Init(float x,float y,float z,float pitch,float yaw, float init_v, float init_k = 0.1);


  /**
   * @brief Get the ballistic control info.
   * @param postion Enemy position(actually it should be the target armor).
   * @param pitch Input and output actual pitch angle
   * @param yaw Input and output actual yaw angle
   * 所有的输入参数中，position的单位是mm,pitch，yaw的单位是角度（deg）
   */
  void Transform(cv::Point3f postion,float &pitch,float &yaw);

 private:
  //! Transformation matrix between camera coordinate system and ballistic coordinate system.
  //! Translation unit: cm
  cv::Point3f offset_;
  //! Rotation matrix unit: degree
  float offset_pitch_;
  float offset_yaw_;

  //! Initial value
  float init_v_;
  float init_k_;

};

} //namespace ballistic

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

//air friction is considered
float GimbalCtrl::BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
  float t, y;
  t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
  y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
  return y;
}

//x:distance , y: height
float GimbalCtrl::GetPitch(float x, float y, float v) {
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
    //printf("Iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i +1, a*RAD2DEG, y_temp, dy);
  }
  return a*RAD2DEG;       //转为角度
}

void GimbalCtrl::Transform(cv::Point3f postion, float &pitch, float &yaw) {
  //y取负数是因为，图像中tVec为正时，装甲板在摄像头正对处下方
  pitch = GetPitch((postion.z + offset_.z) / 1000, -(postion.y + offset_.y) / 1000, init_v_)
        + (float)(offset_pitch_);
  //yaw轴是逆时针为正
  yaw = -(float) (atan2(postion.x + offset_.x, postion.z + offset_.z))*RAD2DEG + (float)(offset_yaw_);
}

} // ballistic
