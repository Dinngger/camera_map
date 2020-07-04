/**=================官方开源弹道模型修改==================
 * 创建时间：2020.1.17 10:57
 * 存在的问题：
 *    1.参数调整：pitch，yaw偏置未调整
 *    2.摩擦系数未确定
 */
#ifndef _GIMBAL_CTRL_HPP
#define _GIMBAL_CTRL_HPP
//#define DEBUG
//#define SINGLE_DIRECTION                  // 使用单方向空气阻力模型则删除此注释

#include <iostream>
#include <cmath>
#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace ballistic{                        //命名空间：弹道模型

const float GRAVITY = 9.7944;
const float RAD2DEG = 57.29578;             //180/PI
const float DEFAULT_K_42 = 0.035544;        //42mm弹丸的初始k值 ///0.017772f;        
const float DEFAULT_K_17 = 0.08334;        //17MM弹丸的初始k值

class GimbalCtrl{
private:

    /**
     * @fn BulletModel:定义了SIGNGLE_DIRECTION是，定义是单向空气阻力模型，否则为双向空气阻力模型
     * @brief 完全的弹道模型（考虑x与y方向上的空气阻力）
     * @param x x方向上的位移（敌方装甲板到枪口的距离，tVec的[0]位置）
     * @param v 子弹出射初速度
     * @param angle 迭代当前pitch角度
     * @return 本次迭代所计算的pitch实际值（位置）
     */
    float BulletModel(float x, float v, float angle) const;

    /**
     * @brief Get the ballistic control angle
     * @param x Distance from enemy(the armor selected to shoot) to ballistic
     * @param y Value of y in ballistic coordinate.
     * @param v Projectile velocity
     * @param t input/output travel_time of the bullet
     * @return Gimbal pitch angle
     */
    float GetPitch(float x,float y,float v, float &t);

public:
    /**
     * @brief Init the Transformation matrix from camera to ballistic //TODO: write in ros tf
     * @param x Translate x, 单位mm
     * @param y Translate y, 单位mm
     * @param z Translate z, 单位mm
     * @param pitch 摄像头与枪管的pitch角度差, 单位角度（deg）
     * @param yaw 摄像头与枪管的yaw角度差, 单位角度（deg）
     * @param init_v 初速度，单位m/s
     * @param init_k 空气摩擦因数，默认为0.1
     */
    void Init(float x,float y,float z,float pitch,float yaw, float init_v, float init_k = DEFAULT_K_42);


    /**
     * @brief Get the ballistic control info.
     * @param postion Enemy position(actually it should be the target armor).
     * @param pitch Input and output actual pitch angle
     * @param yaw Input and output actual yaw angle
     * @param delay shoot delay while the ball is in the air towards the target
     * 所有的输入参数中，position的单位是mm,pitch，yaw的单位是角度（deg）
     */
    void Transform(cv::Point3f postion,float &pitch,float &yaw, float &delay);

    /**
     * @brief 计算子弹滞空时间
     * @param x 到带打击装甲板的距离
     * @angle 当前仰角
     */
    inline float calcTime(const float x, const float v, const float angle);                    
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
    /**
     * 以下系数针对英雄
     * init k的确定：k = k0/m
     * k0 = C*p*S/2, C为摩擦系数，取0.47, p为空气密度，取1.25kg/m^3, S为接触面积， 取球表面积的一半
     * m 的数值暂不清楚，取高尔夫球质量：0.0458kg,计算的结果：init_k取值为0.017772
     * 小弹丸 3.2 g, k0 = 0.08334
    */
};
} // ballistic

#endif //_GIMBAL_CTRL_HPP