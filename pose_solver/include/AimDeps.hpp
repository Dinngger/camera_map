/**=================AIM_DEPENDENCIES================
 * @author hqy
 * @date 2020.2.5
 * @brief 自瞄多个模块所依赖的结构
 * 最近修改：将会修改Target
 * 
*/

#ifndef AIM_DEPS_CC
#define AIM_DEPS_CC

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#define SENTRYDECISION

enum ROBOT_STATUS{
    DISTANCE_AIM    = 0,        //远程自瞄
    AUTO_AIM        = 1,        //近程自瞄
    LARGE_BUFF      = 2,        //大符打击
    SMALL_BUFF      = 3,        //小符打击
    ANTI_TOP        = 4,        //反陀螺CAM
    SWEEP           = 5         //扫描搜索(未发现敌人)
};

namespace aim_deps{

///============COLOR===============///
const cv::Scalar RED        = cv::Scalar(0, 0, 255);
const cv::Scalar ORANGE     = cv::Scalar(0, 127, 255);
const cv::Scalar YELLOW     = cv::Scalar(0, 255, 255);
const cv::Scalar GREEN      = cv::Scalar(0, 255, 0);
const cv::Scalar CYAN       = cv::Scalar(255, 255, 0);
const cv::Scalar BLUE       = cv::Scalar(255, 0, 0);
const cv::Scalar PINK       = cv::Scalar(255, 0, 255);
const cv::Scalar WHITE      = cv::Scalar(255, 255, 255);
const cv::Scalar GREY       = cv::Scalar(127, 127, 127);
const cv::Scalar BLACK      = cv::Scalar(0, 0, 0);
const cv::Scalar PURPLE     = cv::Scalar(255, 10, 100);

///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++=///
//==========================通用的预设===========================//
///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++=///
//=================最小装甲板面积==============
const double _HALF_LENGTH_SMALL     = 65.00;	
const double _HALF_LENGTH_BIG 	    = 105.00;
const double _HALF_HEIGHT           = 28.50;	

const uint8_t NO_SHOOTING           = 64;

const float MIN_ARMOR_AREA          = 40.0;
const cv::Point2f NULLPOINT2f       = cv::Point2f(0.0, 0.0);
const cv::Point3f NULLPOINT3f       = cv::Point3f(0.0, 0.0, 0.0);
const float RAD2DEG                 = 57.2958;     //(constant)(180/pi)
const float DEG2RAD                 = 0.017453;     //(constant)(pi/180)

//LightMatch.hpp 的依赖参数
const float LIGHT_PARAM1            = 10.0;
const float LIGHT_PARAM2            = 7.0;
const float LIGHT_mean              = 40.0;
const float FAILED_SCORE            = INFINITY;

enum PLATE_TYPE{
    UNKNOWN = 0,
    SMALL   = 1,
    LARGE   = 2
};

//======================相机参数=========================
const cv::Mat HERO_INTRINSIC = (cv::Mat_<double>(3, 3) << 
    1776.67168581218, 0, 720,
    0, 1778.59375346543, 540,
    0, 0, 1
);

const std::vector<float> HERO_DIST = std::vector<float>{
		-0.419212525827893, 
		0.175006995615751,
		0.00489209817799368,
		-0.00289049464268412
};

///+++++++++++++++++++++++++++++++++++++++++++++++++++++++=///
///=========================通用函数========================///
///++++++++++++++++++++++++++++++++++++++++++++++++++++++++///
/// 返回距离的平方
inline float getPointDist(const cv::Point2f &p1, const cv::Point2f &p2){
    return ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

inline void getTopCenter(const cv::RotatedRect &rect, cv::Point2f &p1, cv::Point2f &p2){
    cv::Point2f tmp_p1, tmp_p2, corners[4];                                     //找出角点
    rect.points(corners);
    float d1 = getPointDist(corners[0], corners[1]);            //0/1点距离的平方
	float d2 = getPointDist(corners[1], corners[2]);            //1/2点距离的平方
	int i0 = d1 > d2? 1 : 0;								    //长所在边第一个顶点的位置
    tmp_p1 = (corners[i0] + corners[i0 + 1]) / 2;			    //获得旋转矩形两条短边上的中点
	tmp_p2 = (corners[i0 + 2] + corners[(i0 + 3) % 4]) / 2;
    if(tmp_p1.y > tmp_p2.y){                                    //保证输出点的顺序
        p2 = (tmp_p1 + tmp_p2) / 2;    
        p1 = tmp_p2;
    }
    else{                                                       //必须是p1是处于上方的点，p2处于下方（y轴更大）
        p1 = tmp_p1;
        p2 = (tmp_p1 + tmp_p2) / 2;
    }
}

inline float getLineAngleRad(const cv::Point2f &p1, const cv::Point2f &p2){
    return atan2f(p2.x - p1.x, p2.y - p1.y);
}

inline float getLineAngle(const cv::Point2f &p1, const cv::Point2f &p2){
    return getLineAngleRad(p1, p2) * aim_deps::RAD2DEG;
}

inline cv::Point2f Rotate(const cv::Point2f &_vec, float _a){
	cv::Point2f res;
	res.x = _vec.x * cos(_a) - _vec.y * sin(_a);
	res.y = _vec.x * sin(_a) + _vec.y * cos(_a); 
	return res;
}

///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++=///
//=======================灯条和装甲板定义========================//
///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++=///

struct LightBox{
    float length;                   // 灯条长度
    float angle;                    // 灯条角度
    cv::Point2f vex[2];             // vex[0]在上, vex[1]在下
    cv::Point2f center;

    inline void extend(const float k){     // 长度乘以系数变化
        vex[0] += (k - 1.0) * (vex[0] - center);
        vex[1] += (k - 1.0) * (vex[1] - center);
        length *= k;
    }           

    inline void rotate(const float ang){              // 旋转灯条(依照坐标系逆时针)
        cv::Point2f tmp = vex[1] - center;
        tmp = Rotate(tmp, ang);           
        vex[1] = center + tmp;
        vex[0] = center - tmp;
        //printf("Original: %f, rotation: %f\n", angle, RAD2DEG * ang);
        angle -= RAD2DEG * ang;                         // 注意我们定义的角度是逆时针正，与旋转矩阵定义恰好相反
        while(std::abs(angle - 360) < std::abs(angle)){
            angle -= 360;
        }
        while(std::abs(angle + 360) < std::abs(angle)){
            angle += 360;
        }
    }
};


/// @brief 灯条的两点式线段表示
struct Light
{
    bool valid;                 //是否是有效灯条(反光灯条过滤无法完全精准判定，需要依靠装甲板匹配)
    int index;       
    LightBox box;
    int isLeft = -1;           // 用于camera_map优化中，是否为左灯条 0 为左，1为右，否则为未知
    Light(){
        valid = true;
    }

    Light(
        const cv::RotatedRect &_r,
        const cv::Point2f &_p = NULLPOINT2f,
        float len = 0.0, bool _v = true
    ):
        valid(_v)
    {
        getTopCenter(_r, box.vex[0], box.center);
        box.vex[1] = box.center * 2 - box.vex[0];
        box.length = cv::max(_r.size.height, _r.size.width);
        box.angle = aim_deps::getLineAngle(box.vex[0], box.vex[1]);
    }

    Light(
        const cv::Point2f& tp,
        const cv::Point2f& ctr,
        float len = 0.0,
        bool _v = true
    ):
        valid(_v)
    {
        box.vex[0] = tp;
        box.center = ctr;
        box.vex[1] = 2 * ctr - tp;
        box.length = len;
        box.angle = aim_deps::getLineAngle(box.vex[0], box.vex[1]);
    }
};

struct Armor
{
    //可能要删除的valid标签（只需要根据数字判断是否valid就好了）
    bool valid;
    bool Isbigarmor;
    cv::Mat r_vec;                                  //向量
    int armor_number;                              
    cv::Point3f t_vec;
    cv::Point2f vertex[4];
    Light left_light;
    Light right_light;
    Armor(){ valid = true; }                                       //default
    Armor(cv::Point2f _pts[4], int _num, Light _l, Light _r, bool _big = false):
        valid(true), Isbigarmor(_big), armor_number(_num), left_light(_l), right_light(_r)
    {
        for(int i = 0; i < 4; ++i) vertex[i]=_pts[i];							//copy by points
    }

    /// @brief 存在共灯条,则返回相同的灯条的下标，否则返回-1
    inline int collide(const Armor &a){
        if( left_light.index == a.left_light.index ||
            left_light.index == a.right_light.index)
            return left_light.index;
        if( right_light.index == a.left_light.index ||
            right_light.index == a.right_light.index)
            return right_light.index;
        return -1;
    }
};

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////参数集合///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
struct Light_Params{
    //============敌方红色=====================================//
    int red_thresh_low   = 84;           //二值图threshold下阈值
    int red_thresh_high  = 250;          //二值图threshold上阈值 
    int red_exp_short    = 140;          //曝光时间(短曝光)
    int red_exp_long     = 7000;         //曝光时间(长曝光)
    int red_r_balance    = 1957;         //白平衡（红色通道）
    int red_b_balance    = 3600;         //白平衡（蓝色通道）
    int red_reflection   = 160;          //判断是否为反光灯条
    int red_filter       = 50;           //红色杂灯条过滤
    int red_green        = 90;           //红绿通道差异
    int red_reflect_min  = 170;          //红色模式下，正常灯条的内部最大红色通道值的最小允许值
    //============敌方蓝====================================//
    int blue_thresh_low  = 92;           //二值图threshold下阈值
    int blue_thresh_high = 250;          //二值图threshold上阈值
    int blue_exp_short   = 140;          //曝光时间(短曝光)
    int blue_exp_long    = 7000;         //曝光时间(长曝光) // 暂时修改
    int blue_r_balance   = 3600;         //白平衡（红色通道）
    int blue_b_balance   = 1500;         //白平衡（蓝色通道）
    int blue_reflection  = 210;          //判断是否为反光灯条
    int blue_filter      = 20;           //蓝色灯更亮
    int blue_green       = 5;            //蓝绿通道差异
    int blue_reflect_min = 200;          //蓝色模式下，正常灯条的内部最大蓝色通道值的最小允许值
    Light_Params(){}
    Light_Params(bool blue_flag, int thresh_low, int thresh_high, int exp_short,
        int exp_long, int r_balance, int b_balance, int reflection, int ch_diff, int filter)
    {
        set(blue_flag, thresh_low, thresh_high, exp_short, exp_long, r_balance, b_balance, reflection, ch_diff, filter);
    }

    void set(bool blue_flag, int thresh_low, int thresh_high, int exp_short,
        int exp_long, int r_balance, int b_balance, int reflection, int ch_diff = 5, int filter = 20
    ){
        if(blue_flag){
            blue_thresh_low     = thresh_low;
            blue_thresh_high    = thresh_high;
            blue_exp_short      = exp_short;
            blue_exp_long       = exp_long;
            blue_r_balance      = r_balance;
            blue_b_balance      = b_balance;
            blue_reflection     = reflection;
            blue_green          = ch_diff;
            blue_filter         = filter;
        }
        else{
            red_thresh_low      = thresh_low;
            red_thresh_high     = thresh_high;
            red_exp_short       = exp_short;
            red_exp_long        = exp_long;
            red_r_balance       = r_balance;
            red_b_balance       = b_balance;
            red_reflection      = reflection;
            red_green           = ch_diff;
            red_filter          = filter;
        }
    }
};
extern Light_Params light_params;

struct Distance_Params{
    const float OPS_RATIO_HEIGHT    = 16.0;         //对边宽比例        (16.0)
    const float OPS_RATIO_WIDTH     = 1.44;         //对边长比例        (1.44)
    const float NEAR_RATIO_MIN      = 20.0;         //邻边装甲板比例     (12.5)(开放大装甲板)
    const float NEAR_RATIO_MAX      = 26.0;         //中点长宽比        (30.0)            
    const float ANGLE_THRESH        = 14.0;         //角度差阈值        (13.5)
};
extern Distance_Params distance_params;
}   //namespace aim_deps
#endif //AIM_DEPS_CC