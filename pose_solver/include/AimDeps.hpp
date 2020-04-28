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

namespace aim_deps{

///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++=///
//==========================通用的预设===========================//
///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++=///
//=================最小装甲板面积==============
const float MIN_ARMOR_AREA = 40.0;
const cv::Point2f NULLPOINT2f = cv::Point2f(0.0, 0.0);
const cv::Point3f NULLPOINT3f = cv::Point3f(0.0, 0.0, 0.0);
const float RAD2DEG = 57.2958;     //(constant)(180/pi)
const float DEG2RAD = 0.017453;     //(constant)(pi/180)

//LightMatch.hpp 的依赖参数
const float LIGHT_PARAM1 = 10.0;
const float LIGHT_PARAM2 = 6.0;
const float LIGHT_mean = 40.0;
const float FAILED_SCORE = INFINITY;

//自适应装甲板宽高比(四次多项式，降幂排列)
const float coeffs[5] = {
    0.09331009,     //0.081676,
    -1.60621302,    //-2.66139,
    9.1013089,      //32.298636,
    -22.80810291,   //-173.213,
    54.00678226,    //360.15463
};

enum PLATE_TYPE{
    UNKNOWN = 0,
    SMALL = 1,
    LARGE = 2
};

//决策所需参数
struct PnP_depended_param
{
    float Distant_base=0.03;
    float Rotation_base=30;
    float Size_base=0.1;
    int Distance_multi=1;
    int Sentry_score=1;
    int Hero_score=1;
    int Infantry_score=1;
    int Dad_score=0;
    int Base_score=10;
    int None_score=0;
};
extern PnP_depended_param pnp_depended_param;

struct Sentry_decision
{
    int blood_limit=50;
    int bullet_limit=150;
};
//装甲板类别
enum Armor_type
{
    Sentry,                         //哨兵   
    Hero,                           //英雄             
    Infantry,                       //步兵
    Dad,                            //奶爸
    Base,                           //基地
    None,                           //未知
};

///+++++++++++++++++++++++++++++++++++++++++++++++++++++++=///
///=========================通用函数========================///
///++++++++++++++++++++++++++++++++++++++++++++++++++++++++///
/// 返回距离的平方
inline float getPointDist(const cv::Point2f p1, const cv::Point2f p2){
    return ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

inline void getMidPoints(cv::RotatedRect rect, cv::Point2f &p1, cv::Point2f &p2){
    cv::Point2f tmp_p1, tmp_p2, corners[4];                                     //找出角点
    rect.points(corners);
    float d1 = getPointDist(corners[0], corners[1]);            //0/1点距离的平方
	float d2 = getPointDist(corners[1], corners[2]);            //1/2点距离的平方
	int i0 = d1 > d2? 1 : 0;								    //长所在边第一个顶点的位置
    tmp_p1 = (corners[i0] + corners[i0 + 1]) / 2;			    //获得旋转矩形两条短边上的中点
	tmp_p2 = (corners[i0 + 2] + corners[(i0 + 3) % 4]) / 2;
    if(tmp_p1.y > tmp_p2.y){                                    //保证输出点的顺序
        p2 = tmp_p1;    
        p1 = tmp_p2;
    }
    else{                                                       //必须是p1是处于上方的点，p2处于下方（y轴更大）
        p1 = tmp_p1;
        p2 = tmp_p2;
    }
}

inline float getLineAngle(const cv::Point2f p1, const cv::Point2f p2){
    return atan2f(p2.x - p1.x, p2.y - p1.y) * aim_deps::RAD2DEG;
}

inline cv::Point2f Rotate(cv::Point2f _vec, const float _a){
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
    cv::Point3f ref;                //参考点(x: 参考中心x, y:参考中心y, z:参考长度)
    cv::Point2f vex[2];             // vex[0]在上, vex[1]在下
    cv::Point2f center;
    inline void extend(const float k){     // 长度乘以系数变化
        vex[0] += (k - 1.0) * (vex[0] - center);
        vex[1] += (k - 1.0) * (vex[1] - center);
        length *= k;
    }             

    inline void copy(LightBox &lb){
        cv::Point2f direction = lb.vex[1] - lb.center;
        vex[1] = center + direction;
        vex[0] = center - direction;
        length = lb.length;
        angle = lb.angle;
    }    

    inline void even(LightBox &lb){
        float mean_len = (lb.length + length) / 2,
            mean_ang = (lb.angle + angle) / 2;
        if(length < lb.length){
            extend( mean_len / length );
        }
        else{
            lb.extend( mean_len / lb.length);
        }
        rotate(aim_deps::DEG2RAD * (mean_ang - angle) / 2);
        lb.rotate(aim_deps::DEG2RAD * (mean_ang - lb.angle) / 2);
    }

    //依据另一个灯条来重新构建灯条
    inline void rebuild(const cv::Point2f vexes[2], float ratio){        
        float r = ref.z / length;
        cv::Point2f direction = vexes[1] - vexes[0];
        if(r >= 1.2){                   //外矩形与内矩形长度相差太大，认为是中心位置有误（以外矩形为准）
            length = sqrt(getPointDist(vexes[1], vexes[0]));
            float k = 0.2 + ref.z / 4 / length;
            length *= 2 * k;                        // 参考长度/(2×另一灯条长) 与 0.4 的算术平均
            center = cv::Point2f(ref.x, ref.y);
            vex[0] = center - direction * k;
            vex[1] = center + direction * k;         // 长边的长的0.8 / 2为从center到端点的观测长度
        }
        else{                                           // 外矩形匹配也失败了，只能进行经验式的补偿
            vex[0] = center - direction * 0.8 * (1.0f - ratio);
            vex[1] = center + direction * 0.8 * ratio;         // 长边的长的0.75 / 2为从center到端点的观测长度
            center = (vex[1] + vex[0]) / 2;
            length = sqrt(getPointDist(vex[1], vex[0]));
        }
    }

    inline void rotate(const float ang){              // 旋转灯条(依照坐标系逆时针)
        cv::Point2f tmp = vex[1] - center;
        tmp = Rotate(tmp, ang);           
        vex[1] = center + tmp;
        vex[0] = center - tmp;
        //printf("Original: %f, rotation: %f\n", angle, RAD2DEG * ang);
        angle -= RAD2DEG * ang;                         // 注意我们定义的角度是逆时针正，与旋转矩阵定义恰好相反
    }
};


/// @brief 灯条的两点式线段表示
struct Light
{
    bool valid;                 //是否是有效灯条(反光灯条过滤无法完全精准判定，需要依靠装甲板匹配)
    int index;       
    LightBox box;
    Light(){
        valid = true;
    }
    Light(int _i, cv::RotatedRect _r, cv::Point2f _p = NULLPOINT2f, float len = 0.0, bool _v = true):
        valid(_v), index(_i)
    {
        getMidPoints(_r, box.vex[0], box.vex[1]);
        box.length = cv::max(_r.size.height, _r.size.width);
        box.angle = aim_deps::getLineAngle(box.vex[0], box.vex[1]);
        box.center = (box.vex[0] + box.vex[1]) / 2;
        box.ref = cv::Point3f(_p.x, _p.y, len);
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
    cv::Point2f center;                             //center of the armorplate
    Light left_light;
    Light right_light;
    Armor(){ valid = true; }                                       //default
    Armor(cv::Point2f _pts[4], int _num, Light _l, Light _r, bool _big = false):
        valid(true), Isbigarmor(_big), armor_number(_num), left_light(_l), right_light(_r)
    {
        for(int i=0; i<4;++i) vertex[i]=_pts[i];							//copy by points
        center = (_pts[0]+_pts[1]+_pts[2]+_pts[3])/4;			//calc center(maybe useless)
    }

    /// @brief 存在共灯条,则返回相同的灯条的下标，否则返回-1
    inline int collide(Armor a){
        if( left_light.index == a.left_light.index ||
            left_light.index == a.right_light.index)
            return left_light.index;
        if( right_light.index == a.left_light.index ||
            right_light.index == a.right_light.index)
            return right_light.index;
        return -1;
    }
};


//决策之后的装甲板
struct Evaluated_armor
{
    Armor _armor;                   
    Armor_type _type;
    float Distance_score;
    float Size_score;
    float Rotation_score;
    float Type_score;
    float Total_score;
};

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////参数集合///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
struct Light_Params{
    //============敌方红色=====================================//
    const int red_thresh_low    = 120;          //二值图threshold下阈值
    const int red_thresh_high   = 250;          //二值图threshold上阈值 
    const int red_exp_short     = 70;           //曝光时间(短曝光)
    const int red_exp_long      = 7000;         //曝光时间(长曝光)
    const int red_r_balance     = 0;            //白平衡（红色通道）
    const int red_b_balance     = 3600;         //白平衡（蓝色通道）
    //============敌方蓝色=====================================//
    const int blue_thresh_low   = 120;          //二值图threshold下阈值
    const int blue_thresh_high  = 250;          //二值图threshold上阈值
    const int blue_exp_short    = 70;           //曝光时间(短曝光)
    const int blue_exp_long     = 7000;         //曝光时间(长曝光)
    const int blue_r_balance     = 3600;        //白平衡（红色通道）
    const int blue_b_balance     = 0;           //白平衡（蓝色通道）
};
extern Light_Params light_params;

struct Distance_Params{
    const float OPS_RATIO_HEIGHT    = 16.0;      //对边宽比例
    const float OPS_RATIO_WIDTH     = 1.44;     //对边长比例
    const float NEAR_RATIO_MIN      = 12.5;    //邻边装甲板比例
    const float NEAR_RATIO_MAX      = 30.0;
    const float ANGLE_THRESH        = 13.5;     //角度差阈值
};
extern Distance_Params distance_params;

//储存检测装甲板的各种参数
struct Vicinity_param
{
	//预处理信息
	int brightness_threshold;    
	int color_threshold;
	float light_color_detect_extend_ratio;

	//光条本体信息
	float light_min_area;
	float light_max_angle;
	float light_min_size;
	float light_contour_min_solidity;
	float light_max_ratio;

	//光条配对信息
	float light_max_angle_diff_;		//光条最大倾斜角度
	float light_max_height_diff_ratio_; // 光条最大宽高比
	float light_max_y_diff_ratio_;		// 两光条最大y距离比
	float light_min_x_diff_ratio_;		//两光条最大x距离比

	//装甲板信息
    float	armor_big_armor_ratio ;
    float   armor_small_armor_ratio ;
	float armor_min_aspect_ratio_; //装甲宽高比
	float armor_max_aspect_ratio_;
	int enemy_color; //目标颜色
//
		float sight_offset_normalized_base ;
		float area_normalized_base ;
	//构造函数
	Vicinity_param()
	{
		brightness_threshold = 150;
		color_threshold = 100;
		light_color_detect_extend_ratio = 1.1;
		light_min_area = 10;
		light_max_angle = 45.0;
		light_min_size = 5.0;
		light_contour_min_solidity = 0.5;
		light_max_ratio = 1.0;
		light_max_angle_diff_ = 7.0;
		light_max_height_diff_ratio_ = 0.2;
		light_max_y_diff_ratio_ = 2.0;
		light_min_x_diff_ratio_ = 0.8;
	    armor_big_armor_ratio = 3.2;
		armor_small_armor_ratio = 2;
		armor_min_aspect_ratio_ = 1.0;
		armor_max_aspect_ratio_ = 5.0;
		enemy_color = 0;
	    sight_offset_normalized_base = 200;
		area_normalized_base = 1000;
	}
};
}   //namespace aim_deps
#endif //AIM_DEPS_CC