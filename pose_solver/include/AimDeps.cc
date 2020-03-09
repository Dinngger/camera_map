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

//=================通用的预设===============
const cv::Point2f NULLPOINT2f = cv::Point2f(0.0, 0.0);
const cv::Point3f NULLPOINT3f = cv::Point3f(0.0, 0.0, 0.0);
const float RAD2DEG = 57.2958;     //(constant)(180/pi)

//LightMatch.hpp 的依赖参数
const float LIGHT_PARAM1 = 8.0;
const float LIGHT_PARAM2 = 4.8;
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
} pnp_depended_param;
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

struct Light
{
    //index is an indicator of light, with this I can quickly decide shared lights
    int index;          
    //box has width/height/angle/center/area etc.   
    cv::RotatedRect box;
    Light(){}
    Light(int _i, cv::RotatedRect _r): index(_i), box(_r){}
};

struct Armor
{
    //可能要删除的valid标签（只需要根据数字判断是否valid就好了）
    bool valid;
    bool Isbigarmor;
    float ang_aver;                                 //平均灯条角度
    cv::Mat r_vec;                                  //向量
    int armor_number;                              
    cv::Point3f t_vec;
    cv::Point2f vertex[4];
    cv::Point2f center;                             //center of the armorplate
    Light left_light;
    Light right_light;
    Armor(){ valid = true; }                                       //default
    Armor(cv::Point2f _pts[4], int _num, Light _l, Light _r, bool _big = false):
    armor_number(_num), left_light(_l), right_light(_r), valid(true), Isbigarmor(_big)
    {
        for(int i=0; i<4;++i) vertex[i]=_pts[i];							//copy by points
        center = (_pts[0]+_pts[1]+_pts[2]+_pts[3])/4;			//calc center(maybe useless)
    }

    //do two armorplates contains shared light? IF TRUE, there must be a wrong match    
    bool operator == (Armor a){
        return (
            left_light.index == a.left_light.index  ||
            left_light.index == a.right_light.index ||
            right_light.index == a.left_light.index ||
            right_light.index == a.right_light.index
        );
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
}light_params;

struct Distance_Params{
    const float OPS_RATIO_HEIGHT    = 9.0;      //对边宽比例
    const float OPS_RATIO_WIDTH     = 1.44;     //对边长比例
    const float NEAR_RATIO_MIN      = 12.5;    //邻边装甲板比例
    const float NEAR_RATIO_MAX      = 30.0;
    const float ANGLE_THRESH        = 15.0;     //角度差阈值
}distance_params;


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