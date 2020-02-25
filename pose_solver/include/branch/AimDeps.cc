/**=================AIM_DEPENDENCIES================
 * @author hqy/gqr
 * @date 2020.2.5
 * @brief 自瞄多个模块所依赖的结构
 * 最近修改：将会修改Target
 * 
*/


#ifndef AIM_DEPS_CC
#define AIM_DEPS_CC

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

namespace aim_deps{

const cv::Point2f NULLPOINT2f = cv::Point2f(0.0, 0.0);
const cv::Point3f NULLPOINT3f = cv::Point3f(0.0, 0.0, 0.0);

enum PLATE_TYPE{
    UNKNOWN = 0,
    SMALL = 1,
    LARGE = 2
};

//决策所需参数
struct PnP_depended_param
{
    float Distant_base=0.1;
    float Rotation_base=1;
    float Size_base=1;
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
    int armor_number;                              
    cv::Point3f t_vec;
    cv::Point3f r_vec;
    cv::Point2f vertex[4];
    cv::Point2f center;                             //center of the armorplate
    Light left_light;
    Light right_light;
    Armor(){}                                       //default
    Armor(cv::Point2f _pts[4], int _num, Light _l, Light _r):
    armor_number(_num), left_light(_l), right_light(_r), valid(true)
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
    float Total_score;
};

}   //namespace aim_deps
#endif //AIM_DEPS_CC