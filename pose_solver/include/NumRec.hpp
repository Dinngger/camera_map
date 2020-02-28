/**=============Number Reccognition============
 * @author GQR
 * @date 6/2/2020
 * modified version by hqy
*/

#ifndef _NUM_REC_HPP
#define _NUM_REC_HPP

#include <cstdlib>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "AimDeps.hpp"

class NumRec{
private:                                            //Params
     cv::PCA _pca_model;                            //PCA_model to recognize the Armor
     cv::Ptr<cv::ml::KNearest> knn;                 //Knn model to recognize the Armor
     cv::Mat _transformed_Img;                      //the Image of the Armor
     //===============temporary==============//
     int img_cnt;
     char str[48];
private:
    void getMidPoints(const cv::RotatedRect rect, cv::Point2f &p1, cv::Point2f &p2);

    /**
     * @biref 获得放大的装甲板点数组，放大灯条，以获得更大区域的切割图像
     * @param r1 灯条1(left_light)
     * @param r2 灯条2(right_light)
     * @param pts 装甲板点数组（输出）
     */
    void getExtendedVex(cv::RotatedRect r1, cv::RotatedRect r2, cv::Point2f pts[]);
    inline static float getPointDist(const cv::Point2f p1, const cv::Point2f p2);
public:
     NumRec();
     ~NumRec(){ knn->clear(); } 
     ///@param:all the armors waiting for evaluation
     ///@param:the whole image (COLOR_BGR2GRAY)
     void reco(Armor &Input_armor, cv::Mat grayImg, int type=0);               //recognize the number
};

NumRec::NumRec()
{
     cv::FileStorage fs("/home/sentinel/ROSWorkspace/Autoaim/src/armor/pca.xml",
        cv::FileStorage::READ);                        //get pca models
     _pca_model.read(fs.root());
     fs.release();
     knn = cv::Algorithm::load<cv::ml::KNearest>(
        "/home/sentinel/ROSWorkspace/Autoaim/src/armor/knn.xml");  
     img_cnt = 0;                                                                             //get knn models
}

void NumRec::reco(Armor &Input_armor, cv::Mat grayImg, int type){
    cv::Point2f pts[4];
    getExtendedVex(Input_armor.left_light.box, Input_armor.right_light.box, pts);
    const cv::Point2f&
    tl = pts[0],
	bl = pts[1],
	br = pts[2], 
	tr = pts[3]; 

	int width, height;
	if(type==0){
	     width = 50;
	     height = 50;
	}
	cv::Point2f src[4]{cv::Vec2f(tl), cv::Vec2f(tr), cv::Vec2f(br), cv::Vec2f(bl)};//实际矩形
	cv::Point2f dst[4]{cv::Point2f(0.0, 0.0), cv::Point2f(width, 0.0), cv::Point2f(width, height), cv::Point2f(0.0, height)};//目标矩形
	const cv::Mat perspMat = getPerspectiveTransform(src, dst);//变换
	cv::warpPerspective(grayImg, _transformed_Img, perspMat, cv::Size(width, height));//获取矫正图像
    threshold(_transformed_Img, _transformed_Img, 30, 255, cv::THRESH_BINARY);//阈值化，准备识别
    ///snprintf(str, 48, "/home/sentinel/chopped/img%d.png", ++img_cnt);
    ///imwrite(str, _transformed_Img);
	cv::resize(_transformed_Img, _transformed_Img, cv::Size(width, height), (0, 0), (0, 0), cv::INTER_AREA);//防止可能的bug
    cv::normalize(_transformed_Img, _transformed_Img, 1., 0., cv::NormTypes::NORM_MINMAX, CV_32FC1);//归一化一下
    cv::Mat Projection=_pca_model.eigenvectors*_transformed_Img.reshape(0, 1).t();//求出图片的投影向量
	if(sum(Projection)[0]>20||sum(Projection)[0]<-20){
        Input_armor.armor_number = -1;
        return;
    }//非数字
    float r  = knn->predict(_pca_model.project(_transformed_Img.reshape(0, 1)));//预测
    //////////////////
	if((int)r == 0) Input_armor.armor_number = 3;
	if((int)r == 1) Input_armor.armor_number = 5;
    else Input_armor.armor_number = -1;
}

void NumRec::getExtendedVex(
    cv::RotatedRect r1,
    cv::RotatedRect r2,
    cv::Point2f pts[]
){
    r1.size.width  *= 2.0;
    r1.size.height *= 2.0;
    r2.size.width  *= 2.0;
    r2.size.height *= 2.0;
    if(r1.center.x < r2.center.x){      //r1灯条在左侧
        getMidPoints(r1, pts[0], pts[1]);             
        getMidPoints(r2, pts[3], pts[2]);
    }
    else{
        //始终保持第一个入参是x轴坐标小的灯条
        getMidPoints(r2, pts[0], pts[1]);             
        getMidPoints(r1, pts[3], pts[2]);
    }
}

void NumRec::getMidPoints(const cv::RotatedRect rect, cv::Point2f &p1, cv::Point2f &p2){
    cv::Point2f tmp_p1, tmp_p2, corners[4];                     //找出角点
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

float NumRec::getPointDist(const cv::Point2f p1, const cv::Point2f p2){
    return ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}
#endif //_NUM_REC_HPP