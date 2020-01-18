////!Achtung!
///Due to limited armors ,the data set only have 3 and 5,so we can only reocognize 3 and 5 at the time
///To funtion normally ,I 've done some trick in function Recoginze  ::if it perdicts 0 i will return 3 ,if it predicts 1 i will return 5
///We will continue expand the data set ,you can also make your own data set by the main.cpp in the folder 
///Bu Fu Jiu Gan  @XJTURM2020
#include<iostream>
#include<opencv2/opencv.hpp>
enum Armortype
{small=0,big=1};
class Num_reco
{
private:
	cv::PCA _pca_model;
    cv::Ptr<cv::ml::KNearest> knn;
	Armortype _input_armortype;
	cv::Mat _transformed_Img;
public:
	Num_reco(/* args */);
	int Recognize(std::vector<cv::Point2f>vertex, cv::Mat& grayImg,const int Armortype=small);
	~Num_reco();
};
///Create a new rocognize class
///Param void
Num_reco::Num_reco()
{
	 cv::FileStorage fs("./pca.xml",cv::FileStorage::READ); 
     _pca_model.read(fs.root());
     fs.release();
     knn->load("./knn.xml");
}
///kill knn
Num_reco::~Num_reco()
{
	knn->clear();
}
///@brief:   recognize a Armor from grayimage
///@Param:  Vertex of target Armor
///@Param:  The gray image of Armor ,as the size of vertex
///@Param:  ArmorType  default = small armor  
///@Return -1:the img  isn't a number
///@Return  int :the possible number of the vertex 
int Num_reco::Recognize(std::vector<cv::Point2f>vertex, cv::Mat& grayImg,const int Armortype=small)
{
		const cv::Point2f&
		tl = vertex[0],
		tr = vertex[1],
		br = vertex[2],
		bl = vertex[3];

	int width, height;
	if(Armortype==small){
	width = 50;
	height = 50;
	}
	else{}//Todo
	cv::Point2f src[4]{cv::Vec2f(tl), cv::Vec2f(tr),cv:: Vec2f(br), cv::Vec2f(bl)};//实际矩形
	cv::Point2f dst[4]{cv::Point2f(0.0, 0.0), cv::Point2f(width, 0.0), cv::Point2f(width, height), cv::Point2f(0.0, height)};//目标矩形
	const cv::Mat perspMat = getPerspectiveTransform(src, dst);//变换
	cv::warpPerspective(grayImg, _transformed_Img, perspMat, cv::Size(width, height));//获取矫正图像
    threshold(_transformed_Img,_transformed_Img,30,255,cv::THRESH_BINARY);//阈值化，准备识别
	cv::resize(_transformed_Img,_transformed_Img,cv::Size(width,height),(0,0),(0,0),cv::INTER_AREA);//防止可能的bug
    cv::normalize(_transformed_Img, _transformed_Img, 1., 0., cv::NormTypes::NORM_MINMAX, CV_32FC1);//归一化一下
    cv::Mat Projection=_pca_model.eigenvectors*_transformed_Img.reshape(0,1).t();//求出图片的投影向量
	if(sum(Projection)[0]>20||sum(Projection)[0]<-20){return -1;}//非数字
    float r  = knn->predict(_pca_model.project(_transformed_Img.reshape(0,1)));//预测
   //////////////////
	if((int)r==0)return 3;
	if((int)r==1)return 5;
    else return-1;
	/////////////////    Edit above part if you have a big enough dataset
    //return (int)r;    //if we have enough data ,please use this to return the num
}