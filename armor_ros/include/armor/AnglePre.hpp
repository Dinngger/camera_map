/*========基于RBF内核的SVM角度预测量计算模块=======
作者：hqy
latest date of modification: 2020.1.14 00:30
问题：个人预测会有如下一个问题：
    1.计算时间过长，个人不能接受计算时间超过1ms的算法，否则就太慢了
更新：使用了绝对路径，在工控机上可以使用绝对路径（表示xml文件的位置，仅此一处）
描述：
    1.通过GetPos类的pitch,yaw4个历史值，推定下一个值的大小
    2.计算得到下一个值后，返回给GetPos或者Serial_Com类一个附加角度
    3.附加角度的存在是避免云台随动滞后|避免敌方反或刹车时的超调以及跟丢敌方
    思路：
    1.GetPos的4个历史值作为train_data，而对应的now_pitch, now_yaw作为label
    2.训练集的输入方法：
        对几个给定视频进行训练，视频中有匀速移动，变速移动以及picth/yaw协调移动内容
        云台转角不加附加角度时，每四帧输出一次pitches/yaws给本类作为train_data
        对应的now_pitch/now_yaw作为label，存入labels中
        五段视频:
            1.匀速运动（picth/yaw各一段），需要来回运动至少3.5趟，速度不一
            2.变速运动（pitch/yaw各一段），需要来回运动至少3.5趟，速度不一
            3.协调运动（一段即可），从左上到右下，右上到左下，速度可变
        个人还是担心：
            1.训练集过大导致predict计算时间过长
            2.不清楚是否要考虑装甲板位置问题：比如1m/3m/5m处训练是否有不同？
可能的优化方面：
    1.并行优化,每次计算与GetPos同时进行，用ROS再开一个node作为预测进程
    2.SVM适合于小数据量的预测和分类，不确定在这里使用好不好
    3.需要添加一个预测算法的敏感阈值：
        目标和己方在明显不动时（或者相对静止时），不需要预测角度
        角度变动微小时（阈值内），不需要预测角度，免得发生平衡位置震荡的情况
    4.分析了一下，4帧取一次数据，而帧率是60fps，则一秒取15次数据，
        则五个视频加起来，时间不超过四分钟，则能把训练数据量减小到2000个以下
        个人认为最后一个pitch或者yaw的训练量大概在1500个数据左右
        和图像识别的数据量比起来实际上小了很多，希望计算能够在1ms内，越快越好
*/

#include <iostream>
#include <vector>
#include <deque>
#include <opencv2/ml.hpp>
#include <opencv2/core.hpp>
#define DEFAULT_PITCH_PATH "/home/sentinel/ROSWorkSpace/cvProject/src/armor/xmlFiles/pitch_2.xml"
#define DEFAULT_YAW_PATH "/home/sentinel/ROSWorkSpace/cvProject/src/armor/xmlFiles/yaw_2.xml"

using namespace cv::ml;

class AnglePre{
public:
    AnglePre();                                         //参数是角度预测类的生成标示，1表示load  
    ~AnglePre();
public:
    float calcPitch(std::deque<float> pitches);                     //计算pitch预测值
    float calcYaw(std::deque<float> yaws);                          //计算yaw预测值
    void inputPitch(std::deque<float> picthes, float now_pitch);    //pitch轴训练输入
    void inputYaw(std::deque<float> yaws, float now_yaw);           //yaw轴训练输入
    void train();                                                   //进行训练   
    void load(std::string path1 = DEFAULT_PITCH_PATH, 
        std::string path2 = DEFAULT_YAW_PATH);                //载入训练模型
private:
    void save(std::string path1 = DEFAULT_PITCH_PATH,
        std::string path2 = DEFAULT_YAW_PATH);                //保存训练模型                                      
    void setUp();                                                   //debug：初始化训练模型(未训练)
private:
    cv::Ptr<SVM> pitch_m;                                           //pitch轴模型
    cv::Ptr<SVM> yaw_m;                                             //yaw轴模型
    cv::Mat yaw_label;
    cv::Mat pitch_label;
    cv::Mat yaw_data;
    cv::Mat pitch_data;
};

AnglePre::AnglePre(){
    yaw_m=SVM::create();
    pitch_m=SVM::create();
    //if(!status)
    //    setUp();
    //else
    std::cout<<"Start to load from:"<<DEFAULT_PITCH_PATH<<" AND "<<DEFAULT_YAW_PATH<<std::endl;
    load(DEFAULT_PITCH_PATH, DEFAULT_YAW_PATH);
    std::cout<<"Loaded from:"<<DEFAULT_PITCH_PATH<<" AND "<<DEFAULT_YAW_PATH<<std::endl;    
}

AnglePre::~AnglePre(){
    ;
}

void AnglePre::setUp(){
    yaw_m->setType(SVM::C_SVC);                 //还是使用n分类，不过使用RBF为内核
	yaw_m->setKernel(SVM::RBF);
	yaw_m->setDegree(0.5);
	yaw_m->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 1000, 0.01));
	yaw_m->setGamma(1);
	yaw_m->setCoef0(0);
	yaw_m->setC(1);
	yaw_m->setNu(0);
	yaw_m->setP(0);
    //=================pitch========================
    pitch_m->setType(SVM::C_SVC);                 //还是使用n分类，不过使用RBF为内核
	pitch_m->setKernel(SVM::RBF);
	pitch_m->setDegree(0.5);
	pitch_m->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 1000, 0.01));
	pitch_m->setGamma(1);
	pitch_m->setCoef0(0);
	pitch_m->setC(1);
	pitch_m->setNu(0);
	pitch_m->setP(0);
}

float AnglePre::calcYaw(std::deque<float> yaws){
    cv::Mat tmp(1, 4, CV_32F);                    //临时容器，输入训练集
    for(int i=0;i<4;++i){
        tmp.at<float>(0, i)=yaws[i];
    }
    tmp.convertTo(tmp, CV_32F);
    return yaw_m->predict(tmp.reshape(0, 1));
}

float AnglePre::calcPitch(std::deque<float> pitches){
    cv::Mat tmp(1, 4, CV_32F);                    //临时容器，输入训练集
    if(pitches.size())
    for(int i=0;i<4;++i){
        tmp.at<float>(0, i)=pitches[i];
    }
    tmp.convertTo(tmp, CV_32F);
    return pitch_m->predict(tmp.reshape(0, 1));
}

void AnglePre::inputPitch(std::deque<float> pitches, float now_pitch){
    cv::Mat tmp(4, 1, CV_32F);                    //临时容器，输入训练集
    for(int i=0;i<4;++i){
        tmp.at<float>(i)=pitches[i];
    }
    pitch_data.push_back(tmp.reshape(0, 1));
    pitch_label.push_back(int(now_pitch));
}

void AnglePre::inputYaw(std::deque<float> yaws, float now_yaw){
    cv::Mat tmp(4, 1, CV_32F);        
    yaw_data.push_back(tmp.reshape(0, 1));
    yaw_label.push_back(int(now_yaw));
}

void AnglePre::train(){
    yaw_data.convertTo(yaw_data, CV_32F);
    pitch_data.convertTo(pitch_data, CV_32F);
    //std::cout<<"Convert completed."<<std::endl;
    //std::cout<<"Yaw (row, col):"<<yaw_data.rows<<", "<<yaw_data.cols<<std::endl;
    //std::cout<<"Pitch (row, col):"<<pitch_data.rows<<", "<<pitch_data.cols<<std::endl;
    //std::cout<<"Labels row/col:"<<pitch_data.rows<<", "<<pitch_data.cols<<std::endl;
    yaw_m->train(yaw_data, ROW_SAMPLE, yaw_label);
    pitch_m->train(pitch_data, ROW_SAMPLE, pitch_label);
    //std::cout<<"Result of pitch model:"<<pitch_m->isTrained()<<std::endl;
    //std::cout<<"Result of yaw model:"<<yaw_m->isTrained()<<std::endl;
    save();
}

void AnglePre::load(std::string path1, std::string path2){
    pitch_m = SVM::load(path1);
    yaw_m = SVM::load(path2);
}

void AnglePre::save(std::string path1, std::string path2){
    pitch_m->save(path1);
    yaw_m->save(path2);
}
