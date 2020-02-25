#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/ml.hpp"
#include <iostream>

using namespace cv;
using namespace std;
using namespace cv::ml;

int main(int argc, char** argv)
{


    //生产数据
    //训练字符每类数量
    const int sample_mun_perclass = 300;
    //训练字符类数
    const int class_mun = 2;
    const int image_cols = 50;
    const int image_rows = 50;
    cv::Mat sample;
    cv::Mat label(cv::Size(0, 0), CV_32SC1); // 注意这里和2.x区别，3.x必须使用CV_32S，原因后面会说
    cv::Mat mean;
    for(int i =0;i<class_mun;i++)
    {
        for(int j=1;j<sample_mun_perclass;j++)
        {
            stringstream filename;
            filename<<"./num/"<<i<<"/ ("<<j<<").png";
            cv::Mat &&img = imread(filename.str(), cv::IMREAD_GRAYSCALE);
            if (img.dims > 0) {
                // 归一化, 一般认为能加快参数优化时的收敛速度、平衡模型权重等
                resize(img,img,Size(image_cols,image_rows),(0,0),(0,0),INTER_AREA);
                cv::normalize(img, img, 1.0, 0.0, cv::NormTypes::NORM_MINMAX, CV_32FC1);
                // 但如果你的数据量级本身相差不大，也可以不归一化直接convertTo即可
                img.convertTo(img, CV_32FC1);
                sample.push_back(img.reshape(0, 1));
                mean.push_back(cv::mean(img.reshape(0,1),noArray()));
                label.push_back<int>(i); // 注意push_back有模版重载，可能意外改变Mat类型
            }//if
        } 
    }
    Mat currency(3,image_rows*image_cols,CV_32SC1);
    for(int element=1;element<10;element++)
    {
    Ptr<KNearest> knn = KNearest::create();
	knn->setDefaultK(5);
	knn->setIsClassifier(true);
    PCA pca(sample,noArray(),PCA::DATA_AS_ROW,element);
    Mat eigenvalues=pca.eigenvalues;
    Mat eigenvectors=pca.eigenvectors;
    Mat processed_image=eigenvectors*(sample.t());
    cout<<"processedimage's rows and cols"<<processed_image.rows<<processed_image.cols<<endl;
    processed_image=processed_image.t();
    cout<<"processedimage's rows and cols"<<processed_image.rows<<processed_image.cols<<endl;
    cout<<"start train ......"<<endl;
    cv::Ptr<cv::ml::TrainData> &&trainDataSet = cv::ml::TrainData::create(processed_image, cv::ml::ROW_SAMPLE, label);
    knn->train(trainDataSet);
    cout<<"end"<<endl;
    int x =0,y = 0;
    double t=(double)getTickCount();
    for(int num=0;num<class_mun;num++){
        for(int i =1;i<sample_mun_perclass;i++){
            stringstream ss;
            ss<<"./num/"<<num<<"/ ("<<i<<").png";
            cv::Mat &&tk = cv::imread(ss.str(), cv::IMREAD_GRAYSCALE);
            resize(tk,tk,Size(image_cols,image_rows),(0,0),(0,0),INTER_AREA);
            cv::normalize(tk, tk, 1., 0., cv::NormTypes::NORM_MINMAX, CV_32FC1);
            Mat Projection=eigenvectors*tk.reshape(0,1).t();
            //cout<<sum(Projection)<<endl;//通过这里取阈值
            //int distance=norm(Projection,eigenvectors);
            //tk.convertTo(tk, CV_32FC1);
            float r  = knn->predict(pca.project(tk.reshape(0,1)));
            
            //cout<<"result:"<<r<<endl;
            if(((int)r) == num){
                x++;
            }else{
                y++;
            }
        }
        }
        float Accuracy_rate=(int)(x*100/(x+y));
        t=double((getTickCount()-t)/getTickFrequency());//测定结束时间
        cout<<"当降维至"<<element<<"维时 ,准确率为"<<Accuracy_rate<<"时间为"<<(int)(t*1000)<<endl;
        currency.row(0).at<int>(element-1)=element;
        currency.row(1).at<int>(element-1)=Accuracy_rate;
        currency.row(2).at<int>(element-1)=(int)(t*1000);
        knn.reset();//重设knn模型
    }
    double min_time;
    double max_time;
    double min_accurancy;
    double max_accurancy;
    Point min_time_p;
    Point max_time_p;
    Point min_accurancy_p;
    Point max_accurancy_p;
    minMaxLoc(currency.row(2),&min_time,&max_time,&min_time_p,&max_time_p,noArray());
    minMaxLoc(currency.row(1),&min_accurancy,&max_accurancy,&min_accurancy_p,&max_accurancy_p,noArray());
    cout<<"时间最小值："<<min_time_p<<"准确率最大值："<<max_accurancy_p<<endl;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    int element=6;//这里的element需要自己写
    PCA pca(sample,noArray(),PCA::DATA_AS_ROW,6);
    FileStorage fs("pca.xml",FileStorage::WRITE);
    pca.write(fs);
    fs.release();
    /*读取方法
   1    FileStorage fs("pca.xml",FileStorage::READ); 
    2   cv::PCA pca;
    3   pca.read(fs.root());
     4   fs.release();*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Ptr<KNearest> knn = KNearest::create();
	knn->setDefaultK(5);
	knn->setIsClassifier(true);
    //PCA pca(sample,noArray(),PCA::DATA_AS_ROW,element);
    Mat eigenvalues=pca.eigenvalues;
    Mat eigenvectors=pca.eigenvectors;
    Mat processed_image=eigenvectors*(sample.t());
    cout<<"processedimage's rows and cols"<<processed_image.rows<<processed_image.cols<<endl;
    processed_image=processed_image.t();
    cout<<"processedimage's rows and cols"<<processed_image.rows<<processed_image.cols<<endl;
    cout<<"start train ......"<<endl;
    cv::Ptr<cv::ml::TrainData> &&trainDataSet = cv::ml::TrainData::create(processed_image, cv::ml::ROW_SAMPLE, label);
    knn->train(trainDataSet);
    knn->save("./knn.xml");
    /*读取方法
    knn->load("your xml path");
    */
}
