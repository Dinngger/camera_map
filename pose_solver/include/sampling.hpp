#ifndef __SAMPLING_HPP
#define __SAMPLING_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>

///TODO: 改为对多个通道取平均
/**
 * @brief 判断本帧是否是低曝光/ @author Dingger
 * @param src/ @param enemy_blue 敌人是否是蓝色，默认为真
 * @return 如果本帧低曝光，返回真
 */
bool isLowExposure(const cv::Mat &src, const bool enemy_blue = true){
	cv::Scalar mean_values;
	cv::Mat channels[3];
	cv::split(src, channels);
    cv::Mat gray;
	if(enemy_blue) gray = channels[0];											//取出蓝色通道
    else gray = channels[2];                                                    //取出红色通道
    int row_sampling = 16;
    int col_sampling = 16;
    int sampling_sum = 0;
    for(int i=0;i<row_sampling;i++){
        for(int j=0;j<col_sampling;j++){
            int pix_value = gray.at<uchar>(gray.rows*0.1+i*gray.rows*0.8/row_sampling, gray.cols*0.1+j*gray.cols*0.8/col_sampling);
            if(pix_value>30) sampling_sum++;
            // cv::Point2f smp = cv::Point2f(gray.cols*0.1+i*gray.cols*0.8/col_sampling, gray.rows*0.1+j*gray.rows*0.8/row_sampling);
            // cv::circle(src, smp, 3, cv::Scalar(0, 0, 255), -1);
        }
    }
    if(sampling_sum < row_sampling*col_sampling*0.3)
	    return true;
    else
        return false;
}

#endif //__SAMPLING_HPP