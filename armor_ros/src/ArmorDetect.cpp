/**ArmorDetect 装甲板检测节点（debug）
* 装甲板检测的传统实现
* 作者：hqy
* 最后修改日期：2020.1.14. 19:40
* 问题：
*	1.#ifdef #endif等等杂糅，并且灯条匹配没有最后确定函数
*	2.灯条匹配函数（估计最后使用ArmorPlate.hpp）
*	3.没有高性能接口取流
*	目前已经有非传统灯条匹配方法，但个人认为，如果只是用灯条查找而不使用distant detect的方法
*	意义并不大
*
*/

#include <CameraCtl.hpp>
#include <ros/ros.h>
//#define DEBUG						//摄像头录像调试参数
//#define REMOTE						//远程静止单装甲板调试参数

cv::Mat frame, proced;

//防止重复引用文件		
#include <GetLight.hpp>
#include <MatchLight.hpp>
#include <Robust.hpp>
MatchLight match;
GetLight light_get;
Robust rb;

int frame_cnt = 0, judge = 0;
double t_sum = 0, t_start = 0, t_end = 0;		//时间总和，起始时间，结束时间
char key = 0;


//摄像头图像发送节点
int main(int argc, char* argv[])
{
	#ifdef DEBUG
    	//打开摄像头录制功能,有个问题，我不知道节点启动在什么位置
    	std::string outputVideoPath = "cv_output.avi";
    	cv::Size sWH = cv::Size(1440, 1080);
		cv::VideoWriter outputVideo;
		outputVideo.open(outputVideoPath, CV_FOURCC('D', 'I', 'V', 'X'), 60.0, sWH);		
		bool record_judge = false;
	#endif

    ros::init(argc, argv, "GrabOpenCV");
    ros::NodeHandle nh;
    cm::CameraCtl ctl;
    ctl.startGrabbing();
	match.init(nh);
    printf("Press any key to exit.\n");
    char key=0;
    while (ros::ok()) {
        cv::Mat frame = ctl.getOpencvMat();				//取流：需要高性能接口
		cv::resize(frame, frame, cv::Size(720, 540));		//图像大小变换 Size归一化

	    t_start = cv::getTickCount();

		#ifdef DEBUG										//录制视频
	    	if(record_judge) outputVideo<<frame;
    	#endif
		//TODO:海康摄像头能够直接调整输出图像大小吗，resize时间挺长的，约1ms
		//TODO:此处可有性能优化:对图像的遍历非常耗时间
		if (judge%3==0) {
	    	judge = 0;
	    	rb.brightInspect(frame);
		}
		light_get.brightAdjust(frame, rb.alpha, rb.beta);
		//===============遍历结束==================图像调整光线平均耗时1.4ms
		light_get.imgProcess(frame, proced);
		if (!light_get.findPossible(frame, proced)) {
	    	match.matchLights(light_get.light_list);
	    	match.filter(frame);
	    	match.showTargets(frame);
		}
		light_get.reset();

	    t_end = cv::getTickCount();
	    if (frame_cnt)t_sum += (t_end - t_start) / cv::getTickFrequency() * 1000;
	    frame_cnt++; judge++;
	    key = cv::waitKey(1);

	    #ifdef DEBUG
		    if(key=='e'){														//按下e键录像
			    record_judge =! record_judge;
		    }
		    if(record_judge)
		    cv::circle(frame, cv::Point(15, 15), 8, cv::Scalar(0, 0, 255), -1);	//录像标识(左上角小圆点)
	    #endif
		
	    cv::imshow("disp", frame);
		cv::imshow("proced", proced);
    }
    std::cout << "Average running time:" << t_sum / frame_cnt - 1 << std::endl;

	#ifdef DEBUG
        outputVideo.release();
    #endif

	cv::destroyAllWindows();
	return 0;
}
