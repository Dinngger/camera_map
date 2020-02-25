/*==============ROS高性能取流节点(c pthread 实现)==============
作者：hqy
创建日期：2020.1.15 8：55
最后修改：2020.1.17.19:36
主要内容：
    1.双线程同时取流并且解码，解码后直接以cv_bridge发送Mat，由其他需要的节点接收
    2.现在估计的取流会比解码更慢，故解码位置是由取流位置决定的
    3.目前帧数最大值：88fps
    处理思路：
        两个线程，不对外发送图像，图像解码后立即进入处理，读取、解码、处理、发送信息给serial_com是同一线程
        第一个线程用于取流，写入，第二个进程的循环是用subscirber驱动的，只有接收到gimbal的位置
        才能在下一次装甲板检测，判断云台位置时用上本次云台位置
    目前暂留<cv_bridge>以及<image_transport>以备图像处理所花时间过长
    可能出现的情况：图像解码+图像处理，灯条匹配花太长时间,导致帧率无法达到88fps,期望不能低于80fps

TODO:寒假期间将此节点的函数进行封装
*/

#include <iostream>
#include <pthread.h>
#include <ros/ros.h>
#include <opencv2/core.hpp>
//#include <cv_bridge/cv_bridge.h>
#include "../distance/LightMatch.hpp"
#include "../distance/ArmorPlate.hpp" 
#include "../include/universal/CameraCtl.hpp"
#include "../../serial_com/include/serial_com/parameters.h"
#//include <image_transport/image_transport.h>
#define NULL_POS -1
#define DARK
#ifndef DEBUG
    #define print(...)                                //空串替换
#else
    #define print printf
#endif

enum STATUS{
    BUSY = 0,           //繁忙，不支持读写
    AVAIILABLE = 1,     //支持读写
};

struct ImageBuffer{
    volatile int r_tag = BUSY;          
    volatile int w_tag = AVAIILABLE;          
    MV_FRAME_OUT frame;                 //帧数据
};


ImageBuffer g_buf[2];                               //乒乓缓冲池
int g_grab = 1, g_ret;                              //grab-是否取流, ret-返回值
int g_next_pos = 1;                                 //read的下一个位置
cv::Mat g_frame;                                    //解码后的图像  
//image_transport::Publisher g_pub;                   //全局publisher


cm::CameraCtl ctl;
ArmorPlate amp;
LightMatch match;
char str[20], key = 0;
bool record_judge = false;
int t_start = 0, t_end = 0, t_sum = 0, test_count = 0;


//================ROS通信相关函数===================
/*
void pub_image(){
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
         "bgr8", g_frame).toImageMsg();
    g_pub.publish(msg);
}*/


//=================取流全局函数====================

void init(){
    for(int i=0; i<2; ++i){
        g_buf[i].r_tag = BUSY;
        g_buf[i].w_tag = AVAIILABLE;
        g_buf[i].frame.pBufAddr = NULL;
    }
}

//寻找可读的缓冲区位点
//超时保护机制未加入，现在使用的是超时取流：即如果没有写完或者解码完，另一个进程进行无用的循环
int isReadable(int pos){
    return (g_buf[pos].r_tag == AVAIILABLE);                        //此处可读
}

//判断缓冲区是否可以写入
//超时保护机制未加入，现在使用的是超时取流：即如果没有写完或者解码完，另一个进程进行无用的循环
bool isWritable(int pos){       
    print("Writing to [%d], isWritable:%d, isReadable:%d.\n", 
        pos, g_buf[pos].w_tag, g_buf[pos].r_tag);
    while(g_buf[pos].w_tag == BUSY){;}                              //无超时保护,死循环
    return true;
}

//解码函数, YUYV色彩空间转RGB,目前会出现红蓝反色的效果
void decoding(int pos){
    g_frame.create(1080, 1440, CV_8UC3);                            //定义g_frame格式
    cv::Mat yuyv_img(1080, 1440, CV_8UC2, g_buf[pos].frame.pBufAddr);
    cv::cvtColor(yuyv_img, g_frame, cv::COLOR_YUV2BGR_YUYV);
    print("Decoding is a success.\n");
}

//写入缓冲区函数
//入参pUser应该是CameraCtl的handle句柄，因为一个摄像头只能开一个句柄，所以handle必须为public
int g_write(void* pUser, int pos, int &e){
    if(isWritable(pos)){
        g_buf[pos].w_tag = BUSY;
        g_buf[pos].r_tag = BUSY;
        e = MV_CC_GetImageBuffer(pUser, &g_buf[pos].frame, 1000);
        if(e != MV_OK) {
            print("Failed to get image buffer.[0x%x]\n", e);
            return NULL_POS;
        }

        g_buf[pos].w_tag = BUSY;                     
        g_buf[pos].r_tag = AVAIILABLE;                 //对应位置可以读入
        print("Writing succeeded. [%d].r_tag, w_tag=(%d, %d)\n",
            pos, g_buf[pos].r_tag, g_buf[pos].w_tag);
        g_next_pos = pos;                   //可以read的位置在pos处（本次读取位置）
        return 1-pos;                                 
    }
}

//图像处理--------------解码后不使用cv_bridge,直接发送，虽然可能造成阻塞
void process(cv::Mat frame){
    cv::Mat proced;
    t_start = cv::getTickCount();
	if(frame.empty()) return;
	ctl.setExposureTime( match.low_exposure ? 7000 : 140);		
	if(match.lowExposure(frame, proced)){
		match.findPossible(proced);
		#ifdef DARK
			match.drawLights(frame);							//绘制所有灯条
			amp.matchAll(frame, match.matches, match.possibles);//查找匹配灯条
			amp.drawArmorPlates(frame);							//绘制装甲板
			t_end = cv::getTickCount();
			if (test_count)t_sum += (t_end - t_start) / cv::getTickFrequency() * 1000;
			test_count++;
			cv::imshow("disp", frame);
			key=cv::waitKey(1);
			if(key==27) {
                g_grab = false;
                return;
            }
			else if(key==' ') cv::waitKey(0);
		#endif
	}
	#ifndef DARK
	else{
		match.drawLights(frame);							//绘制所有灯条
		amp.matchAll(frame, match.matches, match.possibles);//查找匹配灯条
		amp.drawArmorPlates(frame);							//绘制装甲板
		#ifdef RECORD										//视频帧保存
			if(record_judge) {
				outputVideo<<frame;
				cv::circle(frame, cv::Point(20, 20), 10, cv::Scalar(0, 0, 255), -1);	//录像标识
			}
		#endif

		t_end = cv::getTickCount();
		if (test_count)t_sum += (t_end - t_start) / cv::getTickFrequency() * 1000;
		test_count++;
		cv::imshow("disp", frame);
		key=cv::waitKey(1);
		if(key==27) {
            g_grab = false;
            return;
        }
		
		#ifdef RECORD
			else if(key=='e') record_judge =! record_judge;
		#endif
		
		else if(key==' ') cv::waitKey(0);
	}
	#endif
	t_end = cv::getTickCount();
	if (test_count)t_sum += (t_end - t_start) / cv::getTickFrequency() * 1000;
	test_count++;
}

//缓冲区写入函数
//pUser:camera handle, pos:写入位点的索引
void g_read(int pos){
    g_buf[pos].r_tag = BUSY;
    g_buf[pos].w_tag = BUSY;
    decoding(pos);
    process(g_frame);
    //pub_image();
    g_buf[pos].r_tag = BUSY;                               //一个位置只能解码一次
    g_buf[pos].w_tag = AVAIILABLE;          
    print("Reading succeeded. [%d].r_tag, w_tag=(%d, %d)\n",
        pos, g_buf[pos].r_tag, g_buf[pos].w_tag);
}

//写入进程主循环
void* writeWorkThread(void* pUser){
    pthread_detach(pthread_self());
    int next_pos = 0, pos = 0, error = MV_OK;
    while(1){
        pos = next_pos;                                     //保留本次写入位置
        next_pos = g_write(pUser, next_pos, error);         //每次写入之前会free对应g_buf
        if(next_pos == NULL_POS){
            #ifdef DEBUG
                print("No data[0x%x].\n", error);
                next_pos = pos;                             //如果写入失败，就尝试一直在此位置写入
            #endif
        }
        if(g_buf[pos].frame.pBufAddr != NULL){              //先释放存在已经写入数据的位置
            error = MV_CC_FreeImageBuffer(pUser, &g_buf[pos].frame);
            
            #ifdef DEBUG
                if(error != MV_OK){
                    printf("Freeing previous frame failed[0x%x].\n", error);
                }
                printf("Freed one spot.\n");
            #endif

        }
        if(!g_grab) break;
    }
    printf("Exiting thread: writing...\n");
    pthread_exit(0);
    return 0;
}

void subCallBack(const serial_com::parameters::ConstPtr &msg){
    //把取到的云台板数据写给ArmorPlate的GetPos类中的pitch/yaw；
    amp.pos_getter.now_pitch = msg->pitch;
    amp.pos_getter.now_yaw = msg->yaw;
    std::cout<<"Incoming gimbal position..."<<std::endl;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "HPGrabbing");
    ros::NodeHandle nh;
    //image_transport::ImageTransport it(nh);
    //g_pub = it.advertise("high_perform", 1);  //使用CV_BRIDGE对外发送图像信息 
    amp.init(nh);
    init();                                     //初始化缓冲区以及输出图像
    int judge = 1, pos = 1;                     //主函数循环使用
    ctl.startGrabbing();
    pthread_t w_threadID;                       //写入取流进程ID
    g_ret = pthread_create(&w_threadID, NULL, writeWorkThread, ctl.handle);      //创建取流进程
    if(g_ret != 0){
        printf("Failed to create writing thread.[0x%x]", g_ret);
    }
    ros::Subscriber para_sub = nh.subscribe("gimbalData", 1000, subCallBack);   //从serial_com处获取云台位置

    while(ros::ok()){
        judge = isReadable(g_next_pos);
        if(judge) g_read(g_next_pos);
        else{
            judge = isReadable(1-g_next_pos);
            if(judge) g_read(1-g_next_pos);
        }
        if(!g_grab) break;
        ros::spinOnce();
    }

    /*
    while(1){
        //pos = g_next_pos;                       //下一次查找的缓冲位点
        judge = isReadable(g_next_pos);
        if(judge) g_read(g_next_pos);
        else{
            judge = isReadable(1-g_next_pos);
            if(judge) g_read(1-g_next_pos);
        }
        if(!g_grab) break;
    }*/
    
    printf("Stop Grabing...\n");
    return 0;
}