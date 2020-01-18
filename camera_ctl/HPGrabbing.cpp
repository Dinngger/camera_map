/*==============ROS高性能取流节点(c pthread 实现)==============
作者：hqy
创建日期：2020.1.15 8：55
最后修改：2020.1.16.10:29
主要内容：
    1.双线程同时取流并且解码，解码后直接以cv_bridge发送Mat，由其他需要的节点接收
    2.现在估计的取流会比解码更慢，故解码位置是由取流位置决定的
    3.目前帧数最大值：88fps
*/

#include <iostream>
#include <pthread.h>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include "../include/armor/CameraCtl.hpp"
#include <image_transport/image_transport.h>
#define FRAME_SIZE 1440*1080*2                      //一帧的像素个数
//#define DEBUG                                     //终端输出

struct ImageBuffer{
    volatile int r_tag = 1;          //1为可读，2为在读，0为无可用,volatile防止编译优化空循环
    volatile int w_tag = 1;          //1为可写，2为在写，0为无可用
    MV_FRAME_OUT frame;              //帧数据
};


ImageBuffer g_buf[2];                               //乒乓缓冲池
int g_grab = 1, g_ret;                              //grab-是否取流, ret-返回值
int g_next_pos = -1;                                //read的下一个位置
int _frame_cnt = 0;                                 //帧率测试：总帧数
double _t_start = 0.0, _t_end = 0.0, _t_sum = 0.0;  //计时模块
cv::Mat g_frame;                                    //解码后的图像  
image_transport::Publisher g_pub;                   //全局publisher


//================ROS通信相关函数===================

void g_pub_image(){
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
         "bgr8", g_frame).toImageMsg();
    g_pub.publish(msg);
}


//=================取流全局函数====================

void g_init(){
    for(int i=0; i<2; ++i){
        g_buf[i].r_tag = 1;
        g_buf[i].w_tag = 1;
        g_buf[i].frame.pBufAddr = NULL;
    }
}

//寻找可读的缓冲区位点
//超时保护机制未加入，现在使用的是超时取流：即如果没有写完或者解码完，另一个进程进行无用的循环
int g_isReadable(int pos){
    return (g_buf[pos].r_tag == 1);                              //此处可读
}

//判断缓冲区是否可以写入
//超时保护机制未加入，现在使用的是超时取流：即如果没有写完或者解码完，另一个进程进行无用的循环
bool g_isWritable(int pos){       
    #ifdef  DEBUG
        printf("Writing to [%d], isWritable:%d, isReadable:%d.\n", 
            pos, g_buf[pos].w_tag, g_buf[pos].r_tag);
    #endif
    while(g_buf[pos].w_tag != 1){;}                //无超时保护,死循环
    return true;
}

//解码函数, YUYV色彩空间转RGB,目前会出现红蓝反色的效果
void g_decoding(int pos){
    g_frame.create(1080, 1440, CV_8UC3);                            //定义g_frame格式
    cv::Mat yuyv_img(1080, 1440, CV_8UC2, g_buf[pos].frame.pBufAddr);
    cv::cvtColor(yuyv_img, g_frame, cv::COLOR_YUV2BGR_YUYV);
    #ifdef DEBUG
        printf("Decoding is a success.\n");
    #endif
}

//写入缓冲区函数
//入参pUser应该是CameraCtl的handle句柄，因为一个摄像头只能开一个句柄，所以handle必须为public
int g_write(void* pUser, int pos, int &e){
    if(g_isWritable(pos)){
        g_buf[pos].w_tag = 2;
        g_buf[pos].r_tag = 0;
        e = MV_CC_GetImageBuffer(pUser, &g_buf[pos].frame, 1000);
        if(e != MV_OK) {
            #ifdef DEBUG
                printf("Failed to get image buffer.[0x%x]\n", e);
            #endif
            return -1;
        }
        ++_frame_cnt;
        g_buf[pos].w_tag = 1;                     
        g_buf[pos].r_tag = 1;                          //对应位置可以读入
        #ifdef DEBUG
            printf("Writing succeeded. [%d].r_tag, w_tag=(%d, %d)\n",
                pos, g_buf[pos].r_tag, g_buf[pos].w_tag);
        #endif
        g_next_pos = pos;                   //可以read的位置在pos处（本次读取位置）
        return 1-pos;                                 
    }
}

//缓冲区写入函数
//pUser:camera handle, pos:写入位点的索引
void g_read(int pos){
    g_buf[pos].r_tag = 2;
    g_buf[pos].w_tag = 0;
    g_decoding(pos);
    g_buf[pos].r_tag = 0;                               //一个位置只能解码一次
    g_buf[pos].w_tag = 1;          
    g_pub_image();
    #ifdef DEBUG
        printf("Reading succeeded. [%d].r_tag, w_tag=(%d, %d)\n",
            pos, g_buf[pos].r_tag, g_buf[pos].w_tag);
    #endif
}

//写入进程主循环
void* g_writeWorkThread(void* pUser){
    pthread_detach(pthread_self());
    int next_pos = 0, pos = 0, error = MV_OK;
    _t_start = cv::getTickCount();
    while(1){
        pos = next_pos;                                     //保留本次写入位置
        next_pos = g_write(pUser, next_pos, error);         //每次写入之前会free对应g_buf
        if(next_pos == -1){
            #ifdef DEBUG
                printf("No data[0x%x].\n", error);
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
    _t_end = cv::getTickCount();
    _t_sum = (_t_end - _t_start) / cv::getTickFrequency() * 1000;
    printf("Total time: %f ms, total frames: %d, ", _t_sum, _frame_cnt);
    printf("frame rate:%f fps.\n", _frame_cnt/_t_sum * 1000);
    printf("Exiting thread: writing...\n");
    pthread_exit(0);
    return 0;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "high_perform");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    g_pub = it.advertise("high_perform", 1);
    int judge = 1, pos = 1;                     //主函数循环使用
    g_init();                                   //初始化缓冲区以及输出图像
    Camera::CameraCtl ctl;
    ctl.startGrabbing();
    pthread_t w_threadID;                       //写入取流进程ID
    g_ret = pthread_create(&w_threadID, NULL, g_writeWorkThread, ctl.handle);      //创建取流进程
    if(g_ret != 0){
        printf("Failed to create writing thread.[0x%x]", g_ret);
    }

    _t_start = cv::getTickCount();
    while(1){
        pos = g_next_pos;                       //下一次查找的缓冲位点
        judge = g_isReadable(g_next_pos);
        if(judge){
            g_read(pos);
        }
        if(!g_grab) break;
    }
    _t_end = cv::getTickCount();
    _t_sum = (_t_end - _t_start) / cv::getTickFrequency() * 1000;
    printf("Total time: %f ms, total frames: %d, ", _t_sum, _frame_cnt);
    printf("frame rate:%f fps.\n", _frame_cnt/_t_sum * 1000);
    printf("Stop Grabing...\n");
    return 0;
}