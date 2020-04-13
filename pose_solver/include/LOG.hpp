/**===============简单的日志模块==============
 * @author hqy
 * @date 2020.2.13
 * @note 以文档信息保存输出
 * 示例调用：
 *   rmlog::LOG l("/home/sentinel", "test", rmlog::MSG_QUEUE);  //构造，模式是消息队列
 *   l.log("This is to test how this turns out!", rmlog::ERROR);
 *   l.log("Oh, this is working? I can not believe this.");
 *   l.log(1.243247329432);
 *   l.log(666);
 *   l.log("SHIT I CAN'T DO THIS!", rmlog::HIGHLIGHT);
 *   l.log(4, "Pitch, Yaw:", 1.233242, ", ", 12.34433);         //@overload
 * ==================请修改针对自己系统的DEFAULT——PATH（line 28）=====================
 */

#ifndef _LOG_HPP
#define _LOG_HPP

#include <map>
#include <vector>
#include <ctime>
#include <unistd.h>
#include <cstdarg>
#include <fstream>
#include <sstream>
#include <iostream>
namespace rmlog{

const std::string DEFUALT_PATH = "/home/sentinel/ROSWorkspace/Autoaim/src/serial_com/log/";
const std::string TXT = ".txt";

enum TYPETAG{
    INFO,               //普通记录
    CHECKED,            //正确的输出
    QUESTION,           //存疑的输出
    ERROR,              //错误的输出
    HIGHLIGHT,          //需显著记录的输出
    NONETAG,            //无标记语句
    ISOLATE,            //前后换行
    NOEND,              //输出不换行
    NOHEAD,             //输出没有标准头
    INSERT,             //输出没有标准头，也不换行，相当于是输出的中间部分
};

enum START_UP_MODE{
    REAL_TIME_A,            //实时写入文件（速度可能降低),以追加模式写入
    REAL_TIME_W,            //实时写入文件（速度可能降低),以覆盖模式写入
    MSG_QUEUE,            //消息队列模式（在析构时才写入文件）(速度可能提高，但是如果意外退出，将不会有日志文件)覆盖写入
};

enum COLOR{             //有待加入：带颜色的输出
    //================F_开头是字体色================//
    F_BLACK         = 30,
    F_RED           = 31,
    F_GREEN         = 32,
    F_YELLOW        = 33,
    F_BLUE          = 34,
    F_PURPLE        = 35,
    F_DARK_GREEN    = 36,
    F_WHITE         = 37,
    //=================B_ 开头是背景色==============//
    B_BLACK         = 40,
    B_RED           = 41,
    B_GREEN         = 42, 
    B_BROWN         = 43,
    B_BLUE          = 44,
    B_MAGENTA       = 45,
    B_SAPPHIRE      = 46,
    B_WHITE         = 47
};

/** ==================日志语句标准头===================*/
extern std::map<TYPETAG, std::string> TAGMAP;

class LOG{
public:
    LOG(const START_UP_MODE mode = REAL_TIME_W);                                                          //默认构造

    /**
     * @brief 构造时指定文件名与路径以及构造模式
     * @param mode 写入的模式，默认实时覆盖写入
     * @param path 写入文件的路径
     * @param name 写入文件的名称（未指定则是生成文件的时间）
     */
    LOG(const std::string path, const std::string name, const START_UP_MODE mode = REAL_TIME_W);            
    ~LOG();                                                         //析构时完成文件的读写
public:
    /**
     * @brief 任意个数参数组成日志句段输出，每个参数之间没有默认的连接词,最后一个参数不用加'\n'
     * @param val_num 指定除了val_num以外的参数个数, 必须匹配
     * @param firstArg 递归执行的第一个参数
     * @param args variadic template的参数包
     * @apiNote  默认是以INFO模式输出的
     */
    template<typename T, typename... types>
    void log(const T& firstArg, const types&... args);     

    /**
     * @brief 输入char或者string，以及输出的模式
     * @param str 需要记录的语句
     * @param tag 输出句段模式
     */
    template<typename STR>
    void log(const STR str, const TYPETAG tag = INFO); 
public:
    /**==================print彩色输出=====================*/
    template<typename T, typename... types>
    static void printc(const COLOR _c, const T& firstArg, const types&... args);     

    /** 为了防止参数包展开到最后，没有参数情况下无法输出导致编译错误的情况
     * 必须加入一个能够输出单参数的重载 */
    template<typename T>            //variadic template巨坑
    static void printc(const T STR){
        std::cout<< STR;
    }           
private:
    static std::string getSystemTime();                     //获取系统时间戳
private:
    std::vector<std::string> msg_que;                        //消息队列
    std::ofstream out;
    std::stringstream tmp_s;
    START_UP_MODE mode;                                     //模式
};

///==========================DEFINITIONS===============================///

template <typename STR>
void LOG::log(const STR str, const TYPETAG tag){
    if(tag == ISOLATE) tmp_s << std::endl;                                     //是否隔离显示
    if(tag != NOHEAD && tag != INSERT) tmp_s << TAGMAP[tag] << getSystemTime();//生成标准头
    tmp_s << str;                                                              //内容
    if(tag == ISOLATE) tmp_s << std::endl;                                     //是否隔离显示
    else if(tag == HIGHLIGHT) tmp_s << "\n=====================High light=====================";
    if(tag != NOEND && tag != INSERT) tmp_s << std::endl;   //NOEND, INSERT的输出可以使下一次内容直接在本次后输出
    if(mode == MSG_QUEUE) msg_que.emplace_back(tmp_s.str());                       //string入队
    else out << tmp_s.str();                                                    //写入文件
    tmp_s.str(""); tmp_s.clear();                                               //清空sstream
}

template<typename T, typename... types>
void LOG::printc(const COLOR _color, const T& firstArg, const types&... args){
    char tmp[9];
    if(sizeof...(args) > 0){
        snprintf(tmp, 9, "\033[%dm", _color);
        std::cout << tmp << firstArg;
        printc(_color, args...);
    }else{
        std::cout << firstArg << "\n\033[0m";
    }
}

/** 变长参数递归执行 */
template<typename T, typename... types>
void LOG::log(const T& firstArg, const types&... args){
    if(sizeof...(args) > 0){
        tmp_s << firstArg;
        log(args...);      //递归
    }
    else{
        tmp_s << firstArg << TAGMAP[INFO] << getSystemTime() << tmp_s.str() << std::endl;
        if(mode == MSG_QUEUE) msg_que.emplace_back(tmp_s.str());                   //string入队
        else out << tmp_s.str();                                                    //写入文件
        tmp_s.str("");                  //清空stringstream只能使用.str("")方法
        tmp_s.clear();
    }
}

}   //namespace log

#endif //_LOG_HPP
