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

#include "LOG.hpp"

using namespace rmlog;

/** ==================日志语句标准头===================*/
std::map<TYPETAG, std::string> TAGMAP = {
    {INFO, "[Info Record]"}, {CHECKED, "[[-Checked output-]]"}, {NONETAG, ""},
    {QUESTION, "[?Questionable?]"}, {ERROR, "<<---ERROR OCCURS--->>"}, 
    {HIGHLIGHT, "=====================High light=====================\n"},
};

LOG::LOG(const START_UP_MODE mode){
    std::string name = DEFUALT_PATH + getSystemTime() + TXT;        //默认路径文件，文件名为时间+.txt
    std::string head("====================The START of the log file====================\n");
    tmp_s << "Log file create time:" << getSystemTime() <<std::endl<<std::endl;
    std::string info = tmp_s.str();
    tmp_s.str(""); tmp_s.clear();
    switch(mode){
        case REAL_TIME_A:
            out.open(name, std::ios::app | std::ios::out);      //有文件就追加，无文件就创建
            out << head; out << info; break;
        case MSG_QUEUE:
            msg_que.clear();
            msg_que.emplace_back(name);                         //消息队列的第一个是文件生成位置(文件名)
            msg_que.emplace_back(head);
            msg_que.emplace_back(info); break;
        default: 
            out.open(name, std::ios::out);
            out << head; out << info; break;
    }
    this -> mode = mode;
}

LOG::LOG(const std::string path, const std::string name, const START_UP_MODE mode){
    std::string f_name;
    std::string head("====================The START of the log file====================\n");
    tmp_s << "Log file create time:" << getSystemTime() <<std::endl<<std::endl;
    std::string info = tmp_s.str();
    tmp_s.str(""); tmp_s.clear();
    const char *path_chr = path.data();                 //获取一个const char指针给access函数使用
    if(access(path_chr, W_OK) == 0){                    //R_OK表示存在这个目录, 0是可以访问，-1是不可
        f_name = path + "/" + name + TXT;
    }
    else{
        f_name = DEFUALT_PATH + getSystemTime() + TXT;
        std::cout<<"No such directory named: '"<< path <<"'\n";
    }
    switch(mode){
        case REAL_TIME_A:
            out.open(f_name, std::ios::app | std::ios::out);      //有文件就追加，无文件就创建
            out << head; out << info; break;
        case MSG_QUEUE:
            msg_que.clear();
            msg_que.emplace_back(f_name);                         //消息队列的第一个是文件生成位置(文件名)
            msg_que.emplace_back(head);
            msg_que.emplace_back(info); break;
        default: 
            out.open(f_name, std::ios::out);
            out << head; out << info; break;
    }
    this -> mode = mode;
}

LOG::~LOG(){
    if(mode == MSG_QUEUE){
        std::string tmp = (*msg_que.begin());
        out.open(tmp, std::ios::out);                       //第一个入队的是文件名
        msg_que.pop_back();
        for(size_t i = 1; i < msg_que.size(); ++i){
            out << msg_que[i];
        }
        out << "\n====================The end of the log file====================\n";
        out.close(); return;
    }
    out << "\n====================The end of the log file====================\n";
    out.close();
}

std::string LOG::getSystemTime(){
    time_t tt;
    time( &tt );
    tt = tt + 8*3600;  // 变更时区
    tm* t= gmtime( &tt );
    char str[32];
    snprintf(str, 32, "[%d-%02d-%02d %02d:%02d:%02d] ",
        t->tm_year + 1900, t->tm_mon + 1,
        t->tm_mday, t->tm_hour,
        t->tm_min, t->tm_sec);
    return std::string(str);
}
