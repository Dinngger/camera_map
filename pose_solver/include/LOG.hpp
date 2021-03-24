/**===============代替ROS—STREAM的模块==============
 * @author hqy
 * @date 2021.2.13
 * @note 输出彩色 + header + 时间显示
 */
#ifndef __Log_HPP
#define __Log_HPP

#include <chrono>
#include <iostream>

#define LOG_INFO_STREAM(format) Log::LogInfo("INFO", 0, format)
#define LOG_ERROR_STREAM(format) Log::LogInfo("ERROR", 31, format) 
#define LOG_MARK_STREAM(format) Log::LogInfo("MARK", 33, format)
#define LOG_CHECK_STREAM(format) Log::LogInfo("CHECK", 32, format)
#define LOG_SHELL_STREAM(format) Log::LogInfo("SHELL", 34, format)
#define LOG_GAY_STREAM(format) Log::LogInfo("GAY ", 35, format)
#define LOG_STREAM(color, format, args...) Log::printc(color, format, args)
#define LOG_INFO(format, args...) Log::LogInfo("INFO", 0, format, args)
#define LOG_ERROR(format, args...) Log::LogInfo("ERROR", 31, format, args)
#define LOG_MARK(format, args...) Log::LogInfo("MARK", 33, format, args)
#define LOG_CHECK(format, args...) Log::LogInfo("CHECK", 32, format, args)
#define LOG_SHELL(format, args...) Log::LogInfo("SHELL", 34, format, args)
#define LOG_GAY(format, args...) Log::LogInfo("GAY ", 35, format, args)

class Log{
public:
    Log(){}
    ~Log(){}
public:
    static void LogInfo(std::string header, int color, std::string format){
        uint64_t now = std::chrono::system_clock::now().time_since_epoch().count();
        if (header.length() < 5){
            header = " " + header;
        }
        format = "[" + header + "] [" + std::to_string(now) + "] " + format;
        if (color != 0){
            format = "\033[" + std::to_string(color) + "m" + format + "\n\033[0m";
        }
        else {
            format += "\n";
        }
        std::cout << format;
    }

    template<typename... types>
    static void printc(int color, std::string format, const types&... args){
        format = std::string("\033[") + std::to_string(color) + "m" + format + std::string("\n\033[0m");
        printf(format.c_str(), args...);
    }

    template<typename... types>
    static void LogInfo(std::string header, int color, std::string format, const types&... args){
        uint64_t now = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        if (header.length() < 5){
            header = " " + header;
        }
        format = "[" + header + "] [" + std::to_string(now) + "] " + format;
        if (color != 0){
            format = "\033[" + std::to_string(color) + "m" + format + "\n\033[0m";
        }
        else {
            format += "\n";
        }
        printf(format.c_str(), args...);
    }
};

#endif //__Log_HPP