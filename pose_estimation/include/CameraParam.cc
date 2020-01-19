
/**=======Camera Parameters=====
 * author: gqr
 * create time: unknown, during the first phase of winter training.
 * description: Integration of params
 */

enum GAIN_MODE
{
    NONE = 0,
    ONCE = 1,
    CONTINUOUS = 2
};
enum BITRATE
{                           //kbps
    DEFAULT = 4096,         //默认码率4096kbps
    HIGH = 6144,
    ULTRA = 8192,
    MAXIMUM = 16383,        //Hikvision sdk能接受的最大码率
    MEDIUM = 2048,
    LOW = 1024,
    MINIMUM = 128           //最小码率
};

typedef struct CameraParam
{
    const int Width = 1440;                     //The width
    const int Height = 1080;                    //The height
    const int Fps = 166;                        //Frames per second(Maximum frame rate)
    const int nMsec = 1000;                     //Period of waiting for graph
    const float Exposure_Time = 7000;           //Period of exposure
    const GAIN_MODE Gain_Mode = CONTINUOUS;     //Gain mode
    const float Gain = 17;                      //Set gain value
    const int Auto_Exposure_Mode=2;             //Set exposure mode !!Param should be within (0,3)
    const int Gamma = 100;                      //Gamma Value 
    const int r_balance = 3600;                 //param of white balance (when enemy is blue)
    const int g_balance = 0;
    const int b_balance = 0;
}CameraParam;
