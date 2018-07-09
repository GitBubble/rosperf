#ifndef ROSPERF_DATADEF_H_
#define ROSPERF_DATADEF_H_

#include "ros/ros.h"
#include <stdio.h>
#include "std_msgs/String.h"
#include <map>
#include <regex>
#include <math.h>

using std::map;
using namespace std;
using std::ofstream;
using std::string;
ofstream report;


#define MAX_BUFF_SIZE 1024*1024*5  // 当前网络传输最大设置为5M
#define DEFAULT_FREQUENCY 200      // 发包频率每秒200个
#define MAX_FRAMES_NUM 10000       // 测试发包数最大为10000包
#define EXTRA_QUEUE_LEN 2

typedef enum _ROS_MSG_CMD
{
    ROS_MSG_CMD_NOR,     //正常的消息
    ROS_MSG_CMD_NOREND,  //正常消息的最后一帧
    ROS_MSG_CMD_ACK,     //可以响应
    ROS_MSG_CMD_ACKEND,  //带ACK响应的最后一帧
    ROS_MSG_CMD_MAX      
}ROS_MSG_CMD;

enum
{
    DEBUG_LEVEL_INFO,
    DEBUG_LEVEL_WARM,
    DEBUG_LEVEL_ERROR
}DEBUG_LEVEL;

typedef map<uint64_t,uint64_t> MSG_MAP_T;
MSG_MAP_T msg_pool;

typedef struct _PERF_OPT
{
    uint64_t max_time;  //最大时间
    uint64_t min_time;  //最小时间
    uint64_t avg_time;  //平均时间
    uint64_t msg_cnt;   //当前的计数

    _PERF_OPT()
    {
        memset(this, 0, sizeof(_PERF_OPT));
    }
    
    float GetMsgDev()
    {
        uint64_t diff = 0;
        uint64_t variance = 0;

        for (MSG_MAP_T::iterator it = msg_pool.begin() ; it != msg_pool.end(); ++it)
        {
            diff = (it->second - avg_time);
            variance +=  (diff*diff); 
        }
        
        if(msg_pool.size() > 1)
        {
           variance /= (msg_pool.size()-1);
        }
        return sqrt(variance); 
    }
}PERF_OPT;
PERF_OPT  msg_info;

int debug_switch = 0;
int file_switch  = 0;
#define PERF_DEBUG(_level_, str) \
{\
    if( debug_switch > _level_  ) \
        {\
            printf(str); \
        }\
}

ostream &operator<<(ostream &out, const string& str)
{
    if(debug_switch)
    {
        cout << "abcd"<<endl;
        out << str;
    }

    return out;
}

uint32_t GetNodeIndex()
{  
    int nodIndex = 0;
    try
    {
        std::string s = ros::this_node::getName();  
        std::regex e = std::regex("[^0-9]*([0-9]+).*");  
        std::smatch m;  
        if (std::regex_search(s, m, e)) 
        {    
            auto x = m[m.size() - 1];    
            std::string tmp = x.str();    
            nodIndex = std::stoi(tmp);  
        }  
    }
    catch(exception& e)
    {
        //ros::shutdown();
        //默认为0
        cout << "name must such as:talker_0 or listener_0 "<<endl;
    }
    
    return nodIndex;
}

void Add_Time(ROS_MSG_CMD msg_cmd, uint64_t oldTime)
{
    struct timeval Timeend;
    gettimeofday(&Timeend, NULL); 
    
    //如果是ACK形式，需要来回时间除以2
    uint64_t diff = (Timeend.tv_sec * 1000000 + Timeend.tv_usec ) - oldTime;
    ++msg_info.msg_cnt;

    if(debug_switch)  cout << "newtime:" << (Timeend.tv_sec * 1000000 + Timeend.tv_usec) <<" OldTime:" <<  oldTime << "Diff:" << diff << "Cnt:" << msg_info.msg_cnt << endl;

    if(msg_cmd == ROS_MSG_CMD_ACKEND) diff = diff/2;
        
    if (msg_info.avg_time == 0) msg_info.avg_time = msg_info.max_time = msg_info.min_time = diff;
    else 
    {
        msg_info.avg_time = ( diff + msg_info.avg_time * (msg_info.msg_cnt - 1 )) / msg_info.msg_cnt;
        if(diff > msg_info.max_time) msg_info.max_time = diff;
        if(diff < msg_info.min_time) msg_info.min_time = diff;
    }

    //cout << "msgcnt:" << msg_info.msg_cnt << " diff:" << diff << endl;
    msg_pool[msg_info.msg_cnt] = diff;
}

void Print_Result()
{
    const string nameStr = ros::this_node::getName();
    const char* name = nameStr.c_str();

    if(!file_switch)
    {
        PERF_DEBUG(DEBUG_LEVEL_INFO, "name -> MsgCount max_time   min_time   avg_time  stdDev\n" );
        printf( "%s-> %lu   %lu  %lu   %lu  %f\n", name, msg_info.msg_cnt, msg_info.max_time, msg_info.min_time, msg_info.avg_time, msg_info.GetMsgDev() );
    }
    else
    {
        report.open("/report.txt",ios::out | ios::app | ios::binary);
        if(report.is_open())
        {
            report<<nameStr<<"-> "<<" max: " << msg_info.max_time <<" min: "<<msg_info.min_time<<" avg: "<<msg_info.avg_time<<" received: "<< msg_info.msg_cnt <<" dev: "<<msg_info.GetMsgDev()<<endl;
        }
    }
}

#endif

