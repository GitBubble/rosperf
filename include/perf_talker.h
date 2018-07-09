#ifndef ROSPERF_TALKER_H_
#define ROSPERF_TALKER_H_

#include "ros/ros.h"
#include "sys/time.h"
#include "std_msgs/String.h"
#include "rosperf/perf.h"
#include "perf_datadef.h"

using std::string;

class Perf_Talker
{ 
public:
    Perf_Talker();  
    virtual ~Perf_Talker();

    void Init();
    void PerfAckSubCallback(const rosperf::perf perfMsg);
    void BuildMsg(rosperf::perf& sperf, uint32_t msg_cmd, uint32_t msg_Id, uint32_t msg_subId, string data);
    uint32_t CheckInputPara(int argc, char **argv);
    uint32_t TestSendMsg();

    ros::Publisher  perf_pub;
    ros::Subscriber perf_sub;
    long long buff_size;  //发包长度
    uint32_t frame_num;   //发包数
    uint32_t frequency;   //发包频率
    bool     isNeedAck;   //是否要回应
    uint32_t nodePubIdx;   //本进程中运行的节点ID(区分运行多个实例时)
};

#endif //ROSPERF_TALKER_H_

