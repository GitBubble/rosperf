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
    long long buff_size;  //��������
    uint32_t frame_num;   //������
    uint32_t frequency;   //����Ƶ��
    bool     isNeedAck;   //�Ƿ�Ҫ��Ӧ
    uint32_t nodePubIdx;   //�����������еĽڵ�ID(�������ж��ʵ��ʱ)
};

#endif //ROSPERF_TALKER_H_

