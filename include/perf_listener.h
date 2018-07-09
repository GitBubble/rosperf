#ifndef ROSPERF_LISTENER_H_
#define ROSPERF_LISTENER_H_
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rosperf/perf.h"
#include <thread>
#include "perf_datadef.h"

class Perf_Listener
{ 
public:
    Perf_Listener();
    virtual ~Perf_Listener();
    
    void Init();
    void ListenerCallback(const rosperf::perf perfmsg);
    uint32_t CheckPaInputPara(int argc, char **argv);

    ros::Publisher  perf_pub;
    ros::Subscriber perf_sub;
    uint32_t uNodeSubIdx;
    int uRecvCnt;
};

#endif

