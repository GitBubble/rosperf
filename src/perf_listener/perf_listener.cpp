/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rosperf/perf.h"
#include <math.h>
#include <iostream>  
#include <fstream>
#include "perf_datadef.h"
#include "perf_listener.h"
#include <thread>

using namespace std;

Perf_Listener::Perf_Listener()
{
	uNodeSubIdx = 0;
	uRecvCnt = 0;
	msg_pool.clear();
}
Perf_Listener::~Perf_Listener(){}

void Perf_Listener::Init()
{
	ros::NodeHandle n;
	uNodeSubIdx = GetNodeIndex();

	//创建话题，接收固定5000队列长度
	perf_sub = n.subscribe("perftest", 5000, &Perf_Listener::ListenerCallback, this); 

	ros::Rate wait_rate(1);
	while(!perf_sub.getNumPublishers())
	{
		//说明当前无订阅，发送的消息无接收，等待
		cout << "NO Publishers waitting...." << endl;
		wait_rate.sleep();
	}

	cout << "Init Ok! PubNum:" << perf_sub.getNumPublishers() << "uNodeSubIdx:" << uNodeSubIdx << endl;
	//需要有回应,创建ACK,默认创建
	perf_pub = n.advertise<rosperf::perf>("perftest_ack", 5000);
	
	return;
}

void Perf_Listener::ListenerCallback(const rosperf::perf perfmsg)
{
	//1.无ACK的本地进行解析，判断消息是否发给自身，非自身消息不接收
	if(uNodeSubIdx != perfmsg.msg_subId)
	{
		//PERF_DEBUG(DEBUG_LEVEL_INFO, ("Nothing to do,uNodeSubIdx:%d, msg_subId:%s. ", uNodeSubIdx, perfmsg.msg_subId ));
		return;
	}
	
	//PERF_DEBUG(DEBUG_LEVEL_INFO, ("ListenerCallback,uNodeSubIdx:%d, msg_subId:%s.", uNodeSubIdx, perfmsg.msg_subId));
	//2.解析命令，如有ACK的直接原消息返回
	if(perfmsg.msg_cmd == ROS_MSG_CMD_ACK || perfmsg.msg_cmd == ROS_MSG_CMD_ACKEND)
	{
		perf_pub.publish(perfmsg);
		uRecvCnt++;
		//cout << "ListenerCallback ACK" << uRecvCnt << endl;
		//PERF_DEBUG(DEBUG_LEVEL_INFO, ("ListenerCallback ACK,%d.\n", uRecvCnt));
		return;
	}

	//3.解析对应的时间
	if(perfmsg.msg_cmd != ROS_MSG_CMD_NOREND)
	{
		Add_Time(ROS_MSG_CMD_NOREND, perfmsg.time);
	}
	else
	{
		//最后一帧消息打印结果
		Print_Result(); 
		//ros::shutdown(); //收到最后一帧，退出
	}
}

uint32_t Perf_Listener::CheckPaInputPara(int argc, char **argv)
{
	for (int i=1;i<argc;i++)
	{
	 	cout<< "inPut para"<<"["<<i<<"]" << argv[i] <<endl;
	}

	debug_switch = strtol(argv[1],NULL,10);
    file_switch  = strtol(argv[2],NULL,10);

	return 0;
}

/*
参数说明:
1、调试开关，控制相关打印是否输出:1:输出；0:不输出;
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "perf_listener");
  
  Perf_Listener mPerfLisen;
  (void)mPerfLisen.CheckPaInputPara(argc, argv);
  mPerfLisen.Init();

  ros::spin();

  return 0;
}
