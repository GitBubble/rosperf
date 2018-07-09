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
#include "ros/ros.h"
#include "sys/time.h"
#include <stdlib.h>
#include "std_msgs/String.h"
#include "rosperf/perf.h"
#include "perf_talker.h"
#include <thread>
#include <sstream> 
#include <iostream>

using namespace std; 
using std::string;

Perf_Talker::Perf_Talker()
{
	buff_size = 0;   //发包长度
    frame_num = 0;   //发包数
    frequency = 0;   //发包频率
    isNeedAck = false;   //是否要回应
    nodePubIdx = 0;   //本进程中运行的节点ID(区分运行多个实例时)
    msg_pool.clear();
    debug_switch = true;
	file_switch  = false;	
}
Perf_Talker::~Perf_Talker() {}

void Perf_Talker::Init()
{
	ros::NodeHandle n;
	nodePubIdx = GetNodeIndex();

	//创建话题
	perf_pub = n.advertise<rosperf::perf>("perftest", 5000);    //发包队列在2000左右,默认使用TCP传输

	ros::Rate wait_rate(1);
	while(!perf_pub.getNumSubscribers())
	{
		//说明当前无订阅，发送的消息无接收，等待
		cout << "NO Subscribers waitting...." << endl;
		wait_rate.sleep();
	}

	cout << "Init Ok! SubNum:" << perf_pub.getNumSubscribers() << "nodePubIdx:" << nodePubIdx << endl;
	
    if(isNeedAck)
	{
		//需要有回应,创建ACK
		perf_sub = n.subscribe("perftest_ack", 5000, &Perf_Talker::PerfAckSubCallback, this);
	}
}

uint32_t Perf_Talker::CheckInputPara(int argc, char **argv)
{
	for (int i=1;i<argc;i++)
	{
	 cout<< "inPut para"<<"["<<i<<"]" << argv[i] <<endl;
	}

	buff_size = strtol(argv[1],NULL,10);
	cout<<buff_size << " Bytes will transported"<<endl; 

	frame_num = (uint32_t)strtol(argv[2],NULL,10);
	cout<<frame_num<< " frames will transported"<<endl;

	frequency = (uint32_t)strtol(argv[3],NULL,10);
	cout<<frequency<< " is the send rate"<<endl;

	isNeedAck = (uint32_t)strtol(argv[4],NULL,10);
	cout<< " Need ack state is " << isNeedAck <<endl;

	debug_switch = (uint32_t)strtol(argv[5],NULL,10);
	cout<< " debug_switch is " << debug_switch <<endl;

	if (buff_size > MAX_BUFF_SIZE || frame_num > MAX_FRAMES_NUM )
	{
	cout<<" exceeded max size BUFF: "<<MAX_BUFF_SIZE<<"; or FRAMES: "<<MAX_FRAMES_NUM<<endl; 
	return 1;
	}

	if ( buff_size == 0 ){
	  buff_size = MAX_BUFF_SIZE;
	  cout<<buff_size << " Bytes will transported,error parsing"<<endl; 
	}

	if ( frame_num == 0){
	  frame_num = MAX_FRAMES_NUM;
	  cout<<frame_num << " frames will transported,error parsing"<<endl; 
	}

	if ( frequency == 0){
	  frequency = DEFAULT_FREQUENCY;
	  cout<<frequency<<" is the send rate, error parsing"<<endl;
	}

	return 0;
}

void Perf_Talker::BuildMsg(rosperf::perf& sperf, uint32_t msg_cmd, uint32_t msg_Id, uint32_t msg_subId, string data)
{
	struct timeval start;

	sperf.msg_cmd   = msg_cmd;
	sperf.msg_pubId = nodePubIdx;
	sperf.msg_subId = msg_subId;
	sperf.msg_Id    = msg_Id;
	sperf.data      = data;

	gettimeofday(&start,NULL);
	sperf.time      =start.tv_sec * 1000000 + start.tv_usec;
	
	return;
}

uint32_t Perf_Talker::TestSendMsg()
{
	rosperf::perf sperf;
	string strMsg;
	uint32_t uSubNum = perf_pub.getNumSubscribers();
	ros::Rate interval(frequency); 
	
	//暂时不关注消息内容，仅关注消息长度
	char *buf = new char[buff_size];
	memset(buf, 0x5A, buff_size);
	strMsg = buf;

	for(uint32_t uSendLoop = 0; ros::ok() && uSendLoop < frame_num; uSendLoop++)
	{
    	BuildMsg(sperf, isNeedAck ? ROS_MSG_CMD_ACK : ROS_MSG_CMD_NOR, uSendLoop, (uSendLoop % uSubNum), strMsg);
        perf_pub.publish(sperf);
		interval.sleep();
	}

	ros::Duration(0.5).sleep(); //等待处理完
	//发送最后一帧，通知结束
	BuildMsg(sperf, isNeedAck ? ROS_MSG_CMD_ACKEND : ROS_MSG_CMD_NOREND, 0, 0, strMsg);
    perf_pub.publish(sperf);

	const string nameStr = ros::this_node::getName();
    const char* name = nameStr.c_str();
	cout << "TestSendMsg End! frame_num: " << frame_num << " nodePubIdx:"<< nodePubIdx << " uSubNum: " << uSubNum << endl;
	cout << name <<"[len:" << buff_size << " hz:" << frequency << " num:" << frame_num << "]->" << " PubIdx:"<< nodePubIdx << " SubNum: " << uSubNum << endl;

	ros::Duration(0.5).sleep(); //等待处理完
	
	delete[] buf;
	buf = NULL;
	
	return 0;
}


static int msg_rcv_thread(void) 
{  
	cout << "MESSAGE RECIEVE THREAD CREATED" << endl;  
	ros::spin();
}

void Perf_Talker::PerfAckSubCallback(const rosperf::perf perfMsg)
{
	if(perfMsg.msg_pubId != nodePubIdx) return;

	if(perfMsg.msg_Id != msg_info.msg_cnt)
	{
		//cout << "Message Lost" << "Msgid:" << perfMsg.msg_Id << "RevCnt:" << msg_info.msg_cnt <<endl;
	}
	
	//1.解析对应的时间
	if(perfMsg.msg_cmd != ROS_MSG_CMD_ACKEND)
	{
		Add_Time(ROS_MSG_CMD_ACKEND, perfMsg.time);
	}
	else
	{
		//最后一帧消息打印结果
		Print_Result(); 
		ros::shutdown();
	}

	//PERF_DEBUG(DEBUG_LEVEL_INFO, ("msg_cmd   msg_Id    msg_subId    msg_pubId    time \n"));
	//PERF_DEBUG(DEBUG_LEVEL_INFO, ("%11u %11u %11u %11u %11u", perfMsg.msg_cmd, perfMsg.msg_Id, perfMsg.msg_subId, perfMsg.msg_pubId, perfMsg.time));
	
	return;
}

/** * This tutorial demonstrates simple sending of messages over the ROS system.  */ 
/*使用参数及方法:

参数:
1、消息长度(byte);
2、发送包数(个);
3、发送频率(包/秒);
4、响应模式:1:回应/0:不回应(default);
5、调试开关:1:打开/0:关闭;
*/
int main(int argc, char **argv) 
{ 
	ros::init(argc, argv, "perf_talker");

	Perf_Talker mPerfTalker;
	if(mPerfTalker.CheckInputPara(argc, argv))
	{
		return 0;
	}

    mPerfTalker.Init(); 
	
	std::thread rcvthrd(msg_rcv_thread);
	mPerfTalker.TestSendMsg();
	
	exit(0);
}

