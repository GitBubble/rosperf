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
#include "std_msgs/String.h"
#include "rosperf/perf.h"

#include <sstream> 
#include <iostream>

#define MAX_BUFF_SIZE 1024*1024*50 
#define DEFAULT_FREQUENCY 200
#define MAX_FRAMES_NUM 100000

#define EXTRA_QUEUE_LEN 2

using namespace std; 

/** * This tutorial demonstrates simple sending of messages over the ROS system.  */ 

int main(int argc, char **argv) 
{ 
  ros::init(argc, argv, "arthur_perf_talker");
  ros::NodeHandle n;
  ros::Rate wait_loop(0.2);
  ros::Rate print_stop(0.005);
  cout<< argc<<";"<<argv[1]<<";"<<argv[2]<<endl;
  long long buff_size = 0;
  long long frame_num = 0;
  long long frequency = 0;
 
  for (int i=1;i<argc;i++)
  {
     cout<< "para"<<"["<<i<<"]" << argv[i] <<endl;
  }

  buff_size = strtol(argv[1],NULL,10);
  cout<<buff_size << " Bytes will transported"<<endl; 
  
  frame_num = strtol(argv[2],NULL,10);
  cout<<frame_num<< " frames will transported"<<endl;

  frequency = strtol(argv[3],NULL,10);
  cout<<frequency<< " is the send rate"<<endl;
 
  if (buff_size > MAX_BUFF_SIZE || frame_num > MAX_FRAMES_NUM )
  {
	cout<<" exceeded max size BUFF: "<<MAX_BUFF_SIZE<<"; or FRAMES: "<<MAX_FRAMES_NUM<<endl; 
	return 0;
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
	  frequency == DEFAULT_FREQUENCY;
	  cout<<frequency<<" is the send rate, error parsing"<<endl;
  }
  
  ros::Publisher chatter_pub = n.advertise<rosperf::perf>("chatter", frame_num+EXTRA_QUEUE_LEN);  
  rosperf::perf iperf;
  char *buf = new char[buff_size];
  memset(buf,96,buff_size);
  struct timeval start;

  wait_loop.sleep();
  int count = 0;
  ros::Rate interval(frequency); 
  
  while ( ros::ok() && count < frame_num + EXTRA_QUEUE_LEN)
  {
    iperf.id = count;
    iperf.data = buf;

    gettimeofday(&start,NULL);
    iperf.time = start.tv_sec * 1000000 + start.tv_usec;

    chatter_pub.publish(iperf);

    interval.sleep();
    //cout<<"ok sent"; 
    ++count;
  }  
  

  cout<< count << " messages published !" << endl;
  cout<< "Now ,send print message !" <<endl;
  print_stop.sleep();
  
  iperf.id = UINT32_MAX;
  iperf.data = buf;
  chatter_pub.publish(iperf);
  
  cout<< "print message send " <<endl;

  delete [] buf;  
  return 0;
}
