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
#include "roscpp_tutorials/perf.h"
#include <math.h>
//#include <boost/thread.hpp>
#include <thread>
using namespace std;

#define MAX_ID 200000
#define MRATIO 1000000
#define EXTRA_QUEUE_LEN 2


int64_t msg_count = 0;
uint64_t avg_time = 0;

uint64_t maxv = 0;
uint64_t minv = 0;
uint64_t  diff = 0;
uint64_t queue_length = 0;
int debug_switch = 0;
int file_switch = 0;

typedef map<uint64_t,uint64_t> MSG_MAP_T;
MSG_MAP_T msg_pool;

ofstream report;
using namespace std;

float calcdev()
{
  //ros::Rate period(0.005);
  //while ( ros::ok() && msg_pool.size() > 1 )
  //{
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
    else
    {
   	   printf("message pool is empty!");
    }	
	
    return 	sqrt(variance); 
		
    //const string nameStr = ros::this_node::getName();
	//const char* name = nameStr.c_str();
	//printf("%s-> stdev:%f ",name,sqrt(variance));
	//cout<<endl;
	
	//period.sleep();
  //}
}

void yield_report(const uint64_t& maxv, const uint64_t& minv, const uint64_t& avg_time)
{
	const string nameStr = ros::this_node::getName();
	const char* name = nameStr.c_str();

	if(!file_switch)
    {		
	  printf( "%s-> %lu   %lu  %lu   %lu  %f",name,maxv,minv,avg_time,msg_pool.size(),calcdev() );
	  cout<<endl;
	}
	else
	{
      if(report.is_open())
	  {
	     report<<nameStr<<"-> "<<" max: " << name <<" min: "<<minv<<" avg: "<<avg_time<<" received: "<<msg_pool.size()<<" dev: "<<0<<endl;
	  }  
	}
}


void chatterCallback(const roscpp_tutorials::perf message)
{  
  struct timeval end;
  gettimeofday(&end,NULL); // get time first :)
  
  if( message.id != UINT32_MAX)
  {
      ++msg_count;
      diff = (end.tv_sec * MRATIO + end.tv_usec ) - message.time;
      if (avg_time == 0) {
    	  avg_time = diff;
    	  maxv = diff;
    	  minv = diff;
      } else {
    	  avg_time = ( diff + avg_time * (msg_count - 1 )) / msg_count;
    	  if(diff > maxv ) maxv = diff;
    	  if(diff < minv ) minv = diff;
      }
      
      msg_pool.insert( pair<uint64_t,uint64_t>(msg_count,diff) );
      
      static uint32_t id = 0;
      if ((message.id - id ) > 1 && id != 0 && debug_switch && message.id != UINT32_MAX) {
    	  printf("............subscriber lost ..%u\n" ,id);
    	  printf("....current msg_id ...%u\n" ,message.id);
      }
      
      id = message.id;
    
      /* arthur remarks:
         scenario 2: packet never lost ,so we print the result after one before last*/
      if ( id > queue_length )
      {
          yield_report(maxv,minv,avg_time);
      }	  
  } 
  /*arthur remarks:
    scenario 1: when packet lost, we print the result after print message arrived.
               and we print the result twice if no packet dropped.*/
  else
  {
      yield_report(maxv,minv,avg_time);
	  report.close();
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  
  report.open("/report.txt",ios::out | ios::app | ios::binary);

  //thread devCalcThread(calcVariance);
  //devCalcThread.join();
  
  for(int i=0; i< argc ;++i)
  {
	  cout<< "parameter list as:"<<endl;
	  cout<< "para[" << i<<"]"<< argv[i]<<endl;
  }
  
  debug_switch = strtol(argv[1],NULL,10);
  queue_length = strtol(argv[2],NULL,10);
  file_switch = strtol(argv[3],NULL,10);
  
  if ( queue_length == 0 )
  {
	  queue_length = 100000;
	  cout<<"parse error,100000 will send";
  }
  
  ros::Subscriber sub = n.subscribe("chatter", queue_length+EXTRA_QUEUE_LEN, chatterCallback);

  ros::spin();

  return 0;
}
