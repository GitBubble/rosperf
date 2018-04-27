
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include "std_msgs/Int64MultiArray.h"
#include <boost/thread.hpp>

namespace nodelet_test
{
  class Pub_nodelet : public nodelet::Nodelet
  {
  public:
    Pub_nodelet()
    {
		freq=1;
    }
    ~Pub_nodelet()
    {
      pub_thread_->join();
    }
  private:
    virtual void onInit()
    {
      ros::NodeHandle& nh = getNodeHandle();
      pub = nh.advertise<std_msgs::Int64MultiArray>("nodelet",1);
	  pub_thread_ = new boost::thread(boost::bind(&Pub_nodelet::publish,this));
      //ros::Subscriber sub = nh.subscribe("nodelet", 1, &Sub_nodelet::Callback, this);
	  pub_thread_->join();
    }
    void publish()
    {
      std_msgs::Int64MultiArrayPtr array(new std_msgs::Int64MultiArray);
      array->data.resize(50000000);
      for(double i = 0; i < 50000000; i++)
        array -> data.push_back(std::rand());
      int num = 1;
      ros::Rate r(1);
      while(ros::ok())
      {
        ROS_INFO(" %d ,publish!", num++);
        pub.publish(array);
        ros::spinOnce();
        r.sleep();
      }
    }
    ros::Publisher pub;
    boost::thread* pub_thread_;
	int freq;
  };
}
PLUGINLIB_EXPORT_CLASS(nodelet_test::Pub_nodelet, nodelet::Nodelet)