
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include "std_msgs/Int64MultiArray.h"
#include <boost/thread.hpp>

namespace nodelet_test
{
  class Sub_nodelet : public nodelet::Nodelet
  {
  public:
    Sub_nodelet() :
      num(1)
    {}
  private:
    virtual void onInit()
    {
      ros::NodeHandle& nh = getNodeHandle();
      //pub = nh.advertise("test",1);
      sub = nh.subscribe("nodelet", 1, &Sub_nodelet::Callback, this);
      //ros::spin();
    }
    void Callback(const std_msgs::Int64MultiArray::ConstPtr& msg)
    {
      ROS_INFO(" %d, subsrcibe!", num++);
    }
  private:
    //ros::Publisher pub;
    ros::Subscriber sub;
    int num;
  };
}
PLUGINLIB_EXPORT_CLASS(nodelet_test::Sub_nodelet, nodelet::Nodelet)