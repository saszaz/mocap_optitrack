#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"bwam_jt_track");

    ros::NodeHandle nh;

    //ros::Publisher pub_jt2 = nh.advertise<tf::StampedTransform>("/Jt_2/pose", 10);
    tf::TransformListener listener;
    tf::TransformBroadcaster br;

    ros::Rate rate(1000.0); // 1kHz

    while (nh.ok()){
      tf::StampedTransform transform;
      try{
          listener.lookupTransform("Link_1/base_link", "Base/base_link", ros::Time(0), transform);
      }
      catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.1).sleep();
            continue;
      }

      std::cout << transform.getOrigin().x() << std::endl;
      std::cout << transform.getOrigin().y() << std::endl;
      std::cout << transform.getOrigin().z() << std::endl;
      std::cout << ' ' << std::endl;

      //transform.stamp_ = ros::Time::now();
      //br.sendTransform(transform);

     }
     return 0;
};
