#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <geometry_msgs/Pose.h>
#include <string>
#include <iostream>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <math.h>

using namespace std;

class jointDataHandler { 

    public: 

        jointDataHandler(const ros::NodeHandle nh)
            : node(nh),
              base_name ("Base/base_link"),
              l1_name ("Link_1/base_link"),
              l2_name ("Link_2/base_link")
            {
            // Save home position
            ros::Duration(1.0).sleep();
            getJointTf( tf_jt1_0, l1_name, base_name);
            getJointTf( tf_jt2_0, l2_name, l1_name);

            }

        ~jointDataHandler(){};
        tf::TransformListener listener;
        tf::StampedTransform tf_jt1_0, tf_jt2_0;
        tf::StampedTransform tf_jt1, tf_jt2;
        string base_name ;
        string l1_name ;
        string l2_name ;
        ros::NodeHandle node;

    public:

    void getJointTf(tf::StampedTransform &transform, string child, string parent) {
            while (1) {
            try{
                listener.lookupTransform(child, parent, ros::Time(0), transform);
                break;
            }
            catch (tf::TransformException &ex) {
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(0.1).sleep();
                    continue;
            }
            }
    }

    double getJointAngle(tf::StampedTransform &tf_0, tf::StampedTransform &tf_1) {
        /* Arg 1: world-to-parent transform
        * Arg 2: world-to-child transform
        * */
        
        Eigen::Matrix3d rot_0, rot_1, rot_rel;
        tf::matrixTFToEigen(tf_0.getBasis(), rot_0);
        tf::matrixTFToEigen(tf_1.getBasis(), rot_1);
       
        try {
            rot_rel = rot_0.inverse() * rot_1;
        }
        catch ( std::exception &ex){
            ROS_ERROR("%s",ex.what());
        }

        //Eigen::Quaterniond quat(rot_rel); // Convert Matrix to Quat
        //quat.normalize();
        //return 2 * acos(quat.coeffs().data()[0]); 

        Eigen::Vector3d ea = rot_rel.eulerAngles(0,1,2);
        return ea[0] ; 
    }

    void loop(){
        ros::Rate rate(1000.0); // 1kHz

  //      getJointTf( tf_jt1, l1_name, base_name );
  //      getJointTf( tf_jt2, l2_name, l1_name );
  // 
  //      // Test Joint 1
  //      
  //      double angle1_0 = getJointAngle(tf_jt1, tf_jt1_0);
  //
  //      while (node.ok()) {
  //          getJointTf( tf_jt1, l1_name, base_name );
  //          getJointTf( tf_jt2, l2_name, l1_name );
  //          double angle1 = getJointAngle(tf_jt1, tf_jt1_0);
  //          double delta1 = angle1 - angle1_0;
  //         
  //          if (delta1 > 0.01 || delta1 < -0.01){
  //              angle1_0 = angle1;
  //              angle1 = angle1 + delta1;
  //              std::cout << angle1 * 180.0 / 3.1415 << std::endl;
  //          }
  //      }

        // Test Joint 2
        
        double angle_0 = getJointAngle(tf_jt2, tf_jt2_0);

        while (node.ok()) {
            getJointTf( tf_jt1, l1_name, base_name );
            getJointTf( tf_jt2, l2_name, l1_name );
            double angle = getJointAngle(tf_jt2, tf_jt2_0);
            double delta = angle - angle_0;
           
            if (delta > 0.01 || delta < -0.01){
                angle_0 = angle;
                angle = angle + delta;
                std::cout << angle * 180.0 / 3.1415 << std::endl;
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"bwam_jt_track");

    ros::NodeHandle nh;

    //ros::Publisher pub_jt2 = nh.advertise<tf::StampedTransform>("/Jt_2/pose", 10);
    //tf::TransformBroadcaster br;

    jointDataHandler  jt_dh(nh); 
    
    jt_dh.loop();

      //std::cout << tf_jt1.getOrigin().z()  << std::endl;
      //std::cout << ' ' << std::endl;

      //transform.stamp_ = ros::Time::now();
      //br.sendTransform(transform);

     return 0;
};
