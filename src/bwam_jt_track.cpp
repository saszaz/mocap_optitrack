#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <geometry_msgs/Pose.h>
#include <string>
#include <iostream>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <math.h>
#include <cmath>

using namespace std;

const int DOF=4;
/* Ids to use for Joint angle determination */
// TF 
const vector<int> tfIds={0,0,0,1};
// Euler angle
const vector<int> eaIds={2,0,1,1};
// Rigid bodies 
const vector<string> rgIds={"Base/base_link",
                           "Link_1/base_link",
                           "Link_2/base_link"};

class jointDataHandler { 

    public: 

        jointDataHandler(const ros::NodeHandle nh)
            : node(nh),
              _DOF(DOF),
              _tfIds(tfIds),
              _eaIds(eaIds),
              _rgIds(rgIds),
              _numTF(rgIds.size()-1)
            {
                tfVec_0.resize(_numTF);
                tfVec.resize(_numTF);

                // Save home position
                ros::Duration(1.0).sleep();
                getJointTfs (&tfVec_0);
                
            }

        ~jointDataHandler(){};

    private:

        const int _DOF;
        const int _numTF;
        const vector<int> _tfIds, _eaIds;
        const vector<string> _rgIds;

    public:

        tf::TransformListener listener;
        vector <tf::StampedTransform> tfVec_0, tfVec;
        const string base_name ;
        const string l1_name ;
        const string l2_name ;
        ros::NodeHandle node;

//////////////////////////
    void getJointTf ( tf::StampedTransform * transform, string child, string parent ) {

         while (1) {
            try{
                listener.lookupTransform(child, parent, ros::Time(0),  * transform);
                break;
            }
            catch (tf::TransformException &ex) {
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(0.1).sleep();
                    continue;
            }
         }
    }
//////////////////////////
    void getJointTfs ( vector <tf::StampedTransform> * tf_vec){
        for (int i=0; i<_numTF; i++) {
            tf::StampedTransform transform;
            getJointTf( &transform, rgIds[i+1], rgIds[i]);
            //getJointTf( &transform, rgIds[i+1], rgIds[0]);
            (* tf_vec)[i] = transform;
        }
    }
//////////////////////////
    void getJointAngles ( const vector <tf::StampedTransform> * tf_vec_0,
                          const vector <tf::StampedTransform> * tf_vec_1, 
                          vector <float> * jp_vec ) {
          
        //Eigen::Matrix3d rot_0, rot_1, rot_rel;
        tf::Matrix3x3 rot_0, rot_1, rot_rel;
        for (int j=0; j<_DOF; j++) {
            int tf_id = _tfIds[j];
            int ea_id = _eaIds[j];
            //tf::matrixTFToEigen( (*tf_vec_0)[tf_id].getBasis(), rot_0);
            //tf::matrixTFToEigen( (*tf_vec_1)[tf_id].getBasis(), rot_1);
            
            rot_0 = (*tf_vec_0)[tf_id].getBasis();
            rot_1 = (*tf_vec_1)[tf_id].getBasis();

            try {
                rot_rel = rot_0.inverse() * rot_1;
            }
            catch ( std::exception &ex){
                ROS_ERROR("%s",ex.what());
            }

            //Eigen::Vector3d ea = rot_rel.eulerAngles(0,1,2);
            //(*jp_vec)[j] = ea[ea_id];

            double yaw, pitch, roll;
            rot_rel.getRPY(roll,pitch,yaw);
            vector <float> ea (3);
            ea[0]=roll; ea[1]=pitch; ea[2]=yaw;
            (*jp_vec)[j] = ea[ea_id];
        }

    }

       // Eigen::Matrix3d rot_0, rot_1, rot_rel;
       // tf::matrixTFToEigen(tf_0.getBasis(), rot_0);
    //tf::matrixTFToEigen(tf_1.getBasis(), rot_1);
       
    //    try {
    //        rot_rel = rot_0.inverse() * rot_1;
    //    }
    //    catch ( std::exception &ex){
    //        ROS_ERROR("%s",ex.what());
    //    }

        //Eigen::Quaterniond quat(rot_rel); // Convert Matrix to Quat
        //quat.normalize();
        //return 2 * acos(quat.coeffs().data()[0]); 

        /* Note: Euler sufficient for static base frame, unique rigid bodies. */
    //    Eigen::Vector3d ea = rot_rel.eulerAngles(0,1,2);
    //    return ea[id] ; 
   // }
//////////////////////////
    void printTfs (tf::StampedTransform transform) {
        double yaw, pitch, roll;
        transform.getBasis().getRPY(roll, pitch, yaw);
        cout << roll << " " << pitch << " " << yaw << endl;
        
        //Eigen::Matrix3d rot;
        //tf::matrixTFToEigen( transform.getBasis(), rot);
        //Eigen::Vector3d ea = rot.eulerAngles(0,1,2);
        //cout << ea[0] << " " << ea[1] << " " << ea[2] << endl;
    }
//////////////////////////
    void loop(){
        ros::Rate rate(1000.0); // 1kHz

        vector <float> jpVec_0 (_DOF, 0.0);
        vector <float> jpVec (_DOF);

        while (node.ok()) {
        /* Update TF measurement */
            getJointTfs( &tfVec );
            getJointAngles( &tfVec_0, &tfVec, &jpVec );
            
            float delta = 0.0;
            for (int i=0; i<_DOF; i++) {
                float delta_i = jpVec[i] - jpVec_0[i];
                if ( abs(delta) < abs(delta_i) ) delta = delta_i; 
            }
            if ( abs(delta) > 0.01 ){
                jpVec_0 = jpVec;
                cout << (jpVec[0]) * 180.0/3.1415 << " "   
                     << (jpVec[1]) * 180.0/3.1415 << " "   
                     << (jpVec[2]) * 180.0/3.1415 << " "   
                     << (jpVec[3]) * 180.0/3.1415 << " " << endl; 
            }
        }


//        getJointTf( tf_jt1, l1_name, base_name );
 //       getJointTf( tf_jt2, l2_name, l1_name );
 //  
 //       // Test Joint 1
 //       
 //       double angle1_0 = getJointAngle(tf_jt1, tf_jt1_0);
//  
//        while (node.ok()) {
//            getJointTf( tf_jt1, l1_name, base_name );
//            getJointTf( tf_jt2, l2_name, l1_name );
//            double angle1 = getJointAngle(tf_jt1, tf_jt1_0);
//            double delta1 = angle1 - angle1_0;
//           
//            if (delta1 > 0.01 || delta1 < -0.01){
//                angle1_0 = angle1;
//                angle1 = angle1 + delta1;
//                cout << angle1 * 180.0 / 3.1415 << endl;
//            }
//        }
 //
 //       // Test Joint 2
 //       
 //       double angle_0 = getJointAngle(tf_jt2, tf_jt2_0);
 //
 //       while (node.ok()) {
 //           getJointTf( tf_jt1, l1_name, base_name );
 //           getJointTf( tf_jt2, l2_name, l1_name );
 //           double angle = getJointAngle(tf_jt2, tf_jt2_0);
 //           double delta = angle - angle_0;
 //          
 //           if (delta > 0.01 || delta < -0.01){
 //               angle_0 = angle;
 //               angle = angle + delta;
 //               std::cout << angle * 180.0 / 3.1415 << std::endl;
 //           }
 //       }
    }
};

//////////////////////////
//////////////////////////

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
