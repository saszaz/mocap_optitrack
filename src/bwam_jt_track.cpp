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
#include <fstream>

using namespace std;

const int DOF=4;
/* Ids to use for Joint angle determination */
// TF 
const vector<int> tfIds={0,1,2,3,4};
// Euler angle
const vector<int> eaIds={1,1,1,1,1};
// Rigid bodies 
const vector<string> rgIds={"Base/base_link",
                           "Link_1/base_link",
                           "Link_2/base_link",
                           "Link_3/base_link",
                           "Link_4/base_link"};
// file_path
const string path = "./joint_dat.txt";

//#define DEBUG

class jointDataHandler { 

    public: 

        jointDataHandler(const ros::NodeHandle nh)
            : node(nh),
              _DOF(DOF),
              _tfIds(tfIds),
              _eaIds(eaIds),
              _rgIds(rgIds),
              _numTF(rgIds.size()-1),
              _path(path)
            {
                tfVec_0.resize(_numTF);
                tfVec.resize(_numTF);

                // Save home position
                ros::Duration(1.0).sleep();
                getJointTfs (&tfVec_0);
           
                // Open dat file
                jp_file.open(_path,ios::out);
            }

        ~jointDataHandler(){
            jp_file.close();
            };

    private:

        const int _DOF;
        const int _numTF;
        const vector<int> _tfIds, _eaIds;
        const vector<string> _rgIds;
        const string _path;

    public:

        tf::TransformListener listener;
        vector <tf::StampedTransform> tfVec_0, tfVec;
        ros::NodeHandle node;
        ofstream jp_file; 
        //systems::Time abstime; // Xenomai timestamp

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
            (* tf_vec)[i] = transform;
        }
    }
//////////////////////////
    void getJointAngles ( const vector <tf::StampedTransform> * tf_vec_0,
                          const vector <tf::StampedTransform> * tf_vec_1, 
                          vector <float> * jp_vec ) {
          
        Eigen::Matrix3d rot_0, rot_1, rot_rel;
        //tf::Matrix3x3 rot_0, rot_1, rot_rel;
        for (int j=0; j<_DOF; j++) {
            int tf_id = _tfIds[j];
            int ea_id = _eaIds[j];
            tf::matrixTFToEigen( (*tf_vec_0)[tf_id].getBasis(), rot_0);
            tf::matrixTFToEigen( (*tf_vec_1)[tf_id].getBasis(), rot_1);
            //rot_0 = (*tf_vec_0)[tf_id].getBasis();
            //rot_1 = (*tf_vec_1)[tf_id].getBasis();

            try {
                rot_rel = rot_0.inverse() * rot_1;
            }
            catch ( std::exception &ex){
                ROS_ERROR("%s",ex.what());
            }

            //Eigen::Vector3d ea = rot_rel.eulerAngles(0,1,2);
            Eigen::AngleAxisd aa(rot_rel); 
            (*jp_vec)[j] = aa.angle();

            /* Set angle sign */
            if (aa.axis()[2] < 0.0) (*jp_vec)[j] *= -1.0; // Set parent frame orient. in Motive  
            
            //double yaw, pitch, roll;
            //rot_rel.getRPY(roll,pitch,yaw);
            //vector <float> ea (3);
            //ea[0]=roll; ea[1]=pitch; ea[2]=yaw;
            //(*jp_vec)[j] = ea[ea_id];
        }

    }

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
    void writeJp2File ( const double time, const vector <float> * jp_vec ) {
        
        string jp_out;
        char buffer [100];
        sprintf(buffer, "%30.18f", time);
        jp_out = buffer;
        for (int j=0; j < _DOF; j++) {
        char buffer2 [50];
            sprintf(buffer2, "%8.6f", (*jp_vec)[j]);
            jp_out = jp_out + ", " + buffer2;
        }
        jp_file << jp_out << endl;
    }

//////////////////////////
    void writeTf2File () {
        
        tf::StampedTransform transform;
        getJointTf( &transform, rgIds[1], "world");
        double time = transform.stamp_.toSec();

        string jp_out;
        char buffer [100];
        sprintf(buffer, "%30.18f", time);
        jp_out = buffer;
        for (int j=0; j < 1; j++) {
        char buffer2 [50];
            sprintf(buffer2, "%8.6f", transform.getRotation().getAngle());
            jp_out = jp_out + ", " + buffer2;
        }
        jp_file << jp_out << endl;
    }
//////////////////////////

    void loop(){

        vector <float> jpVec_0 (_DOF, 0.0);
        vector <float> jpVec (_DOF);

        ros::Rate rate(500); // Hz 
        double t_0 = 0.0;
        while (node.ok()) {
        /* Update TF measurement */
            getJointTfs( &tfVec );
            getJointAngles( &tfVec_0, &tfVec, &jpVec );
            double t = tfVec[0].stamp_.toSec();
            if (t > t_0) {
                writeJp2File ( t, &jpVec) ;
                t_0 = t;
            }
           
            #ifdef DEBUG
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
                     << (jpVec[3]) * 180.0/3.1415 << endl;
            }
            #endif
            rate.sleep();
        } 
    }
};

//////////////////////////
//////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc,argv,"bwam_jt_track");

    ros::NodeHandle nh;

    jointDataHandler  jt_dh(nh); 
    
    jt_dh.loop();

    return 0;
};
