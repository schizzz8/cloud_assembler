#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <txt_io/message_reader.h>
#include <txt_io/message_writer.h>
#include <txt_io/pinhole_image_message.h>
#include "globals/system_utils.h"

#include <Eigen/Core>

#include <opencv/highgui.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>


typedef Eigen::Matrix<float, 7, 1> Vector7f;

using namespace system_utils;
using namespace std;
using namespace txt_io;
using namespace std;


int main(int argc, char ** argv) {

    int count = 2000;
    string pose_stamp;
    string line;
    ifstream trajectories("CameraTrajectory.txt");
    ifstream associations("associations.txt");

    MessageReader reader;
    reader.open("orazio_2016_run.txt");

    MessageWriter writer;
    writer.open("orazio_2016_run_corrected.txt");

    BaseMessage* msg = 0;

    if(trajectories.is_open() && associations.is_open()) {
        while (count > 0) {
            trajectories >> pose_stamp;
            float x,y,z,qx,qy,qz,qw;
            trajectories >> x;
            trajectories >> y;
            trajectories >> z;
            trajectories >> qx;
            trajectories >> qy;
            trajectories >> qz;
            trajectories >> qw;
            Eigen::Vector3f t(x,y,z);
            Eigen::Quaternion<float> q(qw,qx,qy,qz);

            string rgb_stamp,rgb_filename,depth_stamp,depth_filename;
            do {
                getline(associations,line);
                istringstream iss(line);
                iss >> rgb_stamp;
                iss >> rgb_filename;
                iss >> depth_stamp;
                iss >> depth_filename;
            } while(pose_stamp.find(rgb_stamp) == string::npos);

            bool found = false;
            while(found == false) {
                msg = reader.readMessage();
                msg->untaint();
                BaseImageMessage* img = dynamic_cast<BaseImageMessage*> (msg);
                if(img)
                    if(strcmp(img->topic().c_str(),"/camera/depth/image_raw") == 0 && atof(depth_stamp.c_str())-img->timestamp() == 0) {
                        img->setOffset(Eigen::Isometry3f::Identity());
                        Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
                        transform.translation() = t;
                        transform.rotate (q);
                        img->setOdometry(transform);
                        found = true;
                    }   
            }
            writer.writeMessage(*msg);
            count--;
        }
    }
    return 0;
}
