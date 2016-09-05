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

    string line1,line2;

    ifstream trajectories("CameraTrajectory.txt");
    if(!trajectories.is_open())
        return -1;

    ifstream associations("associations.txt");
    if(!associations.is_open())
        return -1;

    MessageReader reader;
    reader.open("orazio_2016_run.txt");
    BaseMessage* msg = 0;

    MessageWriter writer;
    writer.open("orazio_2016_run_corrected.txt");


    while (getline(trajectories,line1)) {
        istringstream iss1(line1);
        string pose_stamp;
        float x,y,z,qx,qy,qz,qw;
        iss1 >> pose_stamp;
        iss1 >> x;
        iss1 >> y;
        iss1 >> z;
        iss1 >> qx;
        iss1 >> qy;
        iss1 >> qz;
        iss1 >> qw;
        Eigen::Vector3f t(x,y,z);
        Eigen::Quaternion<float> q(qw,qx,qy,qz);

        string rgb_stamp,rgb_filename,depth_stamp,depth_filename;
        do {
            getline(associations,line2);
            istringstream iss2(line2);
            iss2 >> rgb_stamp;
            iss2 >> rgb_filename;
            iss2 >> depth_stamp;
            iss2 >> depth_filename;
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
    }

    writer.close();
    reader.close();
    associations.close();
    trajectories.close();

    return 0;
}
