#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <txt_io/message_reader.h>
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

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


int main(int argc, char ** argv) {

    int count = 20;
    string pose_stamp;
    string line;
    ifstream trajectories("CameraTrajectory.txt");
    ifstream associations("associations.txt");

    MessageReader reader;
    reader.open("orazio_2016_run.txt");

    BaseMessage* msg = 0;

    Eigen::Matrix3f K;
    K << 285.171, 0, 160,
                    0, 285.171, 120,
                    0, 0, 1;
    Eigen::Matrix3f iK = K.inverse();
    int rows=0,cols=0;
    float depth_scale;
    PointCloud::Ptr aggregate_cloud (new PointCloud ());

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
            Eigen::Quaternion<float> q(qx,qy,qz,qw);

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
            cv::Mat depth_image;
            while(found == false) {
                msg = reader.readMessage();
                BaseImageMessage* img = dynamic_cast<BaseImageMessage*> (msg);
                if(img)
                    if(strcmp(img->topic().c_str(),"/camera/depth/image_raw") == 0 && atof(depth_stamp.c_str())-img->timestamp() == 0) {
                        depth_image = img->image();
                        rows = depth_image.rows;
                        cols = depth_image.cols;
                        depth_scale = img->depthScale();
                        found = true;
                    }
            }

            PointCloud::Ptr temp_cloud (new PointCloud ());
            for(size_t r=0; r<rows; r++) {
                const unsigned short* id_ptr = depth_image.ptr<unsigned short>(r);
                for(size_t c=0; c<cols; c++) {
                    unsigned short id = *id_ptr;
                    float d = id*depth_scale;
                    Eigen::Vector3f p = iK*Eigen::Vector3f(c*d,r*d,d);
                    temp_cloud->push_back(PointT (p.x(),p.y(),p.z()));
                    id_ptr++;
                }
            }

            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.translation() = t;
            transform.rotate (q);
            PointCloud::Ptr transformed_cloud (new PointCloud ());
            pcl::transformPointCloud(*temp_cloud,*transformed_cloud,transform);

            *aggregate_cloud += *transformed_cloud;
            count--;
        }
    }
    pcl::io::savePCDFileASCII ("test_pcd.pcd", *aggregate_cloud);

    return 0;
}
