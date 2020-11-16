/***********************************************/
//STANDARD HEADERS
/************************************************/
#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <chrono>
#include <unordered_map> 
#include <queue>
#include <fstream>
#include <thread>
#include <ctime>
#include <mutex>          // std::mutex

/*********************************************/
//PCL HEADERS
/**********************************************/
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/common/common.h>

/*********************************************/
//OTHER HEADERS
/**********************************************/
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <FileRoutines.hpp>

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace boost::interprocess;

void usage(string program_name)
{
    std::cout<<program_name<<" <Pointcloud filename>"<<std::endl;
    exit(-1);
}

int main(int argc, char** argv)
{

    shared_memory_object shm_obj
        ( open_only
         ,"shared_memory"              //name
         ,read_write                   //read-write mode
        );

    mapped_region region(shm_obj, read_write);
    std::cout<<region.get_size()<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    MatrixXf m;
    m.resize(881261,12);
    memcpy(m.data(),region.get_address(),region.get_size());
    for(int i=0;i<m.rows();i++)
    {
        pcl::PointXYZ pt;
        pt.x = m(i,0);
        pt.y = m(i,1);
        pt.z = m(i,2);
        cloud->points.push_back(pt);
    }
    cloud->width = m.rows();
    cloud->height = 1;
    pcl::io::savePCDFileASCII ("/home/rex/REX_WS/test_new.pcd",*cloud);
    return 0;
}
