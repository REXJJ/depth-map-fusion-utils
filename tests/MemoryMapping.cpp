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
    struct shm_remove
    {
        shm_remove() { shared_memory_object::remove("shared_memory"); }
        ~shm_remove(){ shared_memory_object::remove("shared_memory"); }
    } remover;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    readPointCloud(string(argv[1]),cloud_normal,cloud,normals);

    MatrixXf m = cloud_normal->getMatrixXfMap().transpose();
    std::cout<<m.rows()<<" "<<m.cols()<<std::endl;

    unsigned long long int cloud_size = m.rows()*m.cols();

    std::cout<<cloud_size<<std::endl;

    shared_memory_object shm_obj
        (create_only                  //only create
         ,"shared_memory"              //name
         ,read_write                   //read-write mode
        );

    shm_obj.truncate(cloud_size*sizeof(float));
    mapped_region region(shm_obj, read_write);

    memcpy(region.get_address(),m.data(),region.get_size());

    return 0;
}
