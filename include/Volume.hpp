#pragma once

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

/*********************************************/
//OTHER HEADERS
/**********************************************/
#include <Eigen/Dense>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>

const unsigned long long int k_Prime1 = 73856093;
const unsigned long long int k_Prime2 = 19349669;
const unsigned long long int k_Prime3 = 83492791;

using namespace std;

class VoxelVolume
{
    unordered_map<unsigned long long int,vector<vector<float>>> lut_;
    public:
    double xmin_,xmax_,ymin_,ymax_,zmin_,zmax_;
    double xdelta_,ydelta_,zdelta_;
    int xdim_,ydim_,zdim_;
    unsigned long long int hsize_;
    vector<vector<vector<int>>> voxels_;
    VoxelVolume(){};
    void setDimensions(double xmin,double xmax,double ymin,double ymax, double zmin, double zmax);
    void setResolution(double xdelta, double ydelta, double zdelta);
    void setVolumeSize(int xdim,int ydim,int zdim);
    bool constructVolume();
    template<typename PointT> bool addPointCloud(typename pcl::PointCloud<PointT>::Ptr pointcloud);
    unsigned long long int getHash(float x,float y,float z);
    tuple<int,int,int> getVoxel(float x,float y,float z);
};

void VoxelVolume::setDimensions(double xmin,double xmax,double ymin,double ymax,double zmin,double zmax)
{
    xmin_=xmin;
    xmax_=xmax;
    ymin_=ymin;
    ymax_=ymax;
    zmin_=zmin;
    zmax_=zmax;
}

void VoxelVolume::setResolution(double xdelta,double ydelta,double zdelta)
{
    xdelta_=xdelta;
    ydelta_=ydelta;
    zdelta_=zdelta;
    xdim_=(xmax_-xmin_)/xdelta_;
    ydim_=(ymax_-ymin_)/ydelta_;
    zdim_=(zmax_-zmin_)/zdelta_;
    hsize_=xdim_*ydim_*zdim_;
}

void VoxelVolume::setVolumeSize(int xdim,int ydim,int zdim)
{
    xdim_=xdim;
    ydim_=ydim;
    zdim_=zdim;
    xdelta_=(xmax_-xmin_)/xdim;
    ydelta_=(ymax_-ymin_)/ydim;
    zdelta_=(zmax_-zmin_)/zdim;
    hsize_=xdim_*ydim_*zdim_;
}

bool VoxelVolume::constructVolume()
{
    voxels_=vector<vector<vector<int>>>(xdim_, vector<vector<int>>(ydim_, vector<int>(zdim_,0)));
    return true;
}

template<typename PointT> bool VoxelVolume::addPointCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    return true;
}

inline unsigned long long int VoxelVolume::getHash(float x,float y,float z)
{
    auto coords = getVoxel(x,y,z);
    return ((get<0>(coords)*k_Prime1)^(get<1>(coords)*k_Prime2)^(get<2>(coords)*k_Prime3))%hsize_;
}

inline tuple<int,int,int> VoxelVolume::getVoxel(float x,float y,float z)
{
    int xv = floor((x-xmin_)/xdelta_);
    int yv = floor((y-ymin_)/ydelta_);
    int zv = floor((z-zmin_)/zdelta_);
    return {xv,yv,zv};
}
