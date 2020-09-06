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
using namespace pcl;

struct Voxel
{
    vector<pcl::PointXYZRGB> pts;
    int view;
    Voxel(pcl::PointXYZRGB pt)
    {
        pts.push_back(pt);
        view=0;
    }
};

class VoxelVolume
{
    unordered_map<unsigned long long int,vector<vector<float>>> lut_;
    public:
    double xmin_,xmax_,ymin_,ymax_,zmin_,zmax_;
    double xdelta_,ydelta_,zdelta_;
    int xdim_,ydim_,zdim_;
    unsigned long long int hsize_;
    vector<vector<vector<Voxel*>>> voxels_;
    VoxelVolume(){};
    void setDimensions(double xmin,double xmax,double ymin,double ymax, double zmin, double zmax);
    void setResolution(double xdelta, double ydelta, double zdelta);
    void setVolumeSize(int xdim,int ydim,int zdim);
    bool constructVolume();
    template<typename PointT> bool addPointCloud(typename pcl::PointCloud<PointT>::Ptr pointcloud);
    unsigned long long int getHash(float x,float y,float z);
    tuple<int,int,int> getVoxel(float x,float y,float z);
    ~VoxelVolume();
    bool integratePointCloud(pcl::PointCloud<PointXYZRGB>::Ptr cloud);
    bool validPoints(float x,float y,float z);
};

VoxelVolume::~VoxelVolume()
{
    for(int x=0;x<xdim_;x++)
        for(int y=0;y<ydim_;y++)
            for(int z=0;z<zdim_;z++)
                if(voxels_[x][y][z])
                    delete voxels_[x][y][z];
}

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
    voxels_=vector<vector<vector<Voxel*>>>(xdim_, vector<vector<Voxel*>>(ydim_, vector<Voxel*>(zdim_,nullptr)));
    return true;
}

template<typename PointT> bool VoxelVolume::addPointCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    return true;
}

inline unsigned long long int VoxelVolume::getHash(float x,float y,float z)
{
    auto coords = getVoxel(x,y,z);
    // return ((get<0>(coords)*k_Prime1)^(get<1>(coords)*k_Prime2)^(get<2>(coords)*k_Prime3))%hsize_;
    return (get<0>(coords)<<40)^(get<1>(coords)<<20)^(get<2>(coords));
}

inline tuple<int,int,int> VoxelVolume::getVoxel(float x,float y,float z)
{
    int xv = floor((x-xmin_)/xdelta_);
    int yv = floor((y-ymin_)/ydelta_);
    int zv = floor((z-zmin_)/zdelta_);
    return {xv,yv,zv};
}

bool VoxelVolume::integratePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    for(int i=0;i<cloud->points.size();i++)
    {
        pcl::PointXYZRGB pt = cloud->points[i];
        if(validPoints(pt.x,pt.y,pt.z)==false)
            continue;
        auto coords = getVoxel(pt.x,pt.y,pt.z);
        int x = get<0>(coords);
        int y = get<1>(coords);
        int z = get<2>(coords);
        if(voxels_[x][y][z]==nullptr)
        {
            Voxel *voxel = new Voxel(pt);
            voxels_[get<0>(coords)][get<1>(coords)][get<2>(coords)] = voxel;
        }
        else
        {
            Voxel *voxel = voxels_[x][y][z];
            voxel->pts.push_back(pt);
        }
    }

}

bool VoxelVolume::validPoints(float x,float y,float z)
{
    return !(x>=xmax_||y>=ymax_||z>=zmax_||x<=xmin_||y<=ymin_||z<=zmin_);
}