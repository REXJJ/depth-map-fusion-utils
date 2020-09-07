#pragma once
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
#include <Eigen/Dense>
#include <Eigen/Core>

#include<Camera.hpp>

using namespace Eigen;
using namespace std;

constexpr int degree(double radian){return int((radian*180)/3.14159);};
constexpr double magnitude(float normal[3]){return normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2];};
constexpr int angle(float normal[3]){return degree(acos(-normal[2]/magnitude(normal)));};
template <typename T> constexpr int sgn(T x) {
    return (T(0) < x) - (x < T(0));
}

constexpr double k_AngleMin = 0;
constexpr double k_AngleMax = 45;
constexpr double k_ZMin = 0.20;
constexpr double k_ZMax = 1.0;

class RayTracingEngine
{
    public:
        Camera cam_;
        RayTracingEngine(Camera &cam);
        void rayTrace(VoxelVolume& volume,Eigen::Affine3f& transformation,bool sparse);
        void rayTraceAndClassify(VoxelVolume& volume,Eigen::Affine3f& transformation,bool sparse);
        void rayTraceVolume(VoxelVolume& volume,Eigen::Affine3f& transformation);
};

RayTracingEngine::RayTracingEngine(Camera &cam):cam_(cam){}

void RayTracingEngine::rayTrace(VoxelVolume& volume,Eigen::Affine3f& transformation,bool sparse=true)
{
    int width = cam_.getWidth();
    int height = cam_.getHeight();
    bool found[cam_.getHeight()][cam_.getWidth()]={false};
    int zdelta = 1,rdelta = 1, cdelta = 1;
    if(sparse==true)
    {
        zdelta=5;
        rdelta=10;
        cdelta=10;
    }
    for(int z_depth=10;z_depth<k_ZMax*1000;z_depth+=zdelta)
    {
        for(int r=0;r<height;r+=rdelta)
        { 
            for(int c=0;c<width;c+=cdelta)
            {   
                if(found[r][c])
                    continue;
                auto [x,y,z] = cam_.projectPoint(r,c,z_depth);
                tie(x,y,z) = cam_.transformPoints(x,y,z,transformation);
                if(volume.validPoints(x,y,z)==false)
                    continue;
                auto coords = volume.getVoxel(x,y,z);
                int xid = get<0>(coords);
                int yid = get<1>(coords);
                int zid = get<2>(coords);
                Voxel *voxel = volume.voxels_[xid][yid][zid];
                if(voxel!=nullptr)
                {
                    found[r][c]=true;
                    voxel->view=1;//TODO: Change it to view number.
                }
            }
        }

    }
}

void RayTracingEngine::rayTraceAndClassify(VoxelVolume& volume,Eigen::Affine3f& transformation,bool sparse=true)
{
    int width = cam_.getWidth();
    int height = cam_.getHeight();
    bool found[cam_.getHeight()][cam_.getWidth()]={false};
    Eigen::Affine3f normalsTransformation = Eigen::Affine3f::Identity();
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            normalsTransformation(i,j)=transformation(i,j);
    int zdelta = 1,rdelta = 1, cdelta = 1;
    if(sparse==true)
    {
        zdelta=5;
        rdelta=10;
        cdelta=10;
    }
    for(int z_depth=10;z_depth<k_ZMax*1000;z_depth+=zdelta)
    {
        for(int r=0;r<height;r+=rdelta)
        { 
            for(int c=0;c<width;c+=cdelta)
            {   
                if(found[r][c])
                    continue;
                auto [x,y,z] = cam_.projectPoint(r,c,z_depth);
                tie(x,y,z) = cam_.transformPoints(x,y,z,transformation);
                if(volume.validPoints(x,y,z)==false)
                    continue;
                auto coords = volume.getVoxel(x,y,z);
                int xid = get<0>(coords);
                int yid = get<1>(coords);
                int zid = get<2>(coords);
                Voxel *voxel = volume.voxels_[xid][yid][zid];
                if(voxel!=nullptr)
                {
                    found[r][c]=true;
                    voxel->view=1;//TODO: Change it to view number.
                    if(voxel->good==false)
                        for(auto normal:voxel->normals)
                        {
                            auto [nx,ny,nz] = cam_.transformPoints(normal.normal[0],normal.normal[1],normal.normal[2],normalsTransformation);
                            float nml[3]={nx,ny,nz};
                            int angle_z = angle(nml);
                            if(angle_z>=k_AngleMin&&angle_z<=k_AngleMax&&z>=k_ZMin&&z<=k_ZMax)
                            {
                                voxel->good = true;
                                break;
                            }
                        }
                }
            }
        }

    }
}


void RayTracingEngine::rayTraceVolume(VoxelVolume& volume,Eigen::Affine3f& transformation)
{
    int width = cam_.getWidth();
    int height = cam_.getHeight();
    int depth[cam_.getHeight()][cam_.getWidth()]={-1};
    Eigen::Affine3f inverseTransformation = transformation.inverse();
    for(float x=volume.xmin_;x<volume.xmax_;x+=volume.xdelta_)
        for(float y=volume.ymin_;y<volume.ymax_;y+=volume.ydelta_)
            for(float z=volume.zmax_-volume.zdelta_;z>=volume.zmin_;z-=volume.zdelta_)
            {
                auto coords = volume.getVoxel(x,y,z);
                int xid = get<0>(coords);
                int yid = get<1>(coords);
                int zid = get<2>(coords);
                if(volume.voxels_[xid][yid][zid]==nullptr)
                    continue;
                auto[xx,yy,zz] = cam_.transformPoints(x,y,z,inverseTransformation);
                auto[r,c] = cam_.deProjectPoint(xx,yy,zz);
                if(cam_.validPixel(r,c)==false)
                    continue;
                depth[r][c] = z*1000;
            }
    cout<<"Depth Map Found"<<endl;
    for(int r=0;r<cam_.getHeight();r++)
        for(int c=0;c<cam_.getWidth();c++)
            if(depth[r][c]>0)
            {
                auto[x,y,z] = cam_.projectPoint(r,c,depth[r][c]);
                tie(x,y,z) = cam_.transformPoints(x,y,z,transformation);
                auto coords = volume.getVoxel(x,y,z);
                int xid = get<0>(coords);
                int yid = get<1>(coords);
                int zid = get<2>(coords);
                if(volume.validPoints(x,y,z)==false)
                        continue;
                Voxel *voxel = volume.voxels_[xid][yid][zid];
                if(voxel==nullptr)
                    continue;
                voxel->view=1;//TODO: Change it to view number.
            }
}

