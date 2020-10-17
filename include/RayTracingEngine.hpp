#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <chrono>
#include <unordered_map> 
#include <unordered_set> 
#include <queue>
#include <fstream>
#include <thread>
#include <ctime>
#include <Eigen/Dense>
#include <Eigen/Core>

#include<Camera.hpp>
#include<CommonUtilities.hpp>

using namespace Eigen;
using namespace std;

constexpr double k_AngleMin = 0;
constexpr double k_AngleMax = 45;
constexpr double k_ZMin = 0.20;
constexpr double k_ZMax = 1.0;

class RayTracingEngine
{
    public:
        Camera cam_;
        RayTracingEngine(Camera &cam);
        void rayTrace(VoxelVolume& volume,Eigen::Affine3f& transformation,int zdelta,bool sparse);
        void rayTraceAndClassify(VoxelVolume& volume,Eigen::Affine3f& transformation,bool sparse);
        void rayTraceVolume(VoxelVolume& volume,Eigen::Affine3f& transformation);
        std::pair<bool,std::vector<unsigned long long int>> rayTraceAndGetGoodPoints(VoxelVolume& volume,Eigen::Affine3f& transformation,bool sparse);
};

RayTracingEngine::RayTracingEngine(Camera &cam):cam_(cam){}

//Raytracing from the depth map.
void RayTracingEngine::rayTrace(VoxelVolume& volume,Eigen::Affine3f& transformation,int zdelta = 10,bool sparse=true)
{
    int width = cam_.getWidth();
    int height = cam_.getHeight();
    bool found[cam_.getHeight()][cam_.getWidth()]={false};
    int rdelta = 1, cdelta = 1;
    if(sparse==true)
    {
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
    Eigen::Affine3f normals_transformation = Eigen::Affine3f::Identity();
    Eigen::Affine3f inverse_transformation = transformation.inverse();
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            normals_transformation(i,j)=inverse_transformation(i,j);
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
                    {
                        for(auto normal:voxel->normals)
                        {
                            auto [nx,ny,nz] = cam_.transformPoints(normal.normal[0],normal.normal[1],normal.normal[2],normals_transformation);
                            float nml[3]={nx,ny,nz};
                            int angle_z = angle(nml);
                            if(angle_z>=k_AngleMin&&angle_z<=k_AngleMax)
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
}

std::pair<bool,std::vector<unsigned long long int>> RayTracingEngine::rayTraceAndGetGoodPoints(VoxelVolume& volume,Eigen::Affine3f& transformation,bool sparse=true)
{
    int width = cam_.getWidth();
    int height = cam_.getHeight();
    bool found[cam_.getHeight()][cam_.getWidth()]={false};
    Eigen::Affine3f normals_transformation = Eigen::Affine3f::Identity();
    Eigen::Affine3f inverse_transformation = transformation.inverse();
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            normals_transformation(i,j)=inverse_transformation(i,j);
    int zdelta = 1,rdelta = 1, cdelta = 1;
    if(sparse==true)
    {
        zdelta=5;
        rdelta=10;
        cdelta=10;
    }
    bool point_found = false;
    vector<unsigned long long int> good_points;
    unordered_set<unsigned long long int> checked;
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
                auto id = volume.getHashId(xid,yid,zid);
                if(checked.find(id)!=checked.end())
                    continue;
                Voxel *voxel = volume.voxels_[xid][yid][zid];
                if(voxel!=nullptr)
                {
                    found[r][c]=true;
                    point_found = true;
                    for(auto normal:voxel->normals)
                    {
                        auto [nx,ny,nz] = cam_.transformPoints(normal.normal[0],normal.normal[1],normal.normal[2],normals_transformation);
                        float nml[3]={nx,ny,nz};
                        int angle_z = angle(nml);
                        if(angle_z>=k_AngleMin&&angle_z<=k_AngleMax)
                        {
                            checked.insert(id);
                            good_points.push_back(id);
                            break;
                        }
                    }
                }
            }
        }
    }
    return {point_found,good_points};
}

//Raytracing on the volume.

void RayTracingEngine::rayTraceVolume(VoxelVolume& volume,Eigen::Affine3f& transformation)
{
    int width = cam_.getWidth();
    int height = cam_.getHeight();
    int depth[cam_.getHeight()][cam_.getWidth()];
    for(int i=0;i<height;i++)
        for(int j=0;j<width;j++)
            depth[i][j]=-1;
    Eigen::Affine3f inverse_transformation = transformation.inverse();
    //TODO: Replace these loops with the only the occupied voxels.
    int counter = 0;
    for(float x=volume.xmin_;x<volume.xmax_;x+=volume.xdelta_)
        for(float y=volume.ymin_;y<volume.ymax_;y+=volume.ydelta_)
            for(float z=volume.zmin_;z<volume.zmax_;z+=volume.zdelta_)
            {
                auto coords = volume.getVoxel(x,y,z);
                int xid = get<0>(coords);
                int yid = get<1>(coords);
                int zid = get<2>(coords);
                if(volume.voxels_[xid][yid][zid]==nullptr)
                    continue;
                auto transformed_points = cam_.transformPoints(x+volume.xdelta_/2.0,y+volume.ydelta_/2.0,z+volume.zdelta_/2.0,inverse_transformation);
                float xx = get<0>(transformed_points);
                float yy = get<1>(transformed_points);
                float zz = get<2>(transformed_points);
                auto pixel = cam_.deProjectPoint(xx,yy,zz);
                int r = get<0>(pixel);
                int c = get<1>(pixel);
                if(cam_.validPixel(r,c)==false)
                    continue;
                int d = int(round(zz*1000));
                if(depth[r][c]==-1)
                    depth[r][c] = d;
                else
                {
                    depth[r][c] = min(depth[r][c],d);
                }
                counter++;
            }
    cout<<"Depth Map Found"<<endl;
    std::cout<<counter<<endl;
    
    for(float x=volume.xmin_;x<volume.xmax_;x+=volume.xdelta_)
        for(float y=volume.ymin_;y<volume.ymax_;y+=volume.ydelta_)
            for(float z=volume.zmin_;z<volume.zmax_;z+=volume.zdelta_)
            {
                auto coords = volume.getVoxel(x,y,z);
                int xid = get<0>(coords);
                int yid = get<1>(coords);
                int zid = get<2>(coords);
                if(volume.voxels_[xid][yid][zid]==nullptr)
                    continue;
                auto transformed_points = cam_.transformPoints(x+volume.xdelta_/2.0,y+volume.ydelta_/2.0,z+volume.zdelta_/2.0,inverse_transformation);
                float xx = get<0>(transformed_points);
                float yy = get<1>(transformed_points);
                float zz = get<2>(transformed_points);
                auto pixel = cam_.deProjectPoint(xx,yy,zz);
                int r = get<0>(pixel);
                int c = get<1>(pixel);
                if(cam_.validPixel(r,c)==false)
                    continue;
                int d = int(round(zz*1000));
                Voxel *voxel = volume.voxels_[xid][yid][zid];
                if(depth[r][c]==d)
                    voxel->view=1;//TODO: Change it to view number.
            }
}
