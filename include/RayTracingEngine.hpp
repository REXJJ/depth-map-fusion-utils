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
constexpr double k_AngleMax = 60;
constexpr double k_ZMin = 0.20;
constexpr double k_ZMax = 1.0;

class RayTracingEngine
{
    public:
        Camera cam_;
        RayTracingEngine(Camera &cam);
        void rayTrace(VoxelVolume& volume,Eigen::Affine3f& transformation,int zdelta,bool sparse);
        void rayTraceAndClassify(VoxelVolume& volume,Eigen::Affine3f& transformation,int zdelta,int view,bool sparse);
        void rayTraceVolume(VoxelVolume& volume,Eigen::Affine3f& transformation);
        std::pair<bool,std::vector<unsigned long long int>> rayTraceAndGetGoodPoints(VoxelVolume& volume,Eigen::Affine3f& transformation,int zdelta,bool sparse);
        std::pair<bool,std::vector<unsigned long long int>> rayTraceAndGetPoints(VoxelVolume& volume,Eigen::Affine3f& transformation,int zdelta,bool sparse);
        std::pair<bool,std::vector<unsigned long long int>> reverseRayTrace(VoxelVolume& volume, Eigen::Affine3f transformation,bool viz,int zdelta);
        std::pair<bool,std::vector<unsigned long long int>> reverseRayTraceFast(VoxelVolume& volume, Eigen::Affine3f transformation,bool viz,int zdelta);
};

RayTracingEngine::RayTracingEngine(Camera &cam):cam_(cam){}


std::pair<bool,std::vector<unsigned long long int>> RayTracingEngine::reverseRayTrace(VoxelVolume& volume, Eigen::Affine3f transformation,bool viz, int zdelta = 1)
{
    int width = cam_.getWidth();
    int height = cam_.getHeight();
    Eigen::Affine3f inverseT = transformation.inverse();

    //TODO: Replace this with hash values
    double found = false;
    vector<unsigned long long int> good_points;
    for(float x=volume.xmin_;x<volume.xmax_;x+=volume.xdelta_)
        for(float y=volume.ymin_;y<volume.ymax_;y+=volume.ydelta_)
            for(float z=volume.zmin_;z<volume.zmax_;z+=volume.zdelta_)
            {
                auto coords = volume.getVoxel(x,y,z);
                int xid = get<0>(coords);
                int yid = get<1>(coords);
                int zid = get<2>(coords);
                auto voxel = volume.voxels_[xid][yid][zid];
                if(voxel==nullptr)//If voxel is not occupied
                    continue;
                Vector3f centroid;
                centroid<<x+volume.xdelta_/2.0,y+volume.ydelta_/2.0,z+volume.zdelta_/2.0;
                auto transformed_points = cam_.transformPoints(x+volume.xdelta_/2.0,y+volume.ydelta_/2.0,z+volume.zdelta_/2.0,inverseT);
                float xx = get<0>(transformed_points);
                float yy = get<1>(transformed_points);
                float zz = get<2>(transformed_points);
                auto pixel = cam_.deProjectPoint(xx,yy,zz);
                int r = get<0>(pixel);
                int c = get<1>(pixel);
                unsigned long long int centroid_hash = volume.getHash(centroid(0),centroid(1),centroid(2));
                if(cam_.validPixel(r,c)==false)//If the voxel is not in line with the camera.
                    continue;
                bool collided = false;
                Vector3f camera_center;
                camera_center<<transformation(0,3),transformation(1,3),transformation(2,3);
                Vector3f v = (camera_center-centroid).normalized();
                for(int depth=1;;depth++){
                    Vector3f pt = centroid + v*double(depth)/1000.0;
                    double xx = pt(0);
                    double yy = pt(1);
                    double zz = pt(2);
                    if(volume.validPoints(xx,yy,zz)==false)
                        break;
                    unsigned long long int hash = volume.getHash(xx,yy,zz);
                    if(hash==centroid_hash) //Same point as the origin.
                        continue;
                    auto coords = volume.getVoxel(xx,yy,zz);
                    int xidn = get<0>(coords);
                    int yidn = get<1>(coords);
                    int zidn = get<2>(coords);
                    Voxel *voxelNew = volume.voxels_[xidn][yidn][zidn];
                    if( voxelNew!=nullptr )
                    {
                        collided = true;
                        break;
                    }
                }
                if(collided==false)
                {
                    found = true;
                    if(viz==true)
                        voxel->view=1;//TODO: Change it to view number.
                    std::cout<<zz<<std::endl;
                    if(zz>=k_ZMin&&zz<=k_ZMax)
                    {
                        if(viz)
                            voxel->good = true;
                    }
                    if(false){
                        for(auto normal:voxel->normals)
                        {
                            Vector3f normals;
                            normals<<normal.normal[0],normal.normal[1],normal.normal[2];
                            int angle_z = degree(acos(normals.dot(v)));
                            if(angle_z>=k_AngleMin&&angle_z<=k_AngleMax)
                            {
                                if(viz==true)
                                    voxel->good = true;
                                good_points.push_back(centroid_hash);
                                break;
                            }
                        }
                    }

                }
            }
    return make_pair(found,good_points);
}

std::pair<bool,std::vector<unsigned long long int>> RayTracingEngine::reverseRayTraceFast(VoxelVolume& volume, Eigen::Affine3f transformation,bool viz, int zdelta = 1)
{
    int width = cam_.getWidth();
    int height = cam_.getHeight();
    Eigen::Affine3f inverseT = transformation.inverse();

    //TODO: Replace this with hash values
    double found = false;
    vector<unsigned long long int> good_points;
    for(auto hashes:volume.occupied_cells_)
    {
        float x,y,z;
        auto coords = volume.getVoxelCoords(hashes);
        int xid = get<0>(coords);
        int yid = get<1>(coords);
        int zid = get<2>(coords);
        x = xid*volume.xdelta_ + volume.xmin_;
        y = yid*volume.ydelta_ + volume.ymin_;
        z = zid*volume.zdelta_ + volume.zmin_;
        auto voxel = volume.voxels_[xid][yid][zid];
        if(voxel==nullptr)//If voxel is not occupied
            continue;
        Vector3f centroid;
        centroid<<x+volume.xdelta_/2.0,y+volume.ydelta_/2.0,z+volume.zdelta_/2.0;
        auto transformed_points = cam_.transformPoints(x+volume.xdelta_/2.0,y+volume.ydelta_/2.0,z+volume.zdelta_/2.0,inverseT);
        float xx = get<0>(transformed_points);
        float yy = get<1>(transformed_points);
        float zz = get<2>(transformed_points);
        auto pixel = cam_.deProjectPoint(xx,yy,zz);
        int r = get<0>(pixel);
        int c = get<1>(pixel);
        unsigned long long int centroid_hash = volume.getHash(centroid(0),centroid(1),centroid(2));
        if(cam_.validPixel(r,c)==false)//If the voxel is not in line with the camera.
            continue;
        bool collided = false;
        Vector3f camera_center;
        camera_center<<transformation(0,3),transformation(1,3),transformation(2,3);
        Vector3f v = (camera_center-centroid).normalized();
        for(int depth=1;;depth++){
            Vector3f pt = centroid + v*double(depth)/1000.0;
            double xx = pt(0);
            double yy = pt(1);
            double zz = pt(2);
            if(volume.validPoints(xx,yy,zz)==false)
                break;
            unsigned long long int hash = volume.getHash(xx,yy,zz);
            if(hash==centroid_hash) //Same point as the origin.
                continue;
            auto coords = volume.getVoxel(xx,yy,zz);
            int xidn = get<0>(coords);
            int yidn = get<1>(coords);
            int zidn = get<2>(coords);
            Voxel *voxelNew = volume.voxels_[xidn][yidn][zidn];
            if( voxelNew!=nullptr )
            {
                collided = true;
                break;
            }
        }
        if(collided==false)
        {
            found = true;
            if(viz==true)
                voxel->view=1;//TODO: Change it to view number.
            if(zz>=k_ZMin&&zz<=k_ZMax)
            {
                good_points.push_back(centroid_hash);
                if(viz)
                    voxel->good = true;
            }           // if(zz>=k_ZMin&&zz<=k_ZMax)
            if(false){
                for(auto normal:voxel->normals)
                {
                    Vector3f normals;
                    normals<<normal.normal[0],normal.normal[1],normal.normal[2];
                    int angle_z = degree(acos(normals.dot(v)));
                    if(angle_z>=k_AngleMin&&angle_z<=k_AngleMax)
                    {
                        if(viz==true)
                            voxel->good = true;
                        good_points.push_back(centroid_hash);
                        break;
                    }
                }
            }

        }
    }
    return make_pair(found,good_points);
}


//Raytracing from the depth map.
void RayTracingEngine::rayTrace(VoxelVolume& volume,Eigen::Affine3f& transformation,int zdelta = 10,bool sparse=true)
{
    int width = cam_.getWidth();
    int height = cam_.getHeight();
    bool found[cam_.getHeight()][cam_.getWidth()]={false};
    unordered_set<unsigned long long int> checked;
    int rdelta = 1, cdelta = 1;
    if(sparse==true)
    {
        rdelta=5;
        cdelta=5;
    }
    for(int z_depth=10;z_depth<k_ZMax*1000;z_depth+=zdelta)
    {
        for(int r=0;r<height;r+=rdelta)
        { 
            for(int c=0;c<width;c+=cdelta)
            {   
                if(found[r][c])
                    continue;
                double x,y,z;
                tie(x,y,z) = cam_.projectPoint(r,c,z_depth);
                tie(x,y,z) = cam_.transformPoints(x,y,z,transformation);
                if(volume.validPoints(x,y,z)==false)
                    continue;
                auto coords = volume.getVoxel(x,y,z);
                int xid = get<0>(coords);
                int yid = get<1>(coords);
                int zid = get<2>(coords);
                auto id = volume.getHashId(xid,yid,zid);
                Voxel *voxel = volume.voxels_[xid][yid][zid];
                if(voxel!=nullptr)
                {
                    found[r][c]=true;
                    voxel->view=1;//TODO: Change it to view number.
                    if(checked.find(id)==checked.end())
                        checked.insert(id);
                }
            }
        }
    }
}

void RayTracingEngine::rayTraceAndClassify(VoxelVolume& volume,Eigen::Affine3f& transformation,int zdelta=10,int view = 1,bool sparse=true)
{
    int width = cam_.getWidth();
    int height = cam_.getHeight();
    bool found[cam_.getHeight()][cam_.getWidth()]={false};
    Eigen::Affine3f normals_transformation = Eigen::Affine3f::Identity();
    Eigen::Affine3f inverse_transformation = transformation.inverse();
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            normals_transformation(i,j)=inverse_transformation(i,j);
    int rdelta = 1, cdelta = 1;
    if(sparse==true)
    {
        rdelta=5;
        cdelta=5;
    }
    for(int z_depth=10;z_depth<k_ZMax*1000;z_depth+=zdelta)
    {
        for(int r=0;r<height;r+=rdelta)
        { 
            for(int c=0;c<width;c+=cdelta)
            {   
                if(found[r][c])
                    continue;
                double x,y,z;
                tie(x,y,z) = cam_.projectPoint(r,c,z_depth);
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
                    if(voxel->view==0)
                        voxel->view=view;//TODO: Change it to view number.
                    if(voxel->good==false)
                    {
                        for(auto normal:voxel->normals)
                        {
                            double nx,ny,nz;
                            tie(nx,ny,nz) = cam_.transformPoints(normal.normal[0],normal.normal[1],normal.normal[2],normals_transformation);
                            double nml[3]={nx,ny,nz};
                            int angle_z = angle(nml);
                            if(z_depth>=250&&z_depth<=600)
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

std::pair<bool,std::vector<unsigned long long int>> RayTracingEngine::rayTraceAndGetGoodPoints(VoxelVolume& volume,Eigen::Affine3f& transformation,int zdelta=10,bool sparse=true)
{
    int width = cam_.getWidth();
    int height = cam_.getHeight();
    bool found[cam_.getHeight()][cam_.getWidth()]={false};
    Eigen::Affine3f normals_transformation = Eigen::Affine3f::Identity();
    Eigen::Affine3f inverse_transformation = transformation.inverse();
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            normals_transformation(i,j)=inverse_transformation(i,j);
    int rdelta = 1, cdelta = 1;
    if(sparse==true)
    {
        rdelta=5;
        cdelta=5;
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
                double x,y,z;
                tie(x,y,z) = cam_.projectPoint(r,c,z_depth);
                tie(x,y,z) = cam_.transformPoints(x,y,z,transformation);
                if(volume.validPoints(x,y,z)==false)
                    continue;
                auto coords = volume.getVoxel(x,y,z);
                int xid = get<0>(coords);
                int yid = get<1>(coords);
                int zid = get<2>(coords);
                auto id = volume.getHashId(xid,yid,zid);
                Voxel *voxel = volume.voxels_[xid][yid][zid];
                if(voxel!=nullptr)
                {
                    found[r][c]=true;
                    point_found = true;
                    for(auto normal:voxel->normals)
                    {
                        double nx,ny,nz;
                        tie(nx,ny,nz) = cam_.transformPoints(normal.normal[0],normal.normal[1],normal.normal[2],normals_transformation);
                        double nml[3]={nx,ny,nz};
                        int angle_z = angle(nml);
                        if(z_depth>=250&&z_depth<=600)
                        {
                            if(checked.find(id)==checked.end())
                            {
                                checked.insert(id);
                                good_points.push_back(id);
                            }
                            break;
                        }
                    }
                }
            }
        }
    }
    return make_pair(point_found,good_points);
}

std::pair<bool,std::vector<unsigned long long int>> RayTracingEngine::rayTraceAndGetPoints(VoxelVolume& volume,Eigen::Affine3f& transformation,int zdelta=10,bool sparse=true)
{
    int width = cam_.getWidth();
    int height = cam_.getHeight();
    bool found[cam_.getHeight()][cam_.getWidth()]={false};
    int rdelta = 1, cdelta = 1;
    if(sparse==true)
    {
        rdelta=5;
        cdelta=5;
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
                double x,y,z;
                tie(x,y,z) = cam_.projectPoint(r,c,z_depth);
                tie(x,y,z) = cam_.transformPoints(x,y,z,transformation);
                if(volume.validPoints(x,y,z)==false)
                    continue;
                auto coords = volume.getVoxel(x,y,z);
                int xid = get<0>(coords);
                int yid = get<1>(coords);
                int zid = get<2>(coords);
                auto id = volume.getHashId(xid,yid,zid);
                Voxel *voxel = volume.voxels_[xid][yid][zid];
                if(voxel!=nullptr)
                {
                    found[r][c]=true;
                    point_found = true;
                    if(checked.find(id)==checked.end())
                    {
                        checked.insert(id);
                        good_points.push_back(id);
                    }
                }
            }
        }
    }
    return make_pair(point_found,good_points);
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
