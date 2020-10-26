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

using namespace pcl;

void readPointCloud(string filename,pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals=nullptr)
{
    cout<<"Inside reading function"<<endl;
#if 1
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (filename, *cloud_normal) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return;
    }
#else
    pcl::PLYReader Reader;
    Reader.read(string(argv[1]), *cloud);
#endif
    cout<<"Parsing the pointcloud"<<endl;
    for(int i=0;i<cloud_normal->points.size();i++)
    {
        PointXYZRGBNormal pt = cloud_normal->points[i];
        PointXYZRGB pt_rgb;
        Normal pt_n;
        pt_rgb.x = pt.x;
        pt_rgb.y = pt.y;
        pt_rgb.z = pt.z;
        pt_rgb.r = 0;
        pt_rgb.g = 0;
        pt_rgb.b = 0;
        // pt_n.normal = pt.normal;
        pt_n.normal[0] = pt.normal[0];
        pt_n.normal[1] = pt.normal[1];
        pt_n.normal[2] = pt.normal[2];
        cloud->points.push_back(pt_rgb);
        if(normals!=nullptr)
            normals->points.push_back(pt_n);
    }
    cout<<"Pointcloud Parsed"<<endl;
}

vector<Eigen::Affine3f> readCameraLocations(string filename)
{
    vector<Eigen::Affine3f> transformations;
    ifstream file(filename);
    string line;
    getline(file,line);
    int length = stoi(line);
    cout<<"The Length: "<<length<<endl;
    for(int i=0;i<length;i++)
    {
        Eigen::Affine3f temp = Eigen::Affine3f::Identity();
        for(int j=0;j<3;j++)
        {
            getline(file,line);
            std::cout<<line<<endl;
            vector<string> numbers;
            boost::split(numbers,line,boost::is_any_of(","));
            for(auto x:numbers)
                cout<<x<<" ";
            cout<<endl;
            for(int k=0;k<4;k++)
                temp(j,k) = stof(numbers[k]);
        }
        transformations.push_back(temp);
    }
    return transformations;
}

void writeCameraLocations(string filename, vector<Eigen::Affine3f> transformations)
{
    ofstream file(filename);
    file<<transformations.size()<<endl;
    for(int i=0;i<transformations.size();i++)
    {
        for(int j=0;j<3;j++)
        {
            file<<transformations[i](j,0);
            for(int k = 1;k<4;k++)
                file<<","<<transformations[i](j,k);
            file<<endl;
        }
    }
}
