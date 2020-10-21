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
using namespace Eigen;

namespace Algorithms
{
    vector<unsigned long long int> greedySetCover(vector<vector<unsigned long long int>> &candidate_sets,double resolution = 0.000008)
    {
        vector<unsigned long long int> covered;
        vector<unsigned long long int> selected_sets;
        vector<unsigned long long int> set_ids(candidate_sets.size());
        iota(set_ids.begin(),set_ids.end(),0);
        // double volume_threshold = 1e-6; 
        double volume_threshold = 27e-6; 
        // double volume_threshold = 1.25e-4; 
        cout<<"Volume Threshold: "<<volume_threshold<<endl;
        cout<<volume_threshold<<endl;
        while(true)
        {
            unsigned long long int selected=-1;
            double total_volume_increase = 0;
            unsigned long long int max_points = 0;
            for(auto x:set_ids)
            {
                vector<unsigned long long int> difference;
                std::set_difference(candidate_sets[x].begin(),candidate_sets[x].end(),covered.begin(),covered.end(),std::inserter(difference,difference.begin()));
                if(difference.size()*resolution>total_volume_increase)
                    total_volume_increase = difference.size()*resolution;
                if(difference.size()>max_points)
                {
                    max_points = difference.size();
                    selected = x;
                }
            }
            // if(total_volume_increase<volume_threshold)
            // {
            //     cout<<"Total Volume Increase: "<<total_volume_increase<<endl;
            //     break;
            // }
            if(selected==-1)
                break;
            if(max_points<10)
                break;
            std::cout<<"Max Points: "<<max_points<<" Selected: "<<selected<<std::endl;
            vector<unsigned long long int> difference;
            std::set_difference(candidate_sets[selected].begin(),candidate_sets[selected].end(),covered.begin(),covered.end(),std::inserter(difference,difference.begin()));
            for(auto x:difference)
                covered.push_back(x);
            sort(covered.begin(),covered.end());
            selected_sets.push_back(selected);
            vector<unsigned long long int> temp;
            set_ids.erase(std::remove(set_ids.begin(), set_ids.end(),selected), set_ids.end());
        }
        return selected_sets;
    }

#if 0
    void generateSphere(double radius,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere,Eigen::Affine3f transformation=Eigen::Affine3f::Identity())
    {
        for(double r=radius;r>=0;r-=0.05)
        {
            for(double x=-r;x<=r;x+=0.05)
            {
                double y=sqrt(r*r-(x*x));
                PointXYZRGB pt;
                pt.x=x;
                pt.y=y;
                pt.z=sqrt(radius*radius-(x*x)-(y*y));
                pt.r=0;
                pt.g=255;
                pt.b=0;
                sphere->points.push_back(pt);
                pt.y=-y;
                sphere->points.push_back(pt);
            }
        }
        for(int i=0;i<sphere->points.size();i++)
        {
            pcl::PointXYZRGB pt = sphere->points[i];
            Vector3f point(3);
            point<<pt.x,pt.y,pt.z;
            Vector3f transformed = transformation*point;
            sphere->points[i].x = transformed(0);
            sphere->points[i].y = transformed(1);
            sphere->points[i].z = transformed(2);
        }
    }
#else
    void generateSphere(double radius,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere,Eigen::Affine3f transformation=Eigen::Affine3f::Identity())
    {
        const double PI = 3.141592653589793238462643383279502884197;
        // Iterate through phi, theta then convert r,theta,phi to  XYZ
        const double factor = 5.0;
        for (double phi = 0.; phi <= 2*PI; phi += PI/factor) // Azimuth [0,PI]
        {
            for (double theta = 0.; theta <= PI; theta += PI/factor) // Elevation [0, PI]
            {
                pcl::PointXYZRGB point;
                point.x = radius * cos(phi) * sin(theta);
                point.y = radius * sin(phi) * sin(theta);
                point.z = radius            * cos(theta);
                point.r = 0;
                point.g = 0;
                point.b = 0;
                sphere->points.push_back(point);        
            }
        }
        for(int i=0;i<sphere->points.size();i++)
        {
            pcl::PointXYZRGB pt = sphere->points[i];
            Vector3f point(3);
            point<<pt.x,pt.y,pt.z;
            Vector3f transformed = transformation*point;
            sphere->points[i].x = transformed(0);
            sphere->points[i].y = transformed(1);
            sphere->points[i].z = transformed(2);
        }
        cout<<"Generated "<<sphere->points.size()<<" points.."<<endl;
    }
#endif
};

