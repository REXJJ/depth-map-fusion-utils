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
            if(max_points<5)
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

    vector<double> movePointAway(vector<double> pi,vector<double> nor,double distance)
    {
        Vector3f p1(3);
        Vector3f n(3);
        p1<<pi[0],pi[1],pi[2];
        n<<nor[0],nor[1],nor[2];
        n = n*distance;
        return {n(0)+pi[0],n(1)+pi[1],n(2)+pi[2]};
    }  

    Affine3f positionCamera(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr locations,int id,unsigned int distance=300)
    {
        pcl::PointXYZRGBNormal pt = locations->points[id];
        Vector3f nor(3);
        nor<<pt.normal[0],pt.normal[1],pt.normal[2];
        vector<double> normals = {nor(0),nor(1),nor(2)};
        nor=nor*-1;
        Vector3f x(3);
        if(nor(2)!=0.0)
        {
            x<<1,1,0;
            x(2) = -(nor(0)+nor(1))/nor(2);
        }
        else if(nor(1)!=0)
        {
            std::cout<<"Zero Z component and non-zero y."<<std::endl;
            x<<1,0,1;
            x(1) = -(nor(0)+nor(2))/nor(1);
        }
        else if(nor(0)!=0)
        {
            std::cout<<"Zero Z component and non-zero x."<<std::endl;
            x<<0,1,1;
            x(0) = -(nor(1)+nor(2))/nor(0);
        }
        else
        {
            std::cout<<"Serious Bug. Presence of zero normals detected. Check the data."<<std::endl;
        }
        x = x.normalized();
        Vector3f y = nor.cross(x);
        Affine3f Q = Eigen::Affine3f::Identity();
        auto new_points = movePointAway({pt.x,pt.y,pt.z},normals,double(distance)/1000.0);
        for(int i=0;i<3;i++)
        {
            Q(i,0) = x(i);
            Q(i,1) = y(i);
            Q(i,2) = nor(i);
            Q(i,3) = new_points[i];
        }
        return Q;
    }

    vector<Affine3f> positionCameras(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr locations,unsigned int distance = 300)
    {
        vector<Affine3f> cameras;
        for(int i=0;i<locations->points.size();i++)
        {
            auto Q = positionCamera(locations,i,distance);
            cameras.push_back(Q);
        }
        return cameras;
    }

    Affine3f optimizeCameraPosition(VoxelVolume &volume,RayTracingEngine engine, int resolution_single_dimension, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr locations,int id)
    {
        unsigned int low = 300;
        unsigned int high = 600;
        unsigned int mid = low;
        while(low<high)
        {
            auto camera_location = positionCamera(locations,id,low);
            bool found;
            vector<unsigned long long int> good_points_low,good_points_high;
            // tie(found,good_points_low) = engine.rayTraceAndGetGoodPoints(volume,camera_location,resolution_single_dimension);
            tie(found,good_points_low) = engine.reverseRayTrace(volume,camera_location,false);
            camera_location = positionCamera(locations,id,high);
            // tie(found,good_points_high) = engine.rayTraceAndGetGoodPoints(volume,camera_location,resolution_single_dimension);
            tie(found,good_points_high) = engine.reverseRayTrace(volume,camera_location,false);
            mid = (low+high)/2;
            if(good_points_high.size()>good_points_low.size())
            {
                low = mid+1;
            }
            else
            {
                high = mid;
            }
            cout<<"Low High Mid: "<<low<<" "<<high<<" "<<mid<<endl;
        }
        auto Q = positionCamera(locations,id,mid);
        return Q;
    }
};

