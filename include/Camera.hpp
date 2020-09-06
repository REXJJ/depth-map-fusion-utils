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

using namespace std;

class Camera
{
    vector<float> K_;
    int height_,width_;
    public:
    Camera(vector<float>& K,int height=480,int width=640):K_(K),height_(height),width_(width){};
    tuple<float,float,float> projectPoint(int r,int c,int depth_mm)
    {
        double fx=K_[0],cx=K_[2],fy=K_[4],cy=K_[5];
        double z = depth_mm * 0.001;
        double x = z * ( (double)c - cx ) / (fx);
        double y = z * ((double)r - cy ) / (fy);
        return {x,y,z};
    }
    tuple<int,int> deProjectPoint(double x,double y,double z)
    {
        double fx=K_[0],cx=K_[2],fy=K_[4],cy=K_[5];
        int c = (x * fx)/z + cx;
        int r = (y * fy)/z + cy;
        return {r,c};
    }
    tuple<float,float,float> transformPoints(double x,double y,double z,Eigen::Affine3f& transformation)
    {
        Eigen::Vector3f p1(3);
        p1<<x,y,z;
        Eigen::Vector3f p2 = transformation*p1;
        return {p2(0),p2(1),p2(2)};
    }
    tuple<float,float,float> getPoint(int r,int c,int depth_mm)
    {
        auto [x,y,z] = projectPoint(r,c,depth_mm);
        return {x,y,z};
    }
    tuple<int,int> getPixel(double x,double y,double z,Eigen::Affine3f transformation=Eigen::Affine3f::Identity())
    {
        std::tie(x,y,z) = transformPoints(x,y,z,transformation);
        return deProjectPoint(x,y,z);
    }
    int getHeight()
    {
        return height_;
    }
    int getWidth()
    {
        return width_;
    }
    bool validPixel(int r,int c)
    {
        return (r>=0&&r<height_&&c>=0&&c<width_);
    }
};

