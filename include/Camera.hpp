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
    Camera(){}
    Camera(vector<float>& K,int height=480,int width=640):K_(K),height_(height),width_(width){};
    tuple<float,float,float> projectPoint(int r,int c,int depth_mm)
    {
        double fx=K_[0],cx=K_[2],fy=K_[4],cy=K_[5];
        double z = depth_mm * 0.001;
        double x = z * ( (double)c - cx ) / (fx);
        double y = z * ((double)r - cy ) / (fy);
        return make_tuple(x,y,z);
    }
    tuple<int,int> deProjectPoint(double x,double y,double z)
    {
        double fx=K_[0],cx=K_[2],fy=K_[4],cy=K_[5];
        int c = int(round((x * fx)/z + cx));
        int r = int(round((y * fy)/z + cy));
        return make_tuple(r,c);
    }
    tuple<float,float,float> transformPoints(double x,double y,double z,Eigen::Affine3f& transformation)
    {
        Eigen::Vector3f p1(3);
        p1<<x,y,z;
        Eigen::Vector3f p2 = transformation*p1;
        return make_tuple(p2(0),p2(1),p2(2));
    }
    tuple<float,float,float> getPoint(int r,int c,int depth_mm)
    {
        double x,y,z;
        std::tie(x,y,z) = projectPoint(r,c,depth_mm);
        return make_tuple(x,y,z);
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
    float getAreaCovered(int depth_mm)
    {
        double x1,y1,x2,y2,x3,y3,z;
        std::tie(x1,y1,z) = getPoint(0,0,depth_mm);
        std::tie(x2,y2,z) = getPoint(0,height_,depth_mm);
        std::tie(x3,y3,z) = getPoint(width_,0,depth_mm);
        auto distance = [](double x1,double y1,double x2,double y2){return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));};
        return distance(x1,y1,x2,y2)*distance(x1,y1,x3,y3);
    }
    float getDistance(int depth_mm)
    {
        double x1,x2,y1,y2,z;
        std::tie(x1,y1,z) = getPoint(100,100,depth_mm);
        std::tie(x2,y2,z) = getPoint(101,101,depth_mm);
        auto distance = [](double x1,double y1,double x2,double y2){return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));};
        return distance(x1,y1,x2,y2);
    }
};

