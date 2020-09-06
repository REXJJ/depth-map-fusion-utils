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

class RayTracingEngine
{
    public:
        int height_,width_;
        float fx_,fy_,cx_,cy_;
        RayTracingEngine(int height,int width,float fx,float fy,float cx,float cy);
        tuple<float,float,float> projectPoint(int r,int c,int z);
};

RayTracingEngine::RayTracingEngine(int height,int width,float fx,float fy,float cx,float cy)
    :height_(height),
    width_(width),
    fx_(fx),
    fy_(fy),
    cx_(cx),
    cy_(cy){}

tuple<float,float,float> RayTracingEngine::projectPoint(int r,int c,int depth_mm)
{
    float x,y,z;
    z = depth_mm * 0.001;
    x = z * ( (double)c - cx_ ) / (fx_);
    y = z * ((double)r - cy_ ) / (fy_);
    return {x,y,z};
}

