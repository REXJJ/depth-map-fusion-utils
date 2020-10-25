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

constexpr int degree(double radian){return int((radian*180)/3.14159);};
constexpr double magnitude(double normal[3]){return normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2];};
constexpr int angle(double normal[3]){return degree(acos(-normal[2]/magnitude(normal)));};
template <typename T> constexpr int sgn(T x) {
    return (T(0) < x) - (x < T(0));
}

std::pair<Vector3f, Vector3f> planeFitting(const std::vector<Vector3f> & c)
{
    // copy coordinates to  matrix in Eigen format
    size_t num_atoms = c.size();
    Eigen::Matrix< Vector3f::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_atoms);
    for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];
    // calculate centroid
    Vector3f centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());
    // subtract centroid
    coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);
    // we only need the left-singular matrix here
    //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
    auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Vector3f plane_normal = svd.matrixU().rightCols<1>();
    return std::make_pair(centroid, plane_normal);
}

