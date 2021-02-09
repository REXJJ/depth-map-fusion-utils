#include <bits/stdc++.h> 
#include <limits.h> 
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>

#include <TSPSolver.hpp>

using namespace boost::algorithm;
using namespace std;
using namespace Eigen;

using namespace std; 
using namespace TSP;

int main(int argc,char** argv) 
{ 
    vector<vector<int>> map =  { { 0, 10, 15, 20 },
                       { 10, 0, 35, 25 },
                       { 15, 35, 0, 30 },
                       { 20, 25, 30, 0 } };
    TwoOptSearch solver(map);
    solver.solve();
    solver.printPath();
    std::cout<<solver.getPathLength()<<std::endl;
    return 0;
} 

