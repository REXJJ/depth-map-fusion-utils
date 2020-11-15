// C++ implementation of the above approach 
#include <bits/stdc++.h> 
#include <limits.h> 
/*********************************************/
//OTHER HEADERS
/**********************************************/
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>

#include <TSPSolver.hpp>
#include <nlopt.hpp>

using namespace boost::algorithm;
using namespace std;
using namespace Eigen;

using namespace std; 

namespace TSP
{
    double minFunc(const std::vector<double>& x, std::vector<double>& grad, void* data) 
    {
        GeneticAlgorithm *c = (GeneticAlgorithm *) data;
        return c->objectiveFunction(x);
    };
    double GeneticAlgorithm::objectiveFunction(vector<double> path)
    {
        double distance = 0;
        for(int i=0;i<path.size()-1;i++)
        {
            distance += map_[path[i]][path[i+1]];
        }
        return distance;
    }
    GeneticAlgorithm::GeneticAlgorithm(vector<vector<int>> map)
    {
        map_ = map;
    }
    void GeneticAlgorithm::solve()
    {
        nlopt::algorithm optalg = nlopt::GN_ESCH;
        nlopt::opt opt = nlopt::opt(optalg, map_.size());
        opt.set_min_objective(minFunc, this);
        opt.set_ftol_rel(1e-8);
        opt.set_maxeval(1000);
        std::vector<double> x0(map_.size());
        for(int i=0;i<x0.size();i++)
            x0[i] = i;
        auto solx = x0;
        double solminf = 0;
        int successFlag = 0;
        try{
            nlopt::result result = opt.optimize(solx, solminf);
            successFlag = 1;
        }
        catch(std::exception &e) {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }
    }
}

int main(int argc,char** argv) 
{ 
    vector<vector<int>> map =  { { 0, 10, 15, 20 },
                       { 10, 0, 35, 25 },
                       { 15, 35, 0, 30 },
                       { 20, 25, 30, 0 } };

    TSP::GeneticAlgorithm tsp(map);
    tsp.solve();
    return 0;
} 

