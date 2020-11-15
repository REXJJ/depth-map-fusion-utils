#pragma once

#include <nlopt.hpp>
#include <iostream>
#include <vector>

using namespace std;

namespace TSP
{
    class GeneticAlgorithm
    {
        public:
            double objectiveFunction(vector<double> path);
            GeneticAlgorithm(vector<vector<int>> map);
            vector<vector<int>> map_;
            void solve();
    };
}
