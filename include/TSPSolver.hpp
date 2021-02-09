#pragma once

#include <nlopt.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <unordered_set>
#include <assert.h>

using namespace std;

namespace TSP
{
    class GreedySolver
    {
        public:
        vector<vector<int>> map_;
        vector<int> path_;
        GreedySolver(vector<vector<int>>& );
        void nearestNeighborSearch();
        void printPath();
        vector<int> getPath(){ return path_; }
        void setPath(vector<int>& path) { path_ = path; }
        int getPathLength() 
        {
            int distance = 0;
            for(int i=1;i<path_.size();i++)
                distance+=map_[path_[i-1]][path_[i]];
            return distance;
        }
        int getPathLength(vector<int>& path)
        {
            int distance = 0;
            for(int i=1;i<path.size();i++)
            {
                if(map_[path[i-1]][path[i]]==INT_MAX)
                    return INT_MAX;
                distance+=map_[path[i-1]][path[i]];
            }
            return distance;
        }
    };

    GreedySolver::GreedySolver(vector<vector<int>>& map)
    {
        map_ = map;
        path_.resize(map_.size());
        std::iota(path_.begin(),path_.end(),0);
    }

    void GreedySolver::printPath()
    {
        for(auto x:path_)
            std::cout<<x<<" ";
        std::cout<<std::endl;
    }

    class NearestNeighborSearch : public GreedySolver
    {
        private:
            unordered_set<int> s;
            int start_,path_len_;
        public:
            NearestNeighborSearch(vector<vector<int>>& map);
            void solve();
    };

    NearestNeighborSearch::NearestNeighborSearch(vector<vector<int>>& map) : GreedySolver(map)
    {
        start_ = 0;
        path_len_ = 1;
        s.insert(0);
    }

    void NearestNeighborSearch::solve()
    {
        if(path_len_==map_.size())
            return;
        int shortest = INT_MAX;
        int selected = -1;
        for(int i=0;i<map_.size();i++)
            if(s.find(i)==s.end())
            {
                if(i!=start_&&map_[start_][i]<shortest)    
                {
                    shortest = map_[start_][i];
                    selected = i;
                }

            }
        assert(selected>=0);
        path_[path_len_++] = selected;
        start_ = selected;
        s.insert(selected);
        std::cout<<selected<<std::endl;
        solve();
    }

    class TwoOptSearch : public GreedySolver
    {
        public:
            void solve();
            TwoOptSearch(vector<vector<int>>& map); 
            vector<int> twoOptSwap(vector<int> path, int i, int k);
    };

    TwoOptSearch::TwoOptSearch(vector<vector<int>>& map): GreedySolver(map)
    {

    }

    vector<int> TwoOptSearch::twoOptSwap(vector<int> path, int i, int k)
    {
        vector<int> new_path;
        // std::cout<<"i,k: "<<i<<" "<<k<<std::endl;

        for(int j=0;j<i;j++)
            new_path.push_back(path[j]);
        for(int j=k;j>=i;j--)
            new_path.push_back(path[j]);
        for(int j=k+1;j<path.size();j++)
            new_path.push_back(path[j]);
        // std::cout<<path.size()<<" "<<new_path.size()<<std::endl;

        assert(new_path.size()==path.size());
        unordered_set<int> s;
        for(auto x:new_path)
        {
            assert(s.find(x)==s.end());
            s.insert(x);
        }
        return new_path;
    }

    void TwoOptSearch::solve()
    {
        auto path = path_;
start:
        int best_distance = getPathLength(path);
        for(int i=1;i<path_.size();i++)
        {
            for(int k=i+1;k<path_.size();k++)
            {
                auto new_route = twoOptSwap(path,i,k);
                auto new_distance = getPathLength(new_route);
                if(new_distance<best_distance)
                {
                    path = new_route;
                    best_distance = new_distance;
                    goto start;
                }
            }
        }
        path_ = path;
    }

};
