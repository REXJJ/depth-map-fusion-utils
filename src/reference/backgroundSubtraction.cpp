#include <cstdio>
#include <flann/flann.hpp>
#include <vector>
#include <ctime>
#include "Open3D/Open3D.h"
#include <chrono>
/*****************************************************************/
//Color Definitions
/****************************************************************/
#define _NORMAL_    "\x1b[0m"
#define _BLACK_     "\x1b[30;47m"
#define _RED_       "\x1b[31;40m"
#define _GREEN_     "\x1b[32;40m"
#define _YELLOW_    "\x1b[33;40m"
#define _BLUE_      "\x1b[34;40m"
#define _MAGENTA_   "\x1b[35;40m"
#define _CYAN_      "\x1b[36;40m"
#define _WHITE_     "\x1b[37;40m"
#define _BRED_      "\x1b[1;31;40m"
#define _BGREEN_    "\x1b[1;32;40m"
#define _BYELLOW_   "\x1b[1;33;40m"
#define _BBLUE_     "\x1b[1;34;40m"
#define _BMAGENTA_  "\x1b[1;35;40m"
#define _BCYAN_     "\x1b[1;36;40m"
#define _BWHITE_    "\x1b[1;37;40m"
/******************************************************************/
//Print Utilities for debugging.
/*******************************************************************/
#define dbg_color(...) do { printf(__VA_ARGS__); } while(0)

#define tic() std::chrono::high_resolution_clock::time_point END_TIME,START_TIME=std::chrono::high_resolution_clock::now()

#define reset() START_TIME=std::chrono::high_resolution_clock::now()

#define toc()                                                   \
do                                                              \
{                                                               \
  dbg_color(_GREEN_);                                         \
    END_TIME=std::chrono::high_resolution_clock::now();         \
    std::cout << " Time elapsed in "<<__func__<<"() : "         \
    << std::chrono::duration_cast<std::chrono::microseconds>(END_TIME - START_TIME).count() << " microseconds.\n"; \
    dbg_color(_NORMAL_);                                        \
}while(0)


int main(int argc, char **argv) {
    using namespace open3d;
    using namespace flann;

    if (argc < 3) {
        PrintOpen3DVersion();
        utility::LogInfo("Usage:");
        utility::LogInfo("    > Flann [filename]");
        return 1;
    }

    auto cloud_ptr = std::make_shared<geometry::PointCloud>();
    if (io::ReadPointCloud(argv[1], *cloud_ptr)) {
        utility::LogInfo("Successfully read {}", argv[1]);
    } else {
        utility::LogWarning("Failed to read {}", argv[1]);
        return 1;
    }

    if ((int)cloud_ptr->points_.size() < 100) {
        utility::LogWarning("Boring point cloud.");
        return 1;
    }

    if (cloud_ptr->HasColors() == false) {
        cloud_ptr->colors_.resize(cloud_ptr->points_.size());
        for (size_t i = 0; i < cloud_ptr->points_.size(); i++) {
            cloud_ptr->colors_[i].setZero();
        }
    }

    auto cloud_ptr_obj = std::make_shared<geometry::PointCloud>();
    if (io::ReadPointCloud(argv[2], *cloud_ptr_obj)) {
        utility::LogInfo("Successfully read {}", argv[1]);
    } else {
        utility::LogWarning("Failed to read {}", argv[1]);
        return 1;
    }

    if ((int)cloud_ptr_obj->points_.size() < 100) {
        utility::LogWarning("Boring point cloud.");
        return 1;
    }

    if (cloud_ptr_obj->HasColors() == false) {
        cloud_ptr_obj->colors_.resize(cloud_ptr_obj->points_.size());
        for (size_t i = 0; i < cloud_ptr_obj->points_.size(); i++) {
            cloud_ptr_obj->colors_[i].setZero();
        }
    }

    tic();
    int nn=20;
    geometry::KDTreeFlann kdtree;
    kdtree.SetGeometry(*cloud_ptr);
    toc();
    reset();
    std::vector<int> new_indices_vec(nn);
    std::vector<double> new_dists_vec(nn);
    float r = 0.008;
    auto cloud_ptr_cleaned = std::make_shared<geometry::PointCloud>();
    cloud_ptr_cleaned->points_.resize(cloud_ptr_obj->points_.size());
    cloud_ptr_cleaned->colors_.resize(cloud_ptr_obj->colors_.size());
    toc();
    reset();
    int t=0;
    for(int i=0;i<cloud_ptr_obj->points_.size();i++)
    {
	    int k = kdtree.Search(cloud_ptr_obj->points_[i],
			    geometry::KDTreeSearchParamRadius(r), new_indices_vec,
			    new_dists_vec);
	    if(k==0)
	    {
		    cloud_ptr_cleaned->points_[t]=cloud_ptr_obj->points_[i];
		    cloud_ptr_cleaned->colors_[t]=cloud_ptr_obj->colors_[i];
		    t++;
	    }
    }
    cloud_ptr_cleaned->points_.resize(t);
    cloud_ptr_cleaned->colors_.resize(t);
    toc();

    // visualization::DrawGeometries({cloud_ptr_cleaned}, "TestKDTree", 1600, 900);
    return 0;
}

