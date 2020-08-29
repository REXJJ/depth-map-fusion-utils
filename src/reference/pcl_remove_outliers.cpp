/***********************************************/
//STANDARD HEADERS
/************************************************/
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
#include <bits/stdc++.h>

/*********************************************/
//PCL HEADERS
/**********************************************/
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include<pcl/io/ply_io.h>
#include <pcl/surface/convex_hull.h>
/*********************************************/
//OTHER HEADERS
/**********************************************/
#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/ring.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <iostream>

using namespace std;
namespace bg = boost::geometry;

BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)
typedef boost::tuple<float, float> point;
typedef bg::model::ring<point> ring;

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace cv;

template <typename PointT> static void visualizePointCloud(const pcl::PointCloud<PointT>& cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointT> (cloud.makeShared(), "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    // viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);

    }
}
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



void print(){std::cout<<std::endl;}
    template<typename T,typename... Args>
void print(T Contents, Args... args) 
{
    std::cout<< (Contents) <<" ";print(args...);
}


void pclXYZRGBNormalToPly(const std::string filename,pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud)
{
    ofstream f(filename);
    string header="property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty float nx\nproperty float ny\nproperty float nz\nend_header\n";
    f<<"ply\nformat ascii 1.0\n";
    f<<"element vertex ";
    f<<(to_string(cloud.points.size())+"\n");
    f<<header;
    for(auto pt:cloud.points)
    {
        f<<pt.x<<" "<<pt.y<<" "<<pt.z<<" "<<static_cast<int>(pt.r)<<" "<<static_cast<int>(pt.g)<<" "<<static_cast<int>(pt.b)<<" "<<pt.normal[0]<<" "<<pt.normal[1]<<" "<<pt.normal[2]<<"\n";
    }
    f.close();
}

template <typename PointT> void visualizePolygon(const pcl::PointCloud<PointT>& cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    for(int i=0;i<cloud.points.size()-1;i++)
        viewer->addLine<PointT> ((cloud)[i],
                (cloud)[i+ 1], "line"+to_string(i));

    viewer->addLine<PointT> ((cloud)[0],
            (cloud)[cloud.points.size()- 1], "line");
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);

    }
}
bool lines_intersect(double l1[2][2], double l2[2][2])
{
	// l1 for horizontal ray line...slope is always zero

	// checking if other slope is zero
	if (l2[0][1]==l2[1][1])
	{
		return false;
	}
	else
	{
		// checking both pts of second line above first line
		if ((l2[0][1]>l1[0][1] && l2[1][1]>l1[0][1]) || (l2[0][1]<l1[0][1] && l2[1][1]<l1[0][1]))
		{
			return false;
		}
		else
		{
			// checking both pts of second line either on right or on left of fist line
			if ((l2[0][0]<l1[0][0] && l2[1][0]<l1[0][0]) || (l2[0][0]>l1[1][0] && l2[1][0]>l1[1][0]))
			{
				return false;
			}
			else
			{
				// checking if other line is vertical
				if (l2[0][0]== l2[1][0])
				{
					return true;
				}
				else
				{
					// getting intersection point
					double m2 = (l2[1][1]-l2[0][1])/(l2[1][0]-l2[0][0]);        
					double x = (l1[0][1]+m2*l2[0][0]-l2[0][1])/m2;
					// checking if intersection point lies on the first line
					if ((x>l1[0][0] || std::abs(x-l1[0][0])<1e-9) && (x<l1[1][0] || std::abs(x-l1[1][0])<1e-9))
					{
						return true;
					}
					else
					{
						return false;
					}
				}
			}
		}
	} 
	return false;
}

Eigen::MatrixXd InPoly(Eigen::MatrixXd& q, Eigen::MatrixXd& p)
{
	// p : polygon points
	// q : query points

	double l1[2][2];
	double l2[2][2];

	Eigen::MatrixXd in = Eigen::VectorXd::Constant(q.rows(),1,0);

	double xmin = p.col(0).minCoeff();
	double xmax = p.col(0).maxCoeff();
	double ymin = p.col(1).minCoeff();
	double ymax = p.col(1).maxCoeff();

	for (long i=0;i<q.rows();++i)
	{
		// bounding box test
		if (q(i,0)<xmin || q(i,0)>xmax || q(i,1)<ymin || q(i,1)>ymax)
		{
			continue;
		}
		int intersection_count = 0;
		Eigen::MatrixXd cont_lines = Eigen::MatrixXd::Constant(p.rows(),1,0);
		for (int j=0;j<p.rows();++j)
		{
			if (j==0)
			{
				l1[0][0] = q(i,0);l1[0][1] = q(i,1);
				l1[1][0] = xmax;l1[1][1] = q(i,1);
				l2[0][0] = p(p.rows()-1,0);l2[0][1] = p(p.rows()-1,1);
				l2[1][0] = p(j,0);l2[1][1] = p(j,1);
				if (lines_intersect(l1,l2))
				{
					intersection_count++;
					cont_lines(j,0) = 1;
				}   
			}
			else
			{
				l1[0][0] = q(i,0);l1[0][1] = q(i,1);
				l1[1][0] = xmax;l1[1][1] = q(i,1);
				l2[0][0] = p(j,0);l2[0][1] = p(j,1);
				l2[1][0] = p(j-1,0);l2[1][1] = p(j-1,1);
				if (lines_intersect(l1,l2))
				{
					intersection_count++;
					cont_lines(j,0) = 1;
					if (cont_lines(j-1,0)==1)
					{
						if (p(j-1,1)==q(i,1))
						{
							if (j-1==0)
							{
								if (!((p(p.rows()-1,1)<p(j-1,1) && p(j,1)<p(j-1,1)) || (p(p.rows()-1,1)>p(j-1,1) && p(j,1)>p(j-1,1))))
								{
									intersection_count--;
								}
							}
							else
							{
								if (!((p(j-2,1)<p(j-1,1) && p(j,1)<p(j-1,1)) || (p(j-2,1)>p(j-1,1) && p(j,1)>p(j-1,1))))
								{
									intersection_count--;
								}
							}
						}
					}
				}
			}
		}
		if (intersection_count%2==1)
		{
			in(i,0) = 1;
		}
	}
	return in;
}
pcl::PointCloud<PointXYZRGB> removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    unordered_set<int> visited;
    vector<pcl::PointCloud<pcl::PointXYZRGB>> clusters;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> cloud_bw;
    pcl::copyPointCloud(cloud,cloud_bw);
    tree->setInputCloud (cloud_bw.makeShared());
    double spacing = 0.05;
    for(int i=0;i<cloud.points.size();i++)
    {
        pcl::PointXYZRGB pt = cloud.points[i];
        if(visited.find(i)!=visited.end())
            continue;
        pcl::PointXYZ pt_bw={pt.x,pt.y,pt.z};
        stack<int> next_point;
        next_point.push(i);
        vector<int> ids_cluster;
        ids_cluster.push_back(i);
        while(!next_point.empty())
        {
            auto id=next_point.top();next_point.pop();
            auto pt_local=cloud_bw.points[id];
            vector<int> indices;
            vector<float> dist;
            auto t=tree->radiusSearch(pt_local,spacing,indices,dist,0);
            for(auto ids:indices)
            {
                if(visited.find(ids)==visited.end())
                {
                    next_point.push(ids);
                    ids_cluster.push_back(ids);
                    visited.insert(ids);
                }
            }
        }
        pcl::PointCloud<pcl::PointXYZRGB> temp;
        for(auto id:ids_cluster)
            temp.points.push_back(cloud.points[id]);
        clusters.push_back(temp);
    }
    pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;
    for(auto cluster:clusters)
        if(cluster.points.size()>50)
            filtered_cloud+=cluster;
    return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB> removeBackground(pcl::PointCloud<pcl::PointXYZRGB>& cloud_bg, pcl::PointCloud<pcl::PointXYZRGB>& cloud_obj)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> cloud_bw;
    pcl::copyPointCloud(cloud_bg,cloud_bw);
    tree->setInputCloud (cloud_bw.makeShared());
    double spacing = 0.008;
    for(int i=0;i<cloud_obj.points.size();i++)
    {
        pcl::PointXYZRGB pt = cloud_obj.points[i];
        pcl::PointXYZ pt_b;
        pt_b.x=pt.x;
        pt_b.y=pt.y;
        pt_b.z=pt.z;
        vector<int> indices;
        vector<float> dist;
        auto t=tree->radiusSearch(pt_b,spacing,indices,dist,0);
        if(indices.size()==0)
            cloud_filtered.points.push_back(pt);
    }
    return cloud_filtered;
}

template <typename PointT> pcl::PointCloud<PointT> downsample(pcl::PointCloud<PointT> cloud, double leaf)
{
    pcl::VoxelGrid<PointT> sor;
    pcl::PointCloud<PointT> cloud_filtered;
    sor.setLeafSize (leaf, leaf, leaf);
    sor.setInputCloud (cloud.makeShared());
    sor.filter (cloud_filtered);
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB> reproject(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
    for(int i=0;i<cloud.points.size();i++)
    {
        pcl::PointXYZRGB pt = cloud.points[i];
        pt.z = 0;
        cloud_filtered.points.push_back(pt);
    }
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB> getConvexHull(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    pcl::ConvexHull<pcl::PointXYZ> cHull;
    pcl::PointCloud<pcl::PointXYZ> cHull_points;
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    for(int i=0;i<cloud.points.size();i++)
    {
        pcl::PointXYZ pt(cloud.points[i].x,cloud.points[i].y,0);
        cloud_xyz.points.push_back(pt);
    }
    cHull.setInputCloud(cloud_xyz.makeShared());
    cHull.reconstruct (cHull_points);
    pcl::PointCloud<pcl::PointXYZRGB> cloud_hull;
    for(int i=0;i<cHull_points.points.size();i++)
    {
        pcl::PointXYZRGB pt;
        pt.x = cHull_points.points[i].x;
        pt.y = cHull_points.points[i].y;
        pt.z = 0;
        pt.rgb = 0.8;
        cloud_hull.points.push_back(pt);
    }
    return cloud_hull;
}


template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB> filterPointcloudPolygon(pcl::PointCloud<PointT>& cloud_obj,pcl::PointCloud<PointT>& cloud_poly)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
	Eigen::MatrixXd boundary=MatrixXd::Zero(cloud_poly.points.size(),3);
    for(int i=0;i<cloud_poly.points.size();i++)
    {
        boundary(i,0) = cloud_poly.points[i].x;
        boundary(i,1) = cloud_poly.points[i].y;
    }
	for(int i=0;i<cloud_obj.points.size();i++)
	{
		Eigen::MatrixXd point(1,2);
		point<<cloud_obj.points[i].x,cloud_obj.points[i].y;
		Eigen::MatrixXd state_boundary=InPoly(point,boundary);
		if(state_boundary(0,0))
		{
			cloud_filtered.points.push_back(cloud_obj.points[i]);
		}
	}
    return cloud_filtered;
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bw_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_obj_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/rex/REX_WS/Catkin_WS/data/base.pcd", *cloud_bw_ptr) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return (-1);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/rex/REX_WS/Catkin_WS/data/lifted.pcd", *cloud_obj_ptr) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for lifted.  \n");
        return (-1);
    }
    pcl::PointCloud<pcl::PointXYZRGB> cloud_bw;
    for(auto pt:cloud_bw_ptr->points)
        cloud_bw.points.push_back(pt);
    pcl::PointCloud<pcl::PointXYZRGB> cloud_obj;
    for(auto pt:cloud_obj_ptr->points)
        cloud_obj.points.push_back(pt);
    visualizePointCloud<pcl::PointXYZRGB>(cloud_bw);
    visualizePointCloud<PointXYZRGB>(cloud_obj);
    tic();
    cloud_obj = downsample<pcl::PointXYZRGB>(cloud_obj,0.05);
    pcl::PointCloud<pcl::PointXYZRGB> cloud_of_obj = removeBackground(cloud_bw,cloud_obj);
    toc();
    visualizePointCloud<PointXYZRGB>(cloud_of_obj  );
    pcl::PointCloud<pcl::PointXYZRGB> cleaned_cloud = removeOutliers(cloud_of_obj);
    visualizePointCloud<PointXYZRGB>(cleaned_cloud );
    pcl::PointCloud<pcl::PointXYZRGB> reprojected_cloud = reproject(cleaned_cloud);
    reprojected_cloud.points.push_back(pcl::PointXYZRGB({-0.404,0.372,0,100,100,100}));
    reprojected_cloud.points.push_back(pcl::PointXYZRGB({0.466,0.352,0,100,100,100}));
    pcl::PointCloud<pcl::PointXYZRGB> cloud_poly = getConvexHull(reprojected_cloud);
    visualizePolygon<PointXYZRGB>(cloud_poly);
    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered_with_polygon = filterPointcloudPolygon(cloud_obj,cloud_poly);
    visualizePointCloud<PointXYZRGB>(cloud_filtered_with_polygon);
    return 0;
}
