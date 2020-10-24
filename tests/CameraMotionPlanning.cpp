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
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>

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

#include<Volume.hpp>
#include<VisualizationUtilities.hpp>
#include<DebuggingUtilities.hpp>
#include<TransformationUtilities.hpp>
#include<Camera.hpp>
#include<RayTracingEngine.hpp>

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace cv;

namespace PointCloudProcessing
{
    void makePointCloudNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals,vector<double> viewpoint={0.0,0.0,0.0})
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bw (new PointCloud<pcl::PointXYZ>);
        for(int i=0;i<cloud->points.size();i++)
        {
            PointXYZ ptxyz;
            PointXYZRGB ptxyzrgb = cloud->points[i];
            ptxyz.x=ptxyzrgb.x;
            ptxyz.y=ptxyzrgb.y;
            ptxyz.z=ptxyzrgb.z;
            cloud_bw->points.push_back(ptxyz);
        }
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (cloud_bw);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (0.005);
        ne.setViewPoint(viewpoint[0],viewpoint[1],viewpoint[2]);
        ne.compute (*normals); 
    }
}

Eigen::Affine3f getCameraLocation(vector<double> location)
{
    using namespace TransformationUtilities;
    Eigen::MatrixXd two_T_one = vectorToTransformationMatrix(location);
    Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            transformation(i,j)=two_T_one(i,j);
    return transformation;
}

namespace Algorithms
{
    vector<unsigned long long int> greedySetCover(vector<vector<unsigned long long int>> &candidate_sets,double resolution = 0.000008)
    {
        vector<unsigned long long int> covered;
        vector<unsigned long long int> selected_sets;
        vector<unsigned long long int> set_ids(candidate_sets.size());
        iota(set_ids.begin(),set_ids.end(),0);
        double volume_threshold = 0.000008/(resolution*10);
        cout<<volume_threshold<<endl;
        while(true)
        {
            float minimum=1.0;
            unsigned long long int selected=-1;
            for(auto x:set_ids)
            {
                vector<unsigned long long int> difference;
                std::set_difference(candidate_sets[x].begin(),candidate_sets[x].end(),covered.begin(),covered.end(),std::inserter(difference,difference.begin()));
                float cost = 1/float(difference.size());
                if(difference.size()==0)
                    cost=1.0;
                if(cost<minimum)
                {
                    minimum=cost;
                    selected=x;
                }
            }
            if(minimum>volume_threshold)
                break;
            if(selected==-1)
                break;
            vector<unsigned long long int> difference;
            std::set_difference(candidate_sets[selected].begin(),candidate_sets[selected].end(),covered.begin(),covered.end(),std::inserter(difference,difference.begin()));
            for(auto x:difference)
                covered.push_back(x);
            sort(covered.begin(),covered.end());
            selected_sets.push_back(selected);
            vector<unsigned long long int> temp;
            set_ids.erase(std::remove(set_ids.begin(), set_ids.end(),selected), set_ids.end());
        }
        return selected_sets;
    }

    void generateSphere(double radius,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere,Eigen::Affine3f transformation=Eigen::Affine3f::Identity())
    {
        for(double r=radius;r>=0;r-=0.05)
        {
            for(double x=-r;x<=r;x+=0.05)
            {
                double y=sqrt(r*r-(x*x));
                PointXYZRGB pt;
                pt.x=x;
                pt.y=y;
                pt.z=sqrt(radius*radius-(x*x)-(y*y));
                pt.r=0;
                pt.g=0;
                pt.b=255;
                sphere->points.push_back(pt);
                pt.y=-y;
                sphere->points.push_back(pt);
                pt.y=y;
                sphere->points.push_back(pt);
            }
        }
        for(int i=0;i<sphere->points.size();i++)
        {
            pcl::PointXYZRGB pt = sphere->points[i];
            Vector3f point(3);
            point<<pt.x,pt.y,pt.z;
            Vector3f transformed = transformation*point;
            sphere->points[i].x = transformed(0);
            sphere->points[i].y = transformed(1);
            sphere->points[i].z = transformed(2);
        }
    }
};

#if 0
int main()
{
    vector<int>  S1 = {1, 2};
  vector<int>  S2 = {2, 3, 4, 5};
  vector<int>  S3 = {6, 7, 8, 9, 10, 11, 12, 13};
  vector<int>  S4 = {1, 3, 5, 7, 9, 11, 13};
  vector<int>  S5 = {2, 4, 6, 8, 10, 12, 13};
  vector<int>  U = {1,2,3,4,5,6,7,8,9,10,11,12,13};
  vector<vector<int>> S = {S1,S2,S3,S4,S5};
  vector<int> c(S.size());
  iota(c.begin(),c.end(),0);
  Algorithms::greedySetCoverDummy(U,S,c);
  return 0;
}
#else
int main(int argc, char** argv)
{
    using namespace TransformationUtilities;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
#if 0
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> ("/home/rex/REX_WS/Catkin_WS/data/base.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return (-1);
    }
#else
    pcl::PLYReader Reader;
    Reader.read("/home/rex/REX_WS/Test_WS/POINT_CLOUD_STITCHING/data/empty_box_inside/"+string(argv[1])+".ply", *cloud);
#endif
    for(auto &pt:cloud->points)
        pt.z*=-1;

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    PointCloudProcessing::makePointCloudNormal(cloud,normals);

    vector<float> K = {602.39306640625, 0.0, 314.6370849609375, 0.0, 602.39306640625, 245.04962158203125, 0.0, 0.0, 1.0};
    constexpr double PI = 3.141592;
    VoxelVolume volume;
    volume.setDimensions(-0.5,0.5,-0.5,0.5,0,1);
    volume.setVolumeSize(50,50,50);
    volume.constructVolume();
    volume.integratePointCloud(cloud,normals);
   
    vector<Affine3f> camera_locations;
    camera_locations.push_back(getCameraLocation({0.8,-0.4,0.5,0,-PI/2,-PI/6}));
    camera_locations.push_back(getCameraLocation({0.8,-0.2,0.5,0,-PI/2,0}));
    camera_locations.push_back(getCameraLocation({0.8,-0.6,0.5,0,-PI/2,-PI/6}));
    camera_locations.push_back(getCameraLocation({0,-0.6,+0.4,0,0,-PI/3}));
    Camera cam(K);
    RayTracingEngine engine(cam);

    vector<vector<unsigned long long int>> regions_covered;
    for(int i=0;i<camera_locations.size();i++)
    {
        auto[found,good_points] = engine.rayTraceAndGetGoodPoints(volume,camera_locations[i]);
        sort(good_points.begin(),good_points.end());
        regions_covered.push_back(good_points);
    }

    double resolution = volume.voxel_size_;
    cout<<resolution<<" Resolution"<<endl;

    auto cameras_selected = Algorithms::greedySetCover(regions_covered,resolution);

    for(auto x:cameras_selected)
        cout<<x<<" ";
    cout<<endl;

    // engine.rayTraceAndClassify(volume,camera_locations[1]);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere (new pcl::PointCloud<pcl::PointXYZRGB>);

    Affine3f transformHemiSphere = getCameraLocation({volume.xcenter_,volume.ycenter_,volume.zcenter_,0,0,-PI/2});

    Algorithms::generateSphere(0.3,sphere,transformHemiSphere);

    VisualizationUtilities::PCLVisualizerWrapper viz;
    viz.addCoordinateSystem();
    viz.addVolumeWithVoxelsClassified(volume);
    viz.addPointCloud<pcl::PointXYZRGB>(sphere);
    // for(auto x:cameras_selected)
    //     viz.addCamera(cam,camera_locations[x],"camera"+to_string(x));
    viz.spinViewer();
    return 0;
}
#endif

    // cout<<found<<" "<<good_points.size()<<endl;
    // for(auto id:good_points)
    // {
    //     unsigned long long int mask = (1<<20)-1;
    //     unsigned long long int xid = id>>40;
    //     unsigned long long int yid = id>>20&mask;
    //     unsigned long long int zid = id&mask;
    //     cout<<id<<" "<<xid<<" "<<yid<<" "<<zid<<endl;
    //     Voxel* voxel = volume.voxels_[xid][yid][zid];
    //     voxel->view=1;
    //     voxel->good=true;
    // }
    //    void greedySetCoverDummy(vector<int> U,vector<vector<int>> &u,vector<int> s)
    // {
    //     vector<int> I;
    //     vector<int> st;
    //     vector<int> ans;
    //     for(int i=0;i<u.size();i++)
    //         sort(u[i].begin(),u[i].end());
    //     sort(U.begin(),U.end());
    //     while(true)
    //     {
    //         float mn=10000.0;
    //         int sel=-1;
    //         for(auto x:s)
    //         {
    //             vector<int> temp;
    //             std::set_difference(u[x].begin(),u[x].end(),I.begin(),I.end(),std::inserter(temp,temp.begin()));
    //             float cost = 1/float(temp.size());
    //             if(temp.size()==0)
    //                 cost=10000.0;
    //             if(cost<mn)
    //             {
    //                 mn=cost;
    //                 sel=x;
    //             }
    //         }
    //         if(sel==-1)
    //             break;
    //         vector<int> i_temp;
    //         std::set_difference(u[sel].begin(),u[sel].end(),I.begin(),I.end(),std::inserter(i_temp,i_temp.begin()));
    //         for(auto x:i_temp)
    //             I.push_back(x);
    //         sort(I.begin(),I.end());
    //         ans.push_back(sel);
    //         vector<int> not_found;
    //         std::set_difference(U.begin(),U.end(),I.begin(),I.end(),std::inserter(not_found,not_found.begin()));
    //         if(not_found.size()==0)
    //             break;
    //         vector<int> temp;
    //         for(auto x:s)
    //             if(x!=sel)
    //                 temp.push_back(x);
    //         s=temp;
    //     }
    //     for(auto x:ans)
    //         cout<<x<<" ";
    //     cout<<endl;
    // }
    //
