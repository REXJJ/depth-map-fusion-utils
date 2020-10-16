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
#include <pcl/common/common.h>

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
        // double volume_threshold = 0.000008/(resolution*10);
        // double volume_threshold = 1e-6; 
        double volume_threshold = 27e-6; 
        // double volume_threshold = 1.25e-4; 
        cout<<"Volume Threshold: "<<volume_threshold<<endl;
        cout<<volume_threshold<<endl;
        while(true)
        {
            float minimum=1.0;
            unsigned long long int selected=-1;
            double total_volume_increase = 0;
            for(auto x:set_ids)
            {
                vector<unsigned long long int> difference;
                std::set_difference(candidate_sets[x].begin(),candidate_sets[x].end(),covered.begin(),covered.end(),std::inserter(difference,difference.begin()));
                float cost = 1/float(difference.size());
                if(difference.size()==0)
                    cost=1.0;
                if(difference.size()*resolution>total_volume_increase)
                    total_volume_increase = difference.size()*resolution;
                if(cost<minimum)
                {
                    minimum=cost;
                    selected=x;
                }
                cout<<difference.size()<<" <----------------------"<<endl;
            }
            cout<<"Minimum: "<<minimum<<endl;
            // if(total_volume_increase<volume_threshold)
            // {
            //     cout<<"Total Volume Increase: "<<total_volume_increase<<endl;
            //     break;
            // }
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

#if 0
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
                pt.g=255;
                pt.b=0;
                sphere->points.push_back(pt);
                pt.y=-y;
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
#else
    void generateSphere(double radius,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere,Eigen::Affine3f transformation=Eigen::Affine3f::Identity())
    {
        const double PI = 3.141592653589793238462643383279502884197;

        // Iterate through phi, theta then convert r,theta,phi to  XYZ
        const double factor = 10.0;
        for (double phi = 0.; phi <= PI; phi += PI/factor) // Azimuth [0,PI]
        {
            for (double theta = 0.; theta <= PI; theta += PI/factor) // Elevation [0, PI]
            {
                pcl::PointXYZRGB point;
                point.x = radius * cos(phi) * sin(theta);
                point.y = radius * sin(phi) * sin(theta);
                point.z = radius            * cos(theta);
                point.g = 255;
                sphere->points.push_back(point);        
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
        cout<<"Generated "<<sphere->points.size()<<" points.."<<endl;
    }
#endif
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

void usage(string program_name)
{
    std::cout<<program_name<<" <Pointcloud filename>"<<std::endl;
    exit(-1);
}

#if 0
vector<double> getCameraPosition(double x,double y,double z,double r,Eigen::Affine3f transformation=Eigen::Affine3f::Identity())
{
    Vector3f point(3);
    point<<x,y,z;
    auto transformation_inverse = transformation.inverse();
    Vector3f transformed = transformation_inverse*point;
    double theta = acos(transformed(2)/r);
    double phi = asin(transformed(1)/(r*sin(theta)));
    return {x,y,z,0,0,0};
}
#else
Eigen::Affine3f getCameraPosition(vector<double> center,vector<double> pi,bool print=false)
{
    Vector3f t1(3);
    t1<<center[0],center[1],center[2];
    Vector3f t2(3);
    t2<<pi[0],pi[1],pi[2];
    Vector3f k(3);
    k<<0,0,1;
    k = k.normalized();
    auto n = (t1-t2).normalized();
    double theta = acos(k.dot(n));
    auto b = k.cross(n);
    Eigen::Affine3f Q = Eigen::Affine3f::Identity();
    if(fabs(theta-3.14159)<0.00001)
    {
        cout<<"Camera Directly Above."<<endl;
        Q = getCameraLocation({pi[0],pi[1],pi[2],0,0,-theta});
        return Q;
    }
    if(print)
    {
        cout<<"Printing Data: "<<endl;
        cout<<"Theta: "<<theta<<endl;
        cout<<"t1: "<<t1<<endl;
        cout<<"t2: "<<t2<<endl;
        cout<<"N: "<<n<<endl;
        cout<<"K: "<<k<<endl;
        cout<<"b: "<<b<<endl;
    }
    double q0 = cos(theta/2);
    double q1 = sin(theta/2)*b(0);
    double q2 = sin(theta/2)*b(1);
    double q3 = sin(theta/2)*b(2);
    Q(0,0) = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    Q(0,1) = 2*(q1*q2-q0*q3);
    Q(0,2) = 2*(q1*q3+q0*q2);
    Q(1,0) = 2*(q2*q1+q0*q3);
    Q(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
    Q(1,2) = 2*(q2*q3-q0*q1);
    Q(2,0) = 2*(q3*q1-q0*q2);
    Q(2,1) = 2*(q2*q3+q0*q1);
    Q(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;
    Q(0,3) = pi[0];
    Q(1,3) = pi[1];
    Q(2,3) = pi[2];
    return Q;
}
#endif

vector<double> moveCameraAway(vector<double> center,vector<double> pi,double distance)
{
    Vector3f t1(3);
    Vector3f t2(3);
    t1<<center[0],center[1],center[2];
    t2<<pi[0],pi[1],pi[2];
    auto n = (t2-t1).normalized()*distance;
#if 0
    vector<double> test = {n(0)+center[0],n(1)+center[1],n(2)+center[2]};
    Vector3f tt(3);
    tt<<test[0],test[1],test[2];
    std::cout<<(tt-t1).normalized()<<endl;
    std::cout<<"----------------------------"<<endl;
    std::cout<<n.normalized()<<endl;
#endif
    return {n(0)+center[0],n(1)+center[1],n(2)+center[2]};
}

void repositionCameras(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere,VoxelVolume& volume)
{
    vector<bool> visited(sphere->points.size(),false);
    for(int i=1000;i>=0;i--)
    {
        for(int j=0;j<sphere->points.size();j++)
        {
            if(visited[j])
                continue;
            double distance = double(i)/1000.0;
            auto new_points = moveCameraAway({volume.xcenter_,volume.ycenter_,volume.zcenter_},{sphere->points[j].x,sphere->points[j].y,sphere->points[j].z},distance);
            double x = new_points[0];
            double y = new_points[1];
            double z = new_points[2];
            if(volume.validPoints(x,y,z)==false)
                continue;
            auto coords = volume.getVoxel(x,y,z);
            int xid = get<0>(coords);
            int yid = get<1>(coords);
            int zid = get<2>(coords);
            Voxel *voxel = volume.voxels_[xid][yid][zid];
            if(voxel!=nullptr)
            {
                visited[j]=true;
                new_points = moveCameraAway({volume.xcenter_,volume.ycenter_,volume.zcenter_},new_points,distance+0.3);
                sphere->points[j].x = new_points[0];
                sphere->points[j].y = new_points[1];
                sphere->points[j].z = new_points[2];
            }
        }
    }
}

int main(int argc, char** argv)
{
    using namespace TransformationUtilities;
    if(argc<2)
        usage(string(argv[0]));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
#if 1
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (argv[1], *cloud_temp) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return (-1);
    }
#else
    pcl::PLYReader Reader;
    Reader.read(string(argv[1]), *cloud);
#endif
    // for(auto &pt:cloud->points)
    //     pt.z*=-1;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    for(int i=0;i<cloud_temp->points.size();i++)
    {
        PointXYZRGBNormal pt = cloud_temp->points[i];
        PointXYZRGB pt_rgb;
        Normal pt_n;
        pt_rgb.x = pt.x;
        pt_rgb.y = pt.y;
        pt_rgb.z = pt.z;
        pt_rgb.r = 0;
        pt_rgb.g = 0;
        pt_rgb.b = 0;
        // pt_n.normal = pt.normal;
        memcpy(pt_n.normal,pt.normal,3*sizeof(float));
        cloud->points.push_back(pt_rgb);
        normals->points.push_back(pt_n);
    }
    // PointCloudProcessing::makePointCloudNormal(cloud,normals);
    cout<<"Here:----------------------------> "<<normals->points.size()<<endl;
    vector<float> K = {602.39306640625, 0.0, 314.6370849609375, 0.0, 602.39306640625, 245.04962158203125, 0.0, 0.0, 1.0};
    constexpr double PI = 3.141592653589793238462643383279502884197;
    VoxelVolume volume;
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*cloud, min_pt, max_pt);
    volume.setDimensions(min_pt.x,max_pt.x,min_pt.y,max_pt.y,min_pt.z,max_pt.z);
    volume.setVolumeSize(50,50,50);
    volume.constructVolume();
    volume.integratePointCloud(cloud,normals);
    Camera cam(K);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere (new pcl::PointCloud<pcl::PointXYZRGB>);
    Affine3f transformHemiSphere = getCameraLocation({volume.xcenter_,volume.ycenter_,0,0,0,PI/2.0});
    Algorithms::generateSphere(0.3,sphere,transformHemiSphere);
    repositionCameras(sphere,volume);

    vector<Affine3f> camera_locations;
    for(int i=0;i<sphere->points.size();i++)
    {
        bool print = false;
        if(i==59||i==60)
            print=true;
        else 
            print=false;
        auto camera_location = getCameraPosition({volume.xcenter_,volume.ycenter_,volume.zcenter_},{sphere->points[i].x,sphere->points[i].y,sphere->points[i].z},print);
        camera_locations.push_back(camera_location);
    }
    RayTracingEngine engine(cam);
    vector<vector<unsigned long long int>> regions_covered;
    // cout<<"Printing Good Points"<<endl;
    for(int i=0;i<camera_locations.size();i++)
    {
        auto[found,good_points] = engine.rayTraceAndGetGoodPoints(volume,camera_locations[i]);
        sort(good_points.begin(),good_points.end());
        regions_covered.push_back(good_points);
        // cout<<good_points.size()<<endl;
    }
    double resolution = volume.voxel_size_;
    cout<<resolution<<" Resolution"<<endl;
    auto cameras_selected = Algorithms::greedySetCover(regions_covered,resolution);
    
    for(auto x:cameras_selected)
        cout<<x<<" ";
    cout<<endl;

    VisualizationUtilities::PCLVisualizerWrapper viz;
    // for(auto x:{0,1,2,3,4,5,6,7,8,9,10})
    // for(auto x:{59,60})
    // for(auto x:{stoi(argv[2])})
    for(auto x:cameras_selected)
    // for(int x=0;x<camera_locations.size();x++)
    {
        // if(camera_locations[x](2,3)<0.6)
        //     continue;
        // cout<<x<<" <----- Camera"<<endl;
        engine.rayTraceAndClassify(volume,camera_locations[x]);
        viz.addCamera(cam,camera_locations[x],"camera"+to_string(x));
    }
    //Problem with Camera 60. TODO: Fix it.
    // std::cout<<"Camera 60"<<endl;
    // for(int i=0;i<3;i++)
    // {
    //     for(int j=0;j<4;j++)
    //         cout<<camera_locations[60](i,j)<<" ";
    //     cout<<endl;
    // }
    // std::cout<<"Camera 59"<<endl;
    // for(int i=0;i<3;i++)
    // {
    //     for(int j=0;j<4;j++)
    //         cout<<camera_locations[59](i,j)<<" ";
    //     cout<<endl;
    // }

    viz.addCoordinateSystem();
    viz.addVolumeWithVoxelsClassified(volume);
    // viz.addPointCloudInVolumeRayTraced(volume);
    // viz.addPointCloud<pcl::PointXYZRGB>(sphere);
    viz.addSphere({volume.xcenter_,volume.ycenter_,volume.zcenter_},"origin");
    // viz.addPointCloud<pcl::PointXYZRGB>(cloud);
    // viz.addPointCloud<pcl::PointXYZRGB>(cloud);
    // for(auto x:cameras_selected)
    //     viz.addCamera(cam,camera_locations[x],"camera"+to_string(x));
#if 0
    for(int i=0;i<sphere->points.size();i++)
    {
        auto test_camera_location = getCameraPosition({volume.xcenter_,volume.ycenter_,volume.zcenter_},{sphere->points[i].x,sphere->points[i].y,sphere->points[i].z});
        viz.addCamera(cam,test_camera_location,"camera"+to_string(i));
    }
#endif
    // viz.addPointCloud<pcl::PointXYZRGB>(cloud);
    // for(auto x:cameras_selected)
    //     viz.addCamera(cam,camera_locations[x],"camera"+to_string(x));
    viz.spinViewer();
    return 0;
}
#endif
