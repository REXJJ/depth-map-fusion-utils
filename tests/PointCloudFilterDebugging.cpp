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
#include <mutex>          // std::mutex
#include <ncurses.h>

/*********************************************/
//PCL HEADERS
/**********************************************/
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/common/common.h>

/*********************************************/
//OTHER HEADERS
/**********************************************/
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>

#include <Volume.hpp>
#include <VisualizationUtilities.hpp>
#include <DebuggingUtilities.hpp>
#include <TransformationUtilities.hpp>
#include <Camera.hpp>
#include <RayTracingEngine.hpp>
#include <PointCloudProcessing.hpp>
#include <Algorithms.hpp>
#include <CommonUtilities.hpp>
#include <FileRoutines.hpp>

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace PointCloudProcessing;
using namespace TransformationUtilities;
using namespace Algorithms;

constexpr double ball_radius = 0.01;
constexpr double cylinder_radius = 0.002;
constexpr double downsample_radius = 0.005;

void visualize(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    VisualizationUtilities::PCLVisualizerWrapper viz;
    viz.addCoordinateSystem();
    viz.addPointCloud<pcl::PointXYZRGBNormal>(cloud);
    viz.spinViewer();
}

Eigen::Vector4f fitPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  Eigen::MatrixXd lhs (cloud->size(), 3);
  Eigen::VectorXd rhs (cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i)
  {
    const auto& pt = cloud->points[i];
    lhs(i, 0) = pt.x;
    lhs(i, 1) = pt.y;
    lhs(i, 2) = 1.0;

    rhs(i) = -1.0 * pt.z;
  }
  Eigen::Vector3d params = lhs.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(rhs);
  Eigen::Vector3d normal (params(0), params(1), 1.0);
  auto length = normal.norm();
  normal /= length;
  params(2) /= length;
  return {normal(0), normal(1), normal(2), params(2)};
}

double pointToPlaneDistance(Eigen::Vector4f plane, vector<double> pt)
{
    return fabs(plane(0)*pt[0]+plane(1)*pt[1]+plane(2)*pt[2]+plane(3))/(sqrt(pow(plane(0),2)+pow(plane(1),2)+pow(plane(2),2)));
}

double getMaxError(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    auto plane = fitPlane(cloud);
    std::cout<<"Plane Fitted.."<<std::endl;
    double avg_error = 0.0, max_error = -1000.0;
    for(int i=0;i<cloud->points.size();i++)
    {
        auto pt = cloud->points[i];
        auto err = fabs(pointToPlaneDistance(plane,{pt.x,pt.y,pt.z}));
        if(err>max_error)
            max_error = err;
        avg_error = avg_error + err;
    }
    avg_error = avg_error/cloud->points.size();
    std::cout<<"Average Error: "<<avg_error<<" Max Error: "<<max_error<<std::endl;
    std::cout<<"Plane Equation: "<<std::endl;
    return max_error;
}

Eigen::Vector3f getNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  Eigen::MatrixXd lhs (cloud->size(), 3);
  Eigen::VectorXd rhs (cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i)
  {
    const auto& pt = cloud->points[i];
    lhs(i, 0) = pt.x;
    lhs(i, 1) = pt.y;
    lhs(i, 2) = 1.0;

    rhs(i) = -1.0 * pt.z;
  }
  Eigen::Vector3d params = lhs.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(rhs);
  Eigen::Vector3d normal (params(0), params(1), 1.0);
  auto length = normal.norm();
  normal /= length;
  params(2) /= length;
  return {normal(0), normal(1), normal(2)};
}

Vector3f projectPointToVector(Vector3f pt, Vector3f norm_pt, Vector3f n)
{
    Vector3f d_xyz = n*ball_radius;
    Vector3f a = norm_pt - d_xyz;
    Vector3f b = norm_pt + d_xyz;
    Vector3f ap = (a-pt);
    Vector3f ab = (a-b);
    Vector3f p = a - (ap.dot(ab)/ab.dot(ab))*ab;
    return p;
}

void process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroids,pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bw (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud,*cloud_bw);
    tree->setInputCloud (cloud_bw);
    unsigned long long int good_centers = 0;
    for(int i=0;i<centroids->points.size();i++)
    {
        if(i%100==0)
        {
            std::cout<<i<<" out of "<<centroids->points.size()<<" done."<<std::endl;
        }
        auto pt = centroids->points[i];
        // pcl::PointXYZ ptxyz = pcl::PointXYZ({pt.x,pt.y,pt.z});
        pcl::PointXYZ ptxyz;
        ptxyz.x = pt.x;
        ptxyz.y = pt.y;
        ptxyz.z = pt.z;
        vector<int> indices;
        vector<float> dist;
        auto t=tree->radiusSearch(ptxyz,ball_radius,indices,dist,0);
        pcl::PointCloud<pcl::PointXYZ>::Ptr points( new pcl::PointCloud<pcl::PointXYZ> );
        for(auto ids:indices)
        {
            PointXYZ pt_temp;
            pt_temp.x = cloud->points[ids].x;
            pt_temp.y = cloud->points[ids].y;
            pt_temp.z = cloud->points[ids].z;
            points->points.push_back(pt_temp);
        }
        int good_points = 0;
#if 1
        if(indices.size()>10)
        {
            Vector3f normal = getNormal(points);
            Vector3f norm_pt;
            norm_pt<<pt.x,pt.y,pt.z;
            Vector3f pt_pro;
            pt_pro<<0,0,0;
            double weights = 0.0;
            for(auto ids:indices)
            {
                PointXYZ pt_temp;
                pt_temp.x = cloud->points[ids].x;
                pt_temp.y = cloud->points[ids].y;
                pt_temp.z = cloud->points[ids].z;
                Vector3f pt_loc;
                pt_loc<<pt_temp.x,pt_temp.y,pt_temp.z;
                Vector3f projected_points = projectPointToVector(pt_loc, norm_pt,normal);
                double distance_to_normal = (pt_loc - projected_points).norm();
                // std::cout<<"Distance To Normal: "<<distance_to_normal<<" Distance to normal Pt: "<<(pt_loc-norm_pt).norm()<<std::endl;
                if(distance_to_normal<cylinder_radius)
                {
                    pt_pro+= (1.0-distance_to_normal/cylinder_radius) * projected_points;
                    weights += (1.0-distance_to_normal/cylinder_radius);
                    // cout<<"Points: "<<pt_loc<<" Projected Points: "<<projected_points<<" Normal: "<<normal<<endl;
                    good_points++;

                }
            }
            std::cout<<"Good Points: "<<good_points<<std::endl;
            if(good_points==0)
            {
                cout<<"No Good Points.."<<endl;
                continue;
            }
            pt_pro/=weights;
            PointXYZRGB pt_processed;
            pt_processed.x = pt_pro(0);
            pt_processed.y = pt_pro(1);
            pt_processed.z = pt_pro(2);
            pt_processed.r = pt.r;
            pt_processed.g = pt.g;
            pt_processed.b = pt.b;
            good_centers++;
            processed->points.push_back(pt_processed); 
        }
#endif
    }
    processed->height = 1;
    processed->width = good_centers;
}

void normalsTest(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroids,pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr processed)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bw (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud,*cloud_bw);
    tree->setInputCloud (cloud_bw);
    unsigned long long int good_centers = 0;
    for(int i=0;i<centroids->points.size();i++)
    {
        auto pt = centroids->points[i];
        // pcl::PointXYZ ptxyz = pcl::PointXYZ({pt.x,pt.y,pt.z});
        pcl::PointXYZ ptxyz;
        ptxyz.x = pt.x;
        ptxyz.y = pt.y;
        ptxyz.z = pt.z;
        vector<int> indices;
        vector<float> dist;
        auto t=tree->radiusSearch(ptxyz,ball_radius,indices,dist,0);
        pcl::PointCloud<pcl::PointXYZ>::Ptr points( new pcl::PointCloud<pcl::PointXYZ> );
        for(auto ids:indices)
        {
            PointXYZ pt_temp;
            pt_temp.x = cloud->points[ids].x;
            pt_temp.y = cloud->points[ids].y;
            pt_temp.z = cloud->points[ids].z;
            points->points.push_back(pt_temp);
        }
        int good_points = 0;
        if(indices.size()>10)
        {
            Vector3f normal = getNormal(points);
            normal = normal.normalized();
            PointXYZRGBNormal pt_processed;
            pt_processed.x = pt.x;
            pt_processed.y = pt.y;
            pt_processed.z = pt.z;
            pt_processed.r = pt.r;
            pt_processed.g = pt.g;
            pt_processed.b = pt.b;
            pt_processed.normal[0] = normal(0);
            pt_processed.normal[1] = normal(1);
            pt_processed.normal[2] = normal(2);
            good_centers++;
            processed->points.push_back(pt_processed); 
        }
        processed->height = 1;
        processed->width = good_centers;
    }
}

void histogram(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_processed)
{
    auto plane_1 = fitPlane(cloud_downsampled);
    auto plane_2 = fitPlane(cloud_processed);
    unordered_map<int,int> a,b;
    for(int i=0;i<cloud_downsampled->points.size();i++)
    {
        auto pt1 = cloud_downsampled->points[i];
        auto pt2 = cloud_processed->points[i];
        double err1 = fabs(pointToPlaneDistance(plane_1,{pt1.x,pt1.y,pt1.z}));
        double err2 = fabs(pointToPlaneDistance(plane_2,{pt2.x,pt2.y,pt2.z}));
        a[int(err1*1000.0)]++;
        b[int(err2*1000.0)]++;
    }
    std::cout<<"Cloud Downsampled: "<<std::endl;
    for(int i=0;i<10;i++)
        std::cout<<i<<" : "<<a[i]<<endl;
    
    std::cout<<"Cloud Filtered: "<<std::endl;
    for(int i=0;i<10;i++)
        std::cout<<i<<" : "<<b[i]<<std::endl;
}

struct sphere
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    Vector3f normal;
    Vector3f centroid;
    int state;
};

class VizD : public VizThread
{
    public:
        bool cloud_added;
        int cloud_no;
        int state;
        vector<sphere> clouds;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_processed;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_processed_viz;
        Vector4f plane;
        VizD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw,pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr processed,Vector4f p)
        {
            cloud = raw;
            cloud_processed = processed;
            plane = p;
            cloud_added = false;
            cloud_processed_viz.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
            cloud_no = 0;
            state = 0;
            for(int i=0;i<processed->points.size();i++)
            {
                pcl::PointXYZRGB pt;
                pt.x = processed->points[i].x;
                pt.y = processed->points[i].y;
                pt.z = processed->points[i].z;
                pt.r = processed->points[i].r;
                pt.g = processed->points[i].g;
                pt.b = processed->points[i].b;
                cloud_processed_viz->points.push_back(pt);
            }
        }
        void process(VisualizationUtilities::PCLVisualizerWrapper &viz);
        void input();
};

void VizD::process(VisualizationUtilities::PCLVisualizerWrapper &viz)
{
    if(clouds.size()==0)
    {
        return;
    }

    viz.viewer_->removeAllShapes();
#if 0
    viz.viewer_->removeAllCoordinateSystems();
#else
    viz.viewer_->removeCoordinateSystem();
#endif
    viz.addCoordinateSystem();
    auto cloud = clouds[cloud_no].cloud;
    std::cout<<"State: "<<state<<std::endl;
    std::cout<<"Cloud no: "<<cloud_no<<std::endl;
    PointXYZ centroid;
    centroid.x = clouds[cloud_no].centroid(0);
    centroid.y = clouds[cloud_no].centroid(1);
    centroid.z = clouds[cloud_no].centroid(2);

    if(state==4)
    {
        viz.updatePointCloud<PointXYZRGB>(cloud_processed_viz,"cloud");
        viz.addSphere(centroid,"sphere");
    }
    else     
    {
        pcl::ModelCoefficients::Ptr plane_1 (new pcl::ModelCoefficients);
        plane_1->values.resize (4);
        plane_1->values[0] = plane(0);
        plane_1->values[1] = plane(1);
        plane_1->values[2] = plane(2);
        plane_1->values[3] = plane(3);
        viz.viewer_->addPlane (*plane_1, "plane", 0);
        pcl::PointXYZRGB pt_centroid;
        pt_centroid.x = centroid.x;
        pt_centroid.y = centroid.y;
        pt_centroid.z = centroid.z;
        pt_centroid.r = 255;
        pt_centroid.g = 0;
        pt_centroid.b = 0;

        Vector3f a = {centroid.x,centroid.y,centroid.z};
        Vector3f n = clouds[cloud_no].normal;
        Vector3f b = a+n*0.01;
        vector<double> start = {a(0),a(1),a(2)};
        vector<double> end = {b(0),b(1),b(2)};
        viz.addLine(start,end,"line",{255,0,0});

        if(state==3)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
            for(int i=0;i<cloud.points.size();i++)
            {
                auto pt = cloud.points[i];
                pcl::PointXYZRGB ptrgb;
                ptrgb.x = pt.x;
                ptrgb.y = pt.y;
                ptrgb.z = pt.z;
                ptrgb.r = 0;
                ptrgb.g = 0;
                ptrgb.b = 0;
                temp->points.push_back(ptrgb);
            }
            temp->points.push_back(pt_centroid);
            viz.updatePointCloud<pcl::PointXYZRGB>(temp,"cloud");
        }
        else if(state==2)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
            for(int i=0;i<cloud.points.size();i++)
            {
                auto pt_temp = cloud.points[i];
                Vector3f pt_loc;
                pt_loc<<pt_temp.x,pt_temp.y,pt_temp.z;
                Vector3f projected_points = projectPointToVector(pt_loc, a,n);
                double distance_to_normal = (pt_loc - projected_points).norm();
                if(distance_to_normal<cylinder_radius)
                    temp->points.push_back(pt_temp);
            }
            temp->points.push_back(pt_centroid);
            viz.updatePointCloud<pcl::PointXYZRGB>(temp,"cloud");
        }
        else if(state==1)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
            int good_points = 0;
            for(int i=0;i<cloud.points.size();i++)
            {
                auto pt_temp = cloud.points[i];
                Vector3f pt_loc;
                pt_loc<<pt_temp.x,pt_temp.y,pt_temp.z;
                Vector3f projected_points = projectPointToVector(pt_loc, a,n);
                double distance_to_normal = (pt_loc - projected_points).norm();
                if(distance_to_normal<cylinder_radius)
                {
                    pcl::PointXYZRGB pt_rgb;
                    pt_rgb.x = projected_points(0);
                    pt_rgb.y = projected_points(1);
                    pt_rgb.z = projected_points(2);
                    pt_rgb.r = 0;
                    pt_rgb.g = 0;
                    pt_rgb.b = 255;
                    temp->points.push_back(pt_rgb);
                    good_points++;
                }
            }
            std::cout<<"Good Points: "<<good_points<<std::endl;
            temp->points.push_back(pt_centroid);
            viz.updatePointCloud<pcl::PointXYZRGB>(temp,"cloud");
        }
        else if(state==0)
        {
            // viz.viewer_->removeAllShapes();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
            Vector3f pt_pro;
            pt_pro<<0,0,0;
            double weights = 0.0;
            int good_points = 0;
            PointXYZRGB pt_processed;
            for(int i=0;i<cloud.points.size();i++)
            {
                auto pt_temp = cloud.points[i];
                Vector3f pt_loc;
                pt_loc<<pt_temp.x,pt_temp.y,pt_temp.z;
                Vector3f projected_points = projectPointToVector(pt_loc, a,n);
                double distance_to_normal = (pt_loc - projected_points).norm();
                if(distance_to_normal<cylinder_radius)
                {
                    pt_pro+= projected_points;
                    weights += (1.0);
                    // cout<<"Points: "<<pt_loc<<" Projected Points: "<<projected_points<<" Normal: "<<normal<<endl;
                    good_points++;

                }
            }
            if(good_points)
            {
                pt_pro/=weights;
                pt_processed.x = pt_pro(0);
                pt_processed.y = pt_pro(1);
                pt_processed.z = pt_pro(2);
                pt_processed.r = 0;
                pt_processed.g = 255;
                pt_processed.b = 0;
                // temp->points.push_back(pt_processed);
            }
            // temp->points.push_back(pt_centroid);
            std::cout<<temp->points.size()<<std::endl;
            viz.updatePointCloud<pcl::PointXYZRGB>(temp,"cloud");

            vector<double> start = {pt_centroid.x,pt_centroid.y,pt_centroid.z};
            vector<double> end = {0,0,0};
            viz.addLine(start,end,"line_centroid",{255,0,0});

            vector<double> start_2 = {pt_processed.x,pt_processed.y,pt_processed.z};
            vector<double> end_2 = {0,0,0};
            viz.addLine(start_2,end_2,"line_new",{0,255,0});
        }
    }
}

void VizD::input()
{   
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bw (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud,*cloud_bw);
    tree->setInputCloud (cloud_bw);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    downsample<pcl::PointXYZRGB>(cloud,cloud_filtered,downsample_radius);

    vector<sphere> spheres;
    for(int i=0;i<cloud_processed->points.size();i++)
    {
        auto pt = cloud_processed->points[i];
        Vector3f a = {pt.x,pt.y,pt.z};
        auto err = fabs(pointToPlaneDistance(plane,{a(0),a(1),a(2)}));
        if(err<0.0035)
            continue;
        Vector3f n = {pt.normal[0],pt.normal[1],pt.normal[2]};
        sphere s;
        auto pt_d = cloud_filtered->points[i];
        Vector3f centroid = {pt_d.x,pt_d.y,pt_d.z};
        s.centroid = centroid;
        s.normal = n;

        pcl::PointXYZ ptxyz;
        ptxyz.x = pt_d.x;
        ptxyz.y = pt_d.y;
        ptxyz.z = pt_d.z;
        vector<int> indices;
        vector<float> dist;
        auto t=tree->radiusSearch(ptxyz,ball_radius,indices,dist,0);
        pcl::PointCloud<pcl::PointXYZRGB> points;
        for(auto ids:indices)
        {
            PointXYZRGB pt_temp;
            pt_temp.x = cloud->points[ids].x;
            pt_temp.y = cloud->points[ids].y;
            pt_temp.z = cloud->points[ids].z;
            pt_temp.r = 0;
            pt_temp.g = 0;
            pt_temp.b = 0;
            points.points.push_back(pt_temp);
        }
        s.cloud = points;
        s.state = 3;
        spheres.push_back(s);
    }
    for(auto s:spheres)
    {
        clouds.push_back(s);
    }
    for(int i=0;i<spheres.size();i++)
    {
        int n;
        cloud_no = i;
        state = 4;
        updateViewer();
        // sleep(2);
        state = 3;
        updateViewer();
        // sleep(2);
        state = 2;
        updateViewer();
        // sleep(2);
        state = 1;
        updateViewer();
        // sleep(2);
        // state = 0;
        // updateViewer();
        std::cout<<"Done"<<std::endl;
    }
    std::cout<<"All Done"<<std::endl;
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
#if 1
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return -1;
    }
#else
    pcl::PLYReader Reader;
    Reader.read(string(argv[1]), *cloud);
#endif
    // visualize(cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    downsample<pcl::PointXYZRGB>(cloud,cloud_filtered,downsample_radius);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_processed(new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloud<PointXYZRGBNormal>::Ptr normal(new PointCloud<PointXYZRGBNormal>);
    process(cloud,cloud_filtered,cloud_processed);
    std::cout<<"Histograms: "<<std::endl;
    histogram(cloud_filtered,cloud_processed);
    cloud_filtered = cloud_processed;
    //TODO: Change all cloud_filtered to cloud_processed.
    auto plane = fitPlane(cloud_filtered);
    normalsTest(cloud,cloud_filtered,normal);
    VisualizationUtilities::PCLVisualizerWrapper viz;
    auto max_err = getMaxError(cloud_filtered);
    for(int i=0;i<normal->points.size();i++)
    {
        auto pt = normal->points[i];
        Vector3f a = {pt.x,pt.y,pt.z};
        auto err = fabs(pointToPlaneDistance(plane,{a(0),a(1),a(2)}));
        if(err<0.0035)
            continue;
        cloud_filtered->points[i].r = 255;
        cloud_filtered->points[i].g = 0;
        cloud_filtered->points[i].b = 0;
        Vector3f n = {pt.normal[0],pt.normal[1],pt.normal[2]};
        Vector3f b = a+n*0.01;
        vector<double> start = {a(0),a(1),a(2)};
        vector<double> end = {b(0),b(1),b(2)};
        viz.addLine(start,end,"line"+to_string(i),{255,0,0});
    }
    viz.addPointCloud<pcl::PointXYZRGB>(cloud_filtered);
    viz.spinViewer();
    std::cout<<"Analysing Individual Spheres..."<<std::endl;
    VizD vd(cloud,normal,plane);
    vd.makeThreads();
    return 0;
}
