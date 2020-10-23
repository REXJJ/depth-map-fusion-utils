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

#include<Volume.hpp>
#include<VisualizationUtilities.hpp>
#include<DebuggingUtilities.hpp>
#include<TransformationUtilities.hpp>
#include<Camera.hpp>
#include<RayTracingEngine.hpp>
#include<PointCloudProcessing.hpp>
#include<Algorithms.hpp>
#include<CommonUtilities.hpp>

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace PointCloudProcessing;
using namespace TransformationUtilities;
using namespace Algorithms;

void usage(string program_name)
{
    std::cout<<program_name<<" <Pointcloud filename>"<<std::endl;
    exit(-1);
}

vector<double> movePointAway(vector<double> pi,vector<double> nor,double distance)
{
    Vector3f p1(3);
    Vector3f n(3);
    p1<<pi[0],pi[1],pi[2];
    n<<nor[0],nor[1],nor[2];
    n = n*distance;
    return {n(0)+pi[0],n(1)+pi[1],n(2)+pi[2]};
}

vector<vector<double>> lines;

Affine3f positionCamera(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr locations,int id,unsigned int distance=300)
{
    pcl::PointXYZRGBNormal pt = locations->points[id];
    Vector3f nor(3);
    nor<<pt.normal[0],pt.normal[1],pt.normal[2];
    vector<double> normals = {nor(0),nor(1),nor(2)};
    nor=nor*-1;
    Vector3f x(3);
    x<<1,1,0;
    if(nor(2)!=0.0)
    {
        x(2) = -(nor(0)+nor(1))/nor(2);
    }
    else
    {
        cout<<"Bad Normal"<<endl;
        int id = -1;
        for(int i=0;i<3;i++)
        {
            if(nor(i)!=0)
            {
                id = i;
                break;
            }
        }
        if(id==-1)
        {
            cout<<"Serious bug. Ignore this transformation."<<endl;//TODO
        }
        cout<<"Id: "<<id<<endl;
        vector<int> d;
        for(int j = 0;j<3;j++)
            if(id!=j)
            {
                d.push_back(j);
                x(j) = 1;
            }
        for(int j=0;j<3;j++)
            if(id!=j)
                x(id)-=nor(j);
        x(id) = x(id)/nor(id);
    }
    x = x.normalized();
    Vector3f y = nor.cross(x);
    Affine3f Q = Eigen::Affine3f::Identity();
    auto new_points = movePointAway({pt.x,pt.y,pt.z},normals,double(distance)/1000.0);
    for(int i=0;i<3;i++)
    {
        Q(i,0) = x(i);
        Q(i,1) = y(i);
        Q(i,2) = nor(i);
        Q(i,3) = new_points[i];
    }
    // lines.push_back({pt.x,pt.y,pt.z,new_points[0],new_points[1],new_points[2]});
    return Q;
}

vector<Affine3f> positionCameras(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr locations,unsigned int distance = 300)
{
    vector<Affine3f> cameras;
    for(int i=0;i<locations->points.size();i++)
    {
        auto Q = positionCamera(locations,i,distance);
        cameras.push_back(Q);
    }
    return cameras;
}

Affine3f optimizeCameraPosition(VoxelVolume &volume,RayTracingEngine engine, int resolution_single_dimension, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr locations,int id)
{
    unsigned int low = 300;
    unsigned int high = 600;
    unsigned int mid = low;
    while(low<high)
    {
        auto camera_location = positionCamera(locations,id,low);
        auto[found_1,good_points_low] = engine.rayTraceAndGetGoodPoints(volume,camera_location,resolution_single_dimension);
        camera_location = positionCamera(locations,id,high);
        auto[found_2,good_points_high] = engine.rayTraceAndGetGoodPoints(volume,camera_location,resolution_single_dimension);
        mid = (low+high)/2;
        if(good_points_high.size()>good_points_low.size())
        {
            low = mid+1;
        }
        else
        {
            high = mid;
        }
        cout<<"Low High Mid: "<<low<<" "<<high<<" "<<mid<<endl;
    }
    auto Q = positionCamera(locations,id,mid);
    return Q;
}

void readPointCloud(string filename,pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals=nullptr)
{
    cout<<"Inside reading function"<<endl;
#if 1
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (filename, *cloud_normal) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return;
    }
#else
    pcl::PLYReader Reader;
    Reader.read(string(argv[1]), *cloud);
#endif
    cout<<"Parsing the pointcloud"<<endl;
    for(int i=0;i<cloud_normal->points.size();i++)
    {
        PointXYZRGBNormal pt = cloud_normal->points[i];
        PointXYZRGB pt_rgb;
        Normal pt_n;
        pt_rgb.x = pt.x;
        pt_rgb.y = pt.y;
        pt_rgb.z = pt.z;
        pt_rgb.r = 0;
        pt_rgb.g = 0;
        pt_rgb.b = 0;
        // pt_n.normal = pt.normal;
        pt_n.normal[0] = pt.normal[0];
        pt_n.normal[1] = pt.normal[1];
        pt_n.normal[2] = pt.normal[2];
        cloud->points.push_back(pt_rgb);
        if(normals!=nullptr)
            normals->points.push_back(pt_n);
    }
    cout<<"Pointcloud Parsed"<<endl;

}
vector<string> filenames;
struct CameraInfo
{
    Camera cam;
    Eigen::Affine3f location;
    string id;
    CameraInfo(Camera c,Eigen::Affine3f loc,string i="camera")
    {
        cam = c;
        location = loc;
        id = i;
    }
    CameraInfo()
    {
    }
};
class VizD 
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    VoxelVolume volume;
    vector<CameraInfo> cm_;
    bool changed_;
    bool display_volume_;
    bool lines_added_;
    std::vector<std::thread> threads_;
    std::mutex mtx_;           
    void addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void addCamera(Camera cam,Eigen::Affine3f location,string id="camera");
    void addVolume();
    void removeCamera(string id="camera");
    void addLines();
    void spin();
    void input();
    public:
    VizD()
    {
        changed_ = false;
        display_volume_ = false;
        lines_added_ = false;
    }
    void make_threads();
};

void VizD::addLines()
{
    mtx_.lock();
    lines_added_ = true;
    mtx_.unlock();
}

void VizD::addCamera(Camera cam,Eigen::Affine3f location, string id)
{
    mtx_.lock();
    cm_.push_back(CameraInfo(cam,location,id));
    changed_ = true;
    mtx_.unlock();
}

void VizD::removeCamera(string id)
{
    cm_.erase(std::remove_if(cm_.begin(), cm_.end(),[id](CameraInfo c){return c.id==id;}), cm_.end());
    mtx_.lock();
    changed_ = true;
    mtx_.unlock();
}

void VizD::make_threads()
{
    threads_.push_back(std::thread(&VizD::spin, this));
    threads_.push_back(std::thread(&VizD::input, this));
    for (auto& t: threads_) t.join();
}

void VizD::spin()
{
    VisualizationUtilities::PCLVisualizerWrapper viz;
    viz.addCoordinateSystem();
    viz.addSphere({volume.xcenter_,volume.ycenter_,volume.zcenter_},"origin");
    while(viz.viewerGood())
    {
        mtx_.lock();
        if(changed_==true)
        {
            cout<<"Changed"<<endl;
            viz.updatePointCloud<pcl::PointXYZRGB>(cloud_,"cloud");
            cout<<"Number of cameras"<<cm_.size()<<endl;
            viz.viewer_->removeAllShapes();
            viz.viewer_->removeAllCoordinateSystems();
            viz.addCoordinateSystem();
            if(cm_.size())
            {
                for(int i=0;i<cm_.size();i++)
                    viz.addCamera(cm_[i].cam,cm_[i].location,"camera"+to_string(i));

            }
            if(display_volume_==true)
            {
                // viz.addPointCloudInVolumeRayTraced(volume);
                viz.addVolumeWithVoxelsClassified(volume);
                display_volume_ = false;
            }
            changed_ = false;
        }
        if(lines_added_)
        {
            for(int i=0;i<lines.size();i++)
            {
                vector<double> pt1 = {lines[i][0],lines[i][1],lines[i][2]};
                vector<double> pt2 = {lines[i][3],lines[i][4],lines[i][5]};
                viz.addLine(pt1,pt2,"dbg_line"+to_string(i));
            }
            //addlines;
        }
        mtx_.unlock();
        viz.spinViewerOnce();
    }
}

void VizD::input()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    readPointCloud(filenames[0],cloud_normal,cloud,normals);
    vector<float> K = {602.39306640625, 0.0, 314.6370849609375, 0.0, 602.39306640625, 245.04962158203125, 0.0, 0.0, 1.0};
    constexpr double PI = 3.141592653589793238462643383279502884197;
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*cloud, min_pt, max_pt);
    cout<<min_pt.x<<" "<<max_pt.x<<" "<<min_pt.y<<" "<<max_pt.y<<" "<<min_pt.z<<" "<<max_pt.z<<endl;
    volume.setDimensions(min_pt.x,max_pt.x,min_pt.y,max_pt.y,min_pt.z,max_pt.z);
    //The raycasting mechanism needs the surface to have no holes, so the resolution should be selected accordingly.
    double x_resolution = (max_pt.x-min_pt.x)*125;
    double y_resolution = (max_pt.y-min_pt.y)*125;
    double z_resolution = (max_pt.z-min_pt.z)*125;
    cout<<x_resolution<<" "<<y_resolution<<" "<<z_resolution<<endl;
    volume.setVolumeSize(int(x_resolution),int(y_resolution),int(z_resolution));
    // volume.setVolumeSize(50,50,50);
    volume.constructVolume();
    volume.integratePointCloud(cloud,normals);
    cout<<"Volume Integrated"<<endl;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr locations(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
#if 1
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (filenames[1], *locations) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return;
    }
#else
    pcl::PLYReader Reader;
    Reader.read(string(argv[2]), *cloud);
#endif
    // addCloud(cloud);
    Camera cam(K);
    auto camera_locations = positionCameras(locations);
#if 0
    for(int x=0;x<camera_locations.size();x++)
    {
        addCamera(cam,camera_locations[x],"camera"+to_string(x));
    }
#endif    
    double resolution = volume.voxel_size_;
    int resolution_single_dimension = int(round(cbrt(resolution*1e9)));
    cout<<resolution*1e9<<" Resolution"<<endl;
    cout<<"Resolution Single Dim: "<<resolution_single_dimension<<endl;

    RayTracingEngine engine(cam);
#if 1
    vector<vector<unsigned long long int>> regions_covered;
    cout<<"Printing Good Points"<<endl;
    for(int i=0;i<camera_locations.size();i++)
    {
        std::cout<<"Location: "<<i<<endl;
        auto[found,good_points] = engine.rayTraceAndGetGoodPoints(volume,camera_locations[i],resolution_single_dimension);
        // auto[found,good_points] = engine.rayTraceAndGetPoints(volume,camera_locations[i],resolution_single_dimension,false);
        sort(good_points.begin(),good_points.end());//Very important for set difference.
        regions_covered.push_back(good_points);
        // cout<<good_points.size()<<endl;
    }
    for(auto x:regions_covered)
        std::cout<<"Sizes: "<<x.size()<<endl;

    auto cameras_selected = Algorithms::greedySetCover(regions_covered,resolution);
    std::cout<<"Total Cameras: "<<camera_locations.size()<<endl;
    std::cout<<"Cameras found: "<<cameras_selected.size()<<endl;
    // engine.rayTraceAndClassify(volume,camera_locations[0],resolution_single_dimension,1,false);
    // addVolume();
    // addCamera(cam,camera_locations[0],"camera"+to_string(0));
    // sleep(5);
    // cout<<"Sleep Done"<<endl;
    // auto improved_position = optimizeCameraPosition(volume,engine,resolution_single_dimension,locations,0);
    // cout<<"Optimization done"<<endl;
    // engine.rayTraceAndClassify(volume,improved_position,resolution_single_dimension,1,false);
    // addVolume();
    // addCamera(cam,improved_position,"camera_improved"+to_string(0));
    //
    vector<Affine3f> optimized_camera_locations;
    for(auto x:cameras_selected)
    {
        auto improved_position = optimizeCameraPosition(volume,engine,resolution_single_dimension,locations,x);
        optimized_camera_locations.push_back(improved_position);
    }

    cout<<"Printing Good Points"<<endl;
    regions_covered.clear();
    assert(regions_covered.size()==0);
    for(int i=0;i<optimized_camera_locations.size();i++)
    {
        auto[found,good_points] = engine.rayTraceAndGetGoodPoints(volume,optimized_camera_locations[i],resolution_single_dimension,false);
        // auto[found,good_points] = engine.rayTraceAndGetPoints(volume,camera_locations[i],resolution_single_dimension,false);
        sort(good_points.begin(),good_points.end());//Very important for set difference.
        regions_covered.push_back(good_points);
        // cout<<good_points.size()<<endl;
    }
    for(auto x:regions_covered)
        std::cout<<"Sizes: "<<x.size()<<endl;

    auto cameras_selected_final = Algorithms::greedySetCover(regions_covered,resolution);
    for(auto x:cameras_selected_final)
    // for(int x=0;x<camera_locations.size();x++)
    // for(int x=0;x<1;x++)
    {
        static int counter = 0;
        int view = 1;
        if(counter++>13)
            view = 2;
        std::cout<<"Size of selected region: "<<regions_covered[x].size()<<endl;
        engine.rayTraceAndClassify(volume,optimized_camera_locations[x],resolution_single_dimension,view,false);
        // engine.rayTrace(volume,camera_locations[x],resolution_single_dimension,false);
        addVolume();
        sleep(5);
        addCamera(cam,optimized_camera_locations[x],"camera"+to_string(x));
        // addCloud(volume);
    } 
#else
    {
        auto Q = vectorToAffineMatrix({0.40318,0.103981,0.169578+0.5,0,0,-3.141592});
        engine.rayTraceAndClassify(volume,Q,resolution_single_dimension,false);
        // engine.rayTrace(volume,Q,resolution_single_dimension,false);
        addVolume();
        // sleep(3);
        addCamera(cam,Q,"camera"+to_string(0));
        // addCloud(volume);
    } 
#endif
    // VizD::addLines();

}

void VizD::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    mtx_.lock();
    cloud_ = cloud;
    changed_ = true;
    mtx_.unlock();
}

void VizD::addVolume()
{
    mtx_.lock();
    display_volume_ = true;
    mtx_.unlock();
}

int main(int argc, char** argv)
{
    if(argc<2)
        usage(string(argv[0]));
    filenames.push_back(string(argv[1]));
    filenames.push_back(string(argv[2]));
    VizD vd;
    vd.make_threads();
    return 0;
}
