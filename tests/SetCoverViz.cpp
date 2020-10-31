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

void usage(string program_name)
{
    std::cout<<program_name<<" <Pointcloud filename>"<<std::endl;
    exit(-1);
}

vector<vector<double>> lines;
// lines.push_back({pt.x,pt.y,pt.z,new_points[0],new_points[1],new_points[2]});

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
class VizD : public VizThread 
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    VoxelVolume volume;
    vector<CameraInfo> cm_;
    bool display_volume_;
    bool lines_added_;
    void addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void addCamera(Camera cam,Eigen::Affine3f location,string id="camera");
    void addVolume();
    void removeCamera(string id="camera");
    void addLines();
    void input();
    void process(VisualizationUtilities::PCLVisualizerWrapper &viz);
    public:
    VizD()
    {
        display_volume_ = false;
        lines_added_ = false;
    }
};

void VizD::addLines()
{
    lines_added_ = true;
    updateViewer();
}

void VizD::addCamera(Camera cam,Eigen::Affine3f location, string id)
{
    cm_.push_back(CameraInfo(cam,location,id));
    updateViewer();
}

void VizD::removeCamera(string id)
{
    cm_.erase(std::remove_if(cm_.begin(), cm_.end(),[id](CameraInfo c){return c.id==id;}), cm_.end());
    updateViewer();
}

void VizD::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    cloud_ = cloud;
    updateViewer();
}

void VizD::addVolume()
{
    display_volume_ = true;
    updateViewer();
}

void VizD::process(VisualizationUtilities::PCLVisualizerWrapper &viz)
{
    cout<<"Changed"<<endl;
    viz.updatePointCloud<pcl::PointXYZRGB>(cloud_,"cloud");
    cout<<"Number of cameras"<<cm_.size()<<endl;
    viz.viewer_->removeAllShapes();
#if 0
    viz.viewer_->removeAllCoordinateSystems();
#else
    viz.viewer_->removeCoordinateSystem();
#endif
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
    if(lines_added_)
    {
        for(int i=0;i<lines.size();i++)
        {
            vector<double> pt1 = {lines[i][0],lines[i][1],lines[i][2]};
            vector<double> pt2 = {lines[i][3],lines[i][4],lines[i][5]};
            viz.addLine(pt1,pt2,"dbg_line"+to_string(i));
        }
        lines_added_ = false;
        //addlines;
    }
}

vector<unsigned long long int> setCover(RayTracingEngine engine, VoxelVolume &volume, vector<Affine3f> camera_locations,int resolution_single_dimension,bool sparse = true)
{
    vector<vector<unsigned long long int>> regions_covered;
    cout<<"Printing Good Points"<<endl;
    for(int i=0;i<camera_locations.size();i++)
    {
        std::cout<<"Location: "<<i<<endl;
        vector<unsigned long long int> good_points;
        bool found;
        tie(found,good_points) = engine.rayTraceAndGetGoodPoints(volume,camera_locations[i],resolution_single_dimension,sparse);
        // tie(found,good_points) = engine.rayTraceAndGetPoints(volume,camera_locations[i],resolution_single_dimension,false);
        sort(good_points.begin(),good_points.end());//Very important for set difference.
        regions_covered.push_back(good_points);
        // cout<<good_points.size()<<endl;
    }
    for(auto x:regions_covered)
        std::cout<<"Sizes: "<<x.size()<<endl;

    auto cameras_selected = Algorithms::greedySetCover(regions_covered);
    std::cout<<"Total Cameras: "<<camera_locations.size()<<endl;
    std::cout<<"Cameras found: "<<cameras_selected.size()<<endl;
    return cameras_selected;
}

void VizD::input()
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    readPointCloud(filenames[0],cloud_normal,cloud,normals);
    vector<float> K = {602.39306640625, 0.0, 314.6370849609375, 0.0, 602.39306640625, 245.04962158203125, 0.0, 0.0, 1.0};
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*cloud, min_pt, max_pt);
    cout<<"Pointcloud dimensions: "<<min_pt.x<<" "<<max_pt.x<<" "<<min_pt.y<<" "<<max_pt.y<<" "<<min_pt.z<<" "<<max_pt.z<<endl;
    //Setting up the volume.
    volume.setDimensions(min_pt.x,max_pt.x,min_pt.y,max_pt.y,min_pt.z,max_pt.z);
    //The raycasting mechanism needs the surface to have no holes, so the resolution should be selected accordingly.
    double x_resolution = (max_pt.x-min_pt.x)*125;
    double y_resolution = (max_pt.y-min_pt.y)*125;
    double z_resolution = (max_pt.z-min_pt.z)*125;
    cout<<x_resolution<<" "<<y_resolution<<" "<<z_resolution<<endl;
    volume.setVolumeSize(int(x_resolution),int(y_resolution),int(z_resolution));
    volume.constructVolume();
    volume.integratePointCloud(cloud,normals);
    cout<<"Volume Integrated"<<endl;
    //Setting up the camera locations
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
    auto camera_locations = readCameraLocations("cameras.tmp");
    for(auto x:camera_locations)
    {
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<4;j++)
                cout<<x(i,j)<<" ";
            cout<<endl;
        }
        cout<<"---------------------------------"<<endl;
    }
    double resolution = volume.voxel_size_;
    int resolution_single_dimension = int(round(cbrt(resolution*1e9)));
    cout<<resolution*1e9<<" Resolution"<<endl;
    cout<<"Resolution Single Dim: "<<resolution_single_dimension<<endl;

    /* Setting up the ray tracer.*/
    RayTracingEngine engine(cam);

    /*Displaying the results.*/
    for(int x=0;x<camera_locations.size();x++)
    {
        int view = 1;
        engine.rayTraceAndClassify(volume,camera_locations[x],resolution_single_dimension,view,false);
        // engine.rayTrace(volume,camera_locations[x],resolution_single_dimension,false);
        addCamera(cam,camera_locations[x],"camera"+to_string(x));
        sleep(3);
        addVolume();
        sleep(2);
    } 
    // VizD::addLines();
}

int main(int argc, char** argv)
{
    if(argc<2)
        usage(string(argv[0]));
    filenames.push_back(string(argv[1]));
    filenames.push_back(string(argv[2]));
    VizD vd;
    vd.makeThreads();
    return 0;
}
