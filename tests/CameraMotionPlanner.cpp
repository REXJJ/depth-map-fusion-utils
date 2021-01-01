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
#include <TSPSolver.hpp>

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace PointCloudProcessing;
using namespace TransformationUtilities;
using namespace Algorithms;

// Number of cities in TSP 
int V = 0;
#define GENES ABCDE 

// Starting Node Value 
#define START 0 

// Initial population size for the algorithm 
#define POP_SIZE 10 

// Structure of a GNOME 
// string defines the path traversed 
// by the salesman while the fitness value 
// of the path is stored in an integer 

struct individual { 
	string gnome; 
	int fitness; 
}; 

// Function to return a random number 
// from start and end 
int rand_num(int start, int end) 
{ 
	int r = end - start; 
	int rnum = start + rand() % r; 
	return rnum; 
} 

// Function to check if the character 
// has already occurred in the string 
bool repeat(string s, char ch) 
{ 
	for (int i = 0; i < s.size(); i++) { 
		if (s[i] == ch) 
			return true; 
	} 
	return false; 
} 

// Function to return a mutated GNOME 
// Mutated GNOME is a string 
// with a random interchange 
// of two genes to create variation in species 
string mutatedGene(string gnome) 
{ 
	while (true) { 
		int r = rand_num(1, V); 
		int r1 = rand_num(1, V); 
		if (r1 != r) { 
			char temp = gnome[r]; 
			gnome[r] = gnome[r1]; 
			gnome[r1] = temp; 
			break; 
		} 
	} 
	return gnome; 
} 

// Function to return a valid GNOME string 
// required to create the population 
string create_gnome() 
{ 
	string gnome = "0"; 
	while (true) { 
		if (gnome.size() == V) { 
			gnome += gnome[0]; 
			break; 
		} 
		int temp = rand_num(1, V); 
		if (!repeat(gnome, (char)(temp + 48))) 
			gnome += (char)(temp + 48); 
	} 
	return gnome; 
} 

// Function to return the fitness value of a gnome. 
// The fitness value is the path length 
// of the path represented by the GNOME. 
int cal_fitness(vector<vector<int>> map, string gnome) 
{ 
	int f = 0; 
	for (int i = 0; i < gnome.size() - 1; i++) { 
		if (map[gnome[i] - 48][gnome[i + 1] - 48] == INT_MAX) 
			return INT_MAX; 
		f += map[gnome[i] - 48][gnome[i + 1] - 48]; 
	} 
	return f; 
} 

// Function to return the updated value 
// of the cooling element. 
int cooldown(int temp) 
{ 
	return (90 * temp) / 100; 
} 

// Comparator for GNOME struct. 
bool lessthan(struct individual t1, 
			struct individual t2) 
{ 
	return t1.fitness < t2.fitness; 
} 

// Utility function for TSP problem. 
struct individual TSPUtil(vector<vector<int>> map) 
{ 
	// Generation Number 
	int gen = 1; 
	// Number of Gene Iterations 
	int gen_thres = 500; 

	vector<struct individual> population; 
	struct individual temp; 

	// Populating the GNOME pool. 
	for (int i = 0; i < POP_SIZE; i++) { 
		temp.gnome = create_gnome(); 
		temp.fitness = cal_fitness(map, temp.gnome); 
		population.push_back(temp); 
	} 

	cout << "\nInitial population: " << endl 
		<< "GNOME	 FITNESS VALUE\n"; 
	for (int i = 0; i < POP_SIZE; i++) 
		cout << population[i].gnome << " "
			<< population[i].fitness << endl; 
	cout << "\n"; 

	bool found = false; 
	int temperature = 1000000; 

	// Iteration to perform 
	// population crossing and gene mutation. 
	while (temperature > 1000 && gen <= gen_thres) { 
		sort(population.begin(), population.end(), lessthan); 
		cout << "\nCurrent temp: " << temperature << "\n"; 
		vector<struct individual> new_population; 

		for (int i = 0; i < POP_SIZE; i++) { 
			struct individual p1 = population[i]; 

			while (true) { 
				string new_g = mutatedGene(p1.gnome); 
				struct individual new_gnome; 
				new_gnome.gnome = new_g; 
				new_gnome.fitness = cal_fitness(map,new_gnome.gnome); 

				if (new_gnome.fitness <= population[i].fitness) { 
					new_population.push_back(new_gnome); 
					break; 
				} 
				else { 

					// Accepting the rejected children at 
					// a possible probablity above threshold. 
					float prob = pow(2.7, 
									-1 * ((float)(new_gnome.fitness 
												- population[i].fitness) 
										/ temperature)); 
					if (prob > 0.5) { 
						new_population.push_back(new_gnome); 
						break; 
					} 
				} 
			} 
		} 

		temperature = cooldown(temperature); 
		population = new_population; 
		cout << "Generation " << gen << " \n"; 
		cout << "GNOME	 FITNESS VALUE\n"; 

		for (int i = 0; i < POP_SIZE; i++) 
			cout << population[i].gnome << " "
				<< population[i].fitness << endl; 
		gen++; 
	} 
    auto best = *std::min_element(population.begin(),population.end(),[](struct individual a, struct individual b){ return a.fitness < b.fitness; });
    std::cout<<"Best: "<<best.gnome<<std::endl;
    std::cout<<"Score: "<<best.fitness<<std::endl;
    return best;
} 

double euclideanDistance(Vector3f a, Vector3f b)
{
    return (a-b).norm();
}

bool willCollide(VoxelVolume& volume, Vector3f a, Vector3f b)
{
    double distance = (a-b).norm();
    std::cout<<"Distance: "<<distance<<std::endl;
    Vector3f v = (b-a).normalized();
    bool collided = false;
    for(int depth=1;collided==false;depth++){
        Vector3f pt = a + v*double(depth)/1000.0;
        double xx = pt(0);
        double yy = pt(1);
        double zz = pt(2);
        if(depth>distance*1000)
            break;
        if(volume.validPoints(xx,yy,zz)==false)
            continue;
        auto coords = volume.getVoxel(xx,yy,zz);
        int xidn = get<0>(coords);
        int yidn = get<1>(coords);
        int zidn = get<2>(coords);
        if( volume.voxels_[xidn][yidn][zidn]!=nullptr )
        {
            collided = true;
        }
    }
    return collided;
}

void usage(string program_name)
{
    std::cout<<program_name<<" <Pointcloud filename>"<<std::endl;
    exit(-1);
}

vector<vector<double>> lines;
// lines.push_back({pt.x,pt.y,pt.z,new_points[0],new_points[1],new_points[2]});
//
vector<float> center;

struct CameraInfo
{
    Camera cam;
    Eigen::Affine3f location;
    string id;
    int side;
    CameraInfo(Camera c,Eigen::Affine3f loc,string i="camera")
    {
        cam = c;
        location = loc;
        id = i;
        vector<Vector3f> directions = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};//RLFBUD
        Vector3f camera_vector = {center[0]-loc(0,3),center[1]-loc(1,3),center[2]-loc(2,3)};
        // vector<double> ctt = {fabs(center[0]-loc(0,3)),fabs(center[1]-loc(1,3)),fabs(center[2]-loc(2,3))};
        // side = max_element(ctt.begin(),ctt.end())-ctt.begin();
        // if(center[side]-ctt[side]<0)
            // side = side+3;
        camera_vector = camera_vector.normalized();
        for(int i=0;i<directions.size();i++)
        {
            std::cout<<acos(camera_vector.dot(directions[i]))<<" ";
        }
        std::cout<<std::endl;
        side = min_element(directions.begin(),directions.end(),[camera_vector](Vector3f a,Vector3f b){return acos(camera_vector.dot(a))<acos(camera_vector.dot(b));})-directions.begin();
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
    vector<CameraInfo> cm;
    bool display_volume_;
    bool lines_added_;
    public:
    void input();
    VizD()
    {
        display_volume_ = false;
        lines_added_ = false;
    }
};

int spheres = 0;
vector<string> filenames;

vector<unsigned long long int> setCover(RayTracingEngine engine, VoxelVolume &volume, vector<Affine3f> camera_locations,int resolution_single_dimension,bool sparse = true)
{
    vector<vector<unsigned long long int>> regions_covered;
    cout<<"Printing Good Points"<<endl;
    for(int i=0;i<camera_locations.size();i++)
    {
        std::cout<<"Location: "<<i<<endl;
        vector<unsigned long long int> good_points;
        bool found;
        tie(found,good_points) = engine.reverseRayTraceFast(volume,camera_locations[i],false);
        // tie(found,good_points) = engine.rayTraceAndGetPoints(volume,camera_locations[i],resolution_single_dimension,false);
        sort(good_points.begin(),good_points.end());//Very important for set difference.
        regions_covered.push_back(good_points);
        // cout<<good_points.size()<<endl;
    }
    for(auto x:regions_covered)
        std::cout<<"Sizes: "<<x.size()<<endl;

    auto cameras_selected = Algorithms::greedySetCover(regions_covered);
    std::cout<<"Total Camera Locations: "<<camera_locations.size()<<endl;
    std::cout<<"Total Camera Locations Found: "<<cameras_selected.size()<<endl;
    return cameras_selected;
}


vector<Affine3f> filterCameras(vector<Affine3f> cameras)
{
    vector<Affine3f> cams;
    for(int i=0;i<cameras.size();i++)
    {
        if(cameras[i](2,3)<0)
            continue;
        cams.push_back(cameras[i]);
    }
    return cams;
}

void TSPCamera(VoxelVolume& volume,Camera& cam, RayTracingEngine engine, vector<Affine3f>& camera_locations)
{
    V = camera_locations.size();
    vector<vector<int>> map = vector<vector<int>>(V,vector<int>(V,0));
    for(int i=0;i<V;i++)
    {
        for(int j=0;j<V;j++)
        {
            std::cout<<"I,J: "<<i<<" "<<j<<std::endl;
            Vector3f a = { camera_locations[i](0,3),camera_locations[i](1,3),camera_locations[i](2,3) };
            Vector3f b = { camera_locations[j](0,3),camera_locations[j](1,3),camera_locations[j](2,3) };
            if(willCollide(volume,a,b)==true)
            {
                std::cout<<"Collided"<<std::endl;
                map[i][j] = INT_MAX;
            }
            else
                map[i][j] =  euclideanDistance(a,b)*1000;
        }
    }
    VisualizationUtilities::PCLVisualizerWrapper viz;
    viz.addCoordinateSystem();

    viz.addVolume(volume);
    TSP::NearestNeighborSearch nnsolver(map);
    nnsolver.solve();
    std::cout<<"Nearest Neighbor Path Length: "<<nnsolver.getPathLength()<<std::endl;
    TSP::TwoOptSearch twsolver(map);
    twsolver.path_ = nnsolver.path_;
    twsolver.solve();
    std::cout<<"Two Opt Path Length: "<<twsolver.getPathLength()<<std::endl;
	// auto best = TSPUtil(map); 
    int prev = -1;
    vector<int> path;
    for(int i=0;i<V;i++)
    {
        // int id = best.gnome[i]-'0';
        int id = twsolver.path_[i];
        path.push_back(id);
        std::cout<<"Id: "<<id<<std::endl;
        viz.addCamera(cam,camera_locations[id],"camera"+id);
        engine.reverseRayTraceFast(volume,camera_locations[id],true,1);
        viz.addVolumeWithVoxelsClassified(volume);
        if(prev>=0)
        {
            vector<double> start = { camera_locations[prev](0,3),camera_locations[prev](1,3), camera_locations[prev](2,3) };
            vector<double> end = { camera_locations[id](0,3),camera_locations[id](1,3), camera_locations[id](2,3) };
            viz.addLine(start,end,"line_path"+to_string(prev)+to_string(id));
        }
        prev = id;
    }
    viz.spinViewer();
}

void VizD::input()
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    readPointCloud(filenames[0],cloud_normal,cloud,normals);
    vector<float> K = {602.39306640625, 0.0, 314.6370849609375, 0.0, 602.39306640625, 245.04962158203125, 0.0, 0.0, 1.0};//TODO: Read from a parameter file.
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*cloud, min_pt, max_pt);
    cout<<"Pointcloud dimensions: "<<min_pt.x<<" "<<max_pt.x<<" "<<min_pt.y<<" "<<max_pt.y<<" "<<min_pt.z<<" "<<max_pt.z<<endl;
    //Setting up the volume.
    volume.setDimensions(min_pt.x,max_pt.x,min_pt.y,max_pt.y,min_pt.z,max_pt.z);
    //The raycasting mechanism needs the surface to have no holes, so the resolution should be selected accordingly.
    double x_resolution = (max_pt.x-min_pt.x)*63;//TODO: Constexpr this.
    double y_resolution = (max_pt.y-min_pt.y)*63;
    double z_resolution = (max_pt.z-min_pt.z)*63;
    cout<<x_resolution<<" "<<y_resolution<<" "<<z_resolution<<endl;
    volume.setVolumeSize(int(x_resolution),int(y_resolution),int(z_resolution));
    volume.constructVolume();
    volume.integratePointCloud(cloud,normals);
    center = {volume.xcenter_,volume.ycenter_,volume.zcenter_};
    cout<<"Volume Integrated"<<endl;
    //Setting up the camera locations
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr locations(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    downsample<pcl::PointXYZRGBNormal>(cloud_normal,locations,0.1);//May revert back to sphere or cylinder based camera locations.
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr locations_new(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    auto camera_locations = filterCameras(positionCameras(locations,500));
    
    Camera cam(K);

    double resolution = volume.voxel_size_;
    int resolution_single_dimension = int(round(cbrt(resolution*1e9)));
    cout<<resolution*1e9<<" Resolution"<<endl;
    cout<<"Resolution Single Dim: "<<resolution_single_dimension<<endl;

    /* Setting up the ray tracer.*/
    RayTracingEngine engine(cam);
    auto cameras_selected = setCover(engine,volume,camera_locations,resolution_single_dimension,false);

    /*Optimizing location of the selected cameras.*/
    vector<Affine3f> optimized_camera_locations;
    for(auto x:cameras_selected)
    {
        auto improved_position = optimizeCameraPosition(volume,engine,resolution_single_dimension,camera_locations[x]);
        optimized_camera_locations.push_back(improved_position);
    }

    /*Second run of set cover.*/
    cameras_selected = setCover(engine,volume,optimized_camera_locations,resolution_single_dimension,false);

    // addVolume();

    // for(int i=0;i<optimized_camera_locations.size();i++)
    // {
    //     addCamera(cam,optimized_camera_locations[i],"camera"+to_string(i));
    // }

    // for(int x = 0;x<cameras_selected.size();x++)
    // {
    //
    //     // int view = 1;
    //     // engine.reverseRayTraceFast(volume,optimized_camera_locations[x],true);
    //     addCamera(cam,optimized_camera_locations[x],"camera"+to_string(x));
    //     // sleep(3);
    //     // addVolume();
    //     // sleep(2);
    //     // removeCamera("camera"+to_string(x));
    //     // std::cout<<"Camera: "<<x<<endl;
    // } 
    //
    /*Saving the results.*/
    vector<Affine3f> locations_to_save;
    for(auto x:cameras_selected)
        locations_to_save.push_back(optimized_camera_locations[x]);

    string temp_file = filenames[1];
    std::cout<<"Saving outputs in : "<<temp_file<<std::endl;
    writeCameraLocations(temp_file,locations_to_save);

    TSPCamera(volume,cam,engine,locations_to_save);




    // for(auto x:cameras_selected)
    // {
        // addCamera(cam,camera_locations[x],"camera"+to_string(x));
    // }

}

int main(int argc, char** argv)
{
    if(argc<2)
        usage(string(argv[0]));
    filenames.push_back(string(argv[1]));
    filenames.push_back(string(argv[2]));
    VizD vd;
    vd.input();
    // vd.makeThreads();
    return 0;
}
