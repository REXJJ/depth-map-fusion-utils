// C++ implementation of the above approach 
#include <bits/stdc++.h> 
#include <limits.h> 
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

using namespace std; 

// Number of cities in TSP 
#define V 10 

// Names of the cities 
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
int cal_fitness(int map[V][V], string gnome) 
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
struct individual TSPUtil(int map[V][V]) 
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
	int temperature = 10000; 

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

int main(int argc,char** argv) 
{ 
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    readPointCloud(argv[1],cloud_normal,cloud,normals);
    vector<float> K = {602.39306640625, 0.0, 314.6370849609375, 0.0, 602.39306640625, 245.04962158203125, 0.0, 0.0, 1.0};
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*cloud, min_pt, max_pt);
    cout<<"Pointcloud dimensions: "<<min_pt.x<<" "<<max_pt.x<<" "<<min_pt.y<<" "<<max_pt.y<<" "<<min_pt.z<<" "<<max_pt.z<<endl;
    //Setting up the volume.
    VoxelVolume volume;
    volume.setDimensions(min_pt.x,max_pt.x,min_pt.y,max_pt.y,min_pt.z,max_pt.z);
    //The raycasting mechanism needs the surface to have no holes, so the resolution should be selected accordingly.
    double x_resolution = (max_pt.x-min_pt.x)*125;
    double y_resolution = (max_pt.y-min_pt.y)*125;
    double z_resolution = (max_pt.z-min_pt.z)*125;
    cout<<x_resolution<<" "<<y_resolution<<" "<<z_resolution<<endl;
    volume.setVolumeSize(int(x_resolution),int(y_resolution),int(z_resolution));
    volume.constructVolume();
    volume.integratePointCloud(cloud,normals);
    auto center = {volume.xcenter_,volume.ycenter_,volume.zcenter_};
    cout<<"Volume Integrated"<<endl;

    string filename = argv[2];
    auto camera_locations = readCameraLocations(filename);
    int map[V][V];
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
    //Reading the camera locations here.
	auto best = TSPUtil(map); 
    std::cout<<"Best: "<<best.gnome<<std::endl;
    std::cout<<"Score: "<<best.fitness<<std::endl;
    //Cameras Read..

    VisualizationUtilities::PCLVisualizerWrapper viz;
    Camera cam(K);
    RayTracingEngine engine(cam);

    viz.addCoordinateSystem();
    int prev = -1;
    for(int i=0;i<V;i++)
    {
        int id = best.gnome[i]-'0';
        std::cout<<"Id: "<<id<<std::endl;
        viz.addCamera(cam,camera_locations[id],"camera"+id);
        engine.reverseRayTraceFast(volume,camera_locations[id],true,1);
        viz.addVolumeWithVoxelsClassified(volume);
        if(prev>=0)
        {
            vector<double> start = { camera_locations[prev](0,3),camera_locations[prev](1,3), camera_locations[prev](2,3) };
            vector<double> end = { camera_locations[id](0,3),camera_locations[id](1,3), camera_locations[id](2,3) };
            viz.addLine(start,end,"line"+to_string(prev)+to_string(id));
        }
        prev = id;
    }
    viz.spinViewer();
    for(int i=0;i<V;i++)
    {
        for(int j=0;j<V;j++)
            std::cout<<map[i][j]<<" ";
        std::cout<<std::endl;
    }
    return 0;

} 

