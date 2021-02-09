#include <iostream>
#include <assert.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include<octomap/OcTreeBase.h>

using namespace std;

int main( int argc, char** argv )
{
    if (argc != 2)
    {
        cout<<"Usage: pcd2octomap <input_file>"<<endl;
        return -1;
    }
    string input_file = argv[1];
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::io::loadPCDFile<pcl::PointXYZRGBA> ( input_file, cloud );
    cout<<"point cloud loaded, piont size = "<<cloud.points.size()<<endl;
    cout<<"copy data into octomap..."<<endl;
    octomap::OcTree tree( 0.005 );

    for (auto p:cloud.points)
    {
        tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }
    tree.updateInnerOccupancy();
    // tree.writeBinary( output_file );
    cout<<"done."<<endl;

    pcl::PointCloud<pcl::PointXYZ> cloud_new;
    for(auto it = tree.begin_leafs(),
            end=tree.end_leafs(); it!= end; ++it)
    {
        // std::cout << "Node center: " << it.getCoordinate() << std::endl;
        // std::cout << "Node size: " << it.getSize() << std::endl;
        // std::cout << "Node value: " << it->getValue() << std::endl;
        auto t = it.getCoordinate();
        pcl::PointXYZ pt;
        pt.x = t(0);
        pt.y = t(1);
        pt.z = t(2);
        cloud_new.points.push_back(pt);
    }
    cloud_new.width = cloud_new.points.size();
    cloud_new.height = 1;
    //save cloud
    pcl::io::savePCDFileASCII ("file.pcd", cloud_new);
    return 0;
}
