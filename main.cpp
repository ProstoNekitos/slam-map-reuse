#include "mapReuse.h"
#include "utils.h"

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr full (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr part (new pcl::PointCloud<pcl::PointXYZ>);

    mapReuse mr;

    if ( pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *full) == -1 ) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    mr.setMap(full);

    for( size_t i = 0; i < 2000; ++i )
    {
        if ( pcl::io::loadPCDFile<pcl::PointXYZ> (std::string(argv[2]) + std::to_string(i) + ".pcd" , *part) == -1 )
        {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            return (-1);
        } //"Development of a map reuse method in direct visual odometry algorithms"
        mr.getPosition(part);
    }
}