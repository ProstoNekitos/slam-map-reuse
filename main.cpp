//#include "mapReuse.h"
#include "utils.h"

int main (int argc, char** argv)
{
    /*pcl::PointCloud<pcl::PointXYZ>::Ptr full (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr part (new pcl::PointCloud<pcl::PointXYZ>);

    if ( pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/nolv/data/gear.pcd", *full) == -1 ) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    if ( pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/nolv/data/part.pcd", *part) == -1 ) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    mapReuse mr;
    mr.setMap(full);
    mr.getPosition(part);*/
    getRandomPartShifted("/home/nolv/data/gear.pcd");
}

