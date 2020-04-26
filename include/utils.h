#ifndef FUCKED_UTILS_H
#define FUCKED_UTILS_H

#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>


/**
 * Used for getting random part of cloud to compare and
 * @param path
 * @return
 */
int getRandomPartShifted(const std::string& path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

    //NEW
    pcl::PointCloud<pcl::PointXYZ> newCloud;
    newCloud.height=1;
    newCloud.width=400;
    newCloud.is_dense = false;
    newCloud.points.resize(newCloud.height * newCloud.width);

    for (std::size_t i = 0; i < newCloud.points.size (); ++i)
        newCloud.points[i] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};

    pcl::io::savePCDFileASCII("part.pcd", newCloud);


    return 0;
}


#endif //FUCKED_UTILS_H
