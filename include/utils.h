#ifndef FUCKED_UTILS_H
#define FUCKED_UTILS_H

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace utils
{
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
            PCL_ERROR ("Couldn't read file\n");
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

        float shift = 40;

        for (std::size_t i = 0; i < newCloud.points.size (); ++i)
            newCloud.points[i] = {cloud->points[i].x + shift, cloud->points[i].y + shift, cloud->points[i].z - shift};

        pcl::io::savePCDFileASCII("part.pcd", newCloud);


        return 0;
    }

    namespace stats
    {
        struct Statisctics
        {
            int max_points = 0;
            int min_points = 0;
            int summ = 0;
            double average = 0;
            int cloud_numbers = 0;
        };

        Statisctics getStats4Dir(const std::string& path)
        {
            int i = 0;
            Statisctics stats{};
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            while( pcl::io::loadPCDFile<pcl::PointXYZ> (path + std::to_string(i++) + ".pcd" , *cloud) != -1)
            {
                if( cloud->size() > stats.max_points )
                    stats.max_points = cloud->size();

                if( cloud->size() < stats.min_points || stats.min_points == 0)
                    stats.min_points = cloud->size();

                stats.summ += cloud->size();
            }

            stats.cloud_numbers = i;
            stats.average = static_cast<double>(stats.summ) / i;

            return stats;
        }

        void print(const std::string& path)
        {
            Statisctics stats = getStats4Dir(path);

            std::cout << "Statistics: \n"
            << "Max points in clouds: " << stats.max_points << '\n'
            << "Min points in clouds: " << stats.min_points << '\n'
            << "Average points in clouds: " << stats.average << '\n'
            << "Total points in clouds: " << stats.summ << '\n'
            << "Clouds read: " << stats.cloud_numbers << '\n';
        }
    }
}



#endif //FUCKED_UTILS_H
