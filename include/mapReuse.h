#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/harris_6d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/susan.h>

#include <pcl/features/normal_3d.h>

#include <pcl/features/fpfh.h>

#include "Params.h"
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
//#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/default_convergence_criteria.h>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>


#ifndef FUCKED_MAPREUSE_H
#define FUCKED_MAPREUSE_H

class mapReuse {
public:

    mapReuse() :
    map(new pcl::PointCloud<pcl::PointXYZ>),
    mapKeys(new pcl::PointCloud<pcl::PointXYZ>),
    mapFeatures(new pcl::PointCloud<pcl::FPFHSignature33>),

    input(new pcl::PointCloud<pcl::PointXYZ>),
    inputKeys(new pcl::PointCloud<pcl::PointXYZ>),
    inputFeatures(new pcl::PointCloud<pcl::FPFHSignature33>),

    tempInput(new pcl::PointCloud<pcl::PointXYZ>),
    tempInputKeys(new pcl::PointCloud<pcl::PointXYZ>),
    tempInputFeatures(new pcl::PointCloud<pcl::FPFHSignature33>),

    transformed(new pcl::PointCloud<pcl::PointXYZ>),

    allCorresp(new pcl::Correspondences),
    goodCorresp(new pcl::Correspondences)
    {}


    pcl::PointXYZ getPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr& in)
    {
        switch( parameters.mode )
        {
            case 0:
                input = in;
                pcl::copyPointCloud(harrisKP(input), *inputKeys);
                break;

            case 1:
                *input += *in;
                pcl::copyPointCloud(harrisKP(input), *inputKeys);
                break;

            default:
                std::cout << "Something wrong\n";
        }

        inputFeatures = getDescriptors(inputKeys);

        findCorrespondences();

        //rejectBadCorrespondences();
        getTransformation();
        //std::cout << transformation << "\n\n";

        std::cout << '\n' << input->size() << '\n';
        std::cout << '\n' << inputKeys->size() << '\n';


        return {};
    }

    static pcl::PointCloud<pcl::PointWithScale> siftKP(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        const float min_scale = 0.01f;
        const int n_octaves = 3;
        const int n_scales_per_octave = 4;
        const float min_contrast = 0.005f;

        //Normal estimation
        pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());

        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree_n);
        ne.setRadiusSearch(0.2);
        ne.compute(*cloud_normals);

        for(std::size_t i = 0; i<cloud_normals->points.size(); ++i)
        {
            cloud_normals->points[i].x = cloud->points[i].x;
            cloud_normals->points[i].y = cloud->points[i].y;
            cloud_normals->points[i].z = cloud->points[i].z;
        }

        pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());

        sift.setSearchMethod(tree);
        sift.setScales(min_scale, n_octaves, n_scales_per_octave);
        sift.setMinimumContrast(min_contrast);
        sift.setInputCloud(cloud_normals);

        pcl::PointCloud<pcl::PointWithScale> result;
        sift.compute(result);

        return result;
    }

    static pcl::PointCloud<pcl::PointXYZI> harrisKP(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> harris;
        harris.setNonMaxSupression (true);
        harris.setRefine(false);
        harris.setThreshold (1e-7);
        harris.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ,pcl::PointXYZI>::HARRIS);
        harris.setInputCloud(cloud);
        //harris.setRadiusSearch(100);

        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
        harris.compute(*keypoints);

        return *keypoints;
    }

    static pcl::PointCloud<pcl::PointXYZ> issKP(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        double cloud_resolution (0.0058329);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
        iss_detector.setSearchMethod (tree);
        iss_detector.setSalientRadius (6 * cloud_resolution);
        iss_detector.setNonMaxRadius (4 * cloud_resolution);

        iss_detector.setThreshold21 (0.975);
        iss_detector.setThreshold32 (0.975);
        iss_detector.setMinNeighbors (3);
        iss_detector.setNumberOfThreads (1);
        iss_detector.setInputCloud (cloud);

        pcl::PointCloud<pcl::PointXYZ> keypoints;
        iss_detector.compute (keypoints);

        return keypoints;
    }

    void
    findCorrespondences ()
    {
        pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33,pcl::FPFHSignature33> est;
        est.setInputSource (inputFeatures);
        est.setInputTarget (mapFeatures);

        est.determineCorrespondences(*allCorresp);
        std::cout << "Break point\n";
    }

    void
    rejectBadCorrespondences ()
    {
        pcl::registration::CorrespondenceRejectorMedianDistance rej;
        rej.setMedianFactor (8.79241104);
        rej.setInputCorrespondences (allCorresp);

        rej.getCorrespondences (*goodCorresp);
    }

    void getTransformation ()
    {
        pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>::Ptr
                transformation_estimation(
                new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,
                        pcl::PointXYZ>);

        transformation_estimation->estimateRigidTransformation(
                *inputKeys,
                *mapKeys,
                *allCorresp,
                initial_transformation_matrix_);

        pcl::transformPointCloud(
                *inputKeys, *transformed, initial_transformation_matrix_);

        std::cout << "A\n";
        std::cout << initial_transformation_matrix_ << '\n';
        std::cout << "A\n";
    }

    static pcl::PointCloud<pcl::FPFHSignature33>::Ptr getDescriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        //Normals
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        normal_estimation.setSearchMethod (tree);
        normal_estimation.setRadiusSearch(0.004);
        normal_estimation.setInputCloud (cloud);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);
        normal_estimation.compute (*cloud_with_normals);

        //Features
        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;

        fpfh_estimation.setInputCloud (cloud);
        fpfh_estimation.setInputNormals (cloud_with_normals);
        fpfh_estimation.setRadiusSearch (0.2);
        fpfh_estimation.setSearchMethod (tree);

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features (new pcl::PointCloud<pcl::FPFHSignature33>);
        fpfh_estimation.compute (*pfh_features);

        return pfh_features;
    }

    /**
     * Sets new map, recalculates descr and keypoints
     * @param m
     */
    void setMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& m)
    {
        map = m;
        pcl::copyPointCloud(harrisKP(map), *mapKeys);
        mapFeatures = getDescriptors(mapKeys);
        std::cout << "Stop point" << '\n';
    }

    void visualize()
    {
        pcl::visualization::PCLVisualizer  viewer ("Correspondence Viewer");
        viewer.setBackgroundColor (0, 0, 0);

        //Color handlers
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> mapColorHandler (map, 255, 8, 49);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> inputColorHandler (input, 123, 22, 255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> mapKeyColorHandler (mapKeys, 255, 181, 34);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> inputKeyColorHandler (inputKeys, 60, 255, 34);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformedColorHandler (transformed, 255, 255, 255);


        //Adds clouds
        viewer.addPointCloud<pcl::PointXYZ> (map, mapColorHandler, "map");
        viewer.addPointCloud<pcl::PointXYZ> (input, inputColorHandler,  "input");
        viewer.addPointCloud<pcl::PointXYZ> (mapKeys, mapKeyColorHandler, "mapKey");
        viewer.addPointCloud<pcl::PointXYZ> (inputKeys, inputKeyColorHandler, "inputKey");
        viewer.addPointCloud<pcl::PointXYZ> (transformed, transformedColorHandler, "transformed");


        //Point size
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "map");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "mapKey");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "inputKey");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "transformed");


        viewer.addCorrespondences<pcl::PointXYZ> (mapKeys,inputKeys,*allCorresp,"Correspondences");

        while(!viewer.wasStopped ())
        {
            viewer.spinOnce();
        }
    }

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapKeys;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr mapFeatures;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputKeys;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr inputFeatures;

    pcl::PointCloud<pcl::PointXYZ>::Ptr tempInput;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempInputKeys;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr tempInputFeatures;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed;

    pcl::CorrespondencesPtr allCorresp;
    pcl::CorrespondencesPtr goodCorresp;

    Eigen::Matrix4f initial_transformation_matrix_;
    Eigen::Matrix4f transformation_matrix_;

    Params parameters;
};


#endif //FUCKED_MAPREUSE_H
