#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>

#include <pcl/features/fpfh.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/default_convergence_criteria.h>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>


#ifndef FUCKED_MAPREUSE_H
#define FUCKED_MAPREUSE_H


class mapReuse {
public:

    /**
     * Main method
     */
    pcl::PointXYZ getPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr& in)
    {
        input = in;
        pcl::PointCloud<pcl::PointXYZ> temp;
        pcl::copyPointCloud(getKeypoints(input), temp);
        inputKeys.reset(&temp);

        inputFeatures = getDescriptors(inputKeys);

        findCorrespondences();
        rejectBadCorrespondences();
        findTransformation();
        std::cout << transform << '\n';
    }

    void
    findCorrespondences ()
    {
        pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
        est.setInputSource (inputFeatures);
        est.setInputTarget (mapFeatures);

        pcl::Correspondences corresp;
        est.determineCorrespondences(corresp);

        allCorresp.reset(&corresp);
    }

    void
    rejectBadCorrespondences ()
    {
        pcl::registration::CorrespondenceRejectorMedianDistance rej;
        rej.setMedianFactor (8.79241104);
        rej.setInputCorrespondences (allCorresp);

        pcl::Correspondences corresp;
        rej.getCorrespondences (corresp);

        goodCorresp.reset(&corresp);

        /*CorrespondencesPtr remaining_correspondences_temp (new Correspondences);
        rej.getCorrespondences (*remaining_correspondences_temp);
        PCL_DEBUG ("[rejectBadCorrespondences] Number of allCorresp remaining after rejection: %d\n", remaining_correspondences_temp->size ());

        // Reject if the angle between the normals is really off
        CorrespondenceRejectorSurfaceNormal rej_normals;
        rej_normals.setThreshold (std::acos (deg2rad (45.0)));
        rej_normals.initializeDataContainer<PointT, PointT> ();
        rej_normals.setInputCloud<PointT> (src);
        rej_normals.setInputNormals<PointT, PointT> (src);
        rej_normals.setInputTarget<PointT> (tgt);
        rej_normals.setTargetNormals<PointT, PointT> (tgt);
        rej_normals.setInputCorrespondences (remaining_correspondences_temp);
        rej_normals.getCorrespondences (remaining_correspondences);*/
    }

    void findTransformation ()
    {
        pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::FPFHSignature33, pcl::FPFHSignature33, double> trans_est;
        trans_est.estimateRigidTransformation (*mapFeatures, *inputFeatures, *goodCorresp, transform);
    }

    static pcl::PointCloud<pcl::FPFHSignature33>::Ptr getDescriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
        normal_estimation.setInputCloud (cloud);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        normal_estimation.setSearchMethod (tree);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);

        normal_estimation.setRadiusSearch (0.03);

        normal_estimation.compute (*cloud_with_normals);

        // Setup the feature computation

        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
        // Provide the original point cloud (without normals)
        fpfh_estimation.setInputCloud (cloud);
        // Provide the point cloud with normals
        fpfh_estimation.setInputNormals (cloud_with_normals);

        // fpfhEstimation.setInputWithNormals(cloud, cloudWithNormals); PFHEstimation does not have this function
        // Use the same KdTree from the normal estimation
        fpfh_estimation.setSearchMethod (tree);

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features (new pcl::PointCloud<pcl::FPFHSignature33>);

        fpfh_estimation.setRadiusSearch (0.2);

        // Actually compute the spin images
        fpfh_estimation.compute (*pfh_features);

        return pfh_features;
    }

    /**
     * finds keypoints for passed point cloud
     * @param cloud
     * @return
     */
    static pcl::PointCloud<pcl::PointWithScale> getKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        const float min_scale = 0.01f;
        const int n_octaves = 3;
        const int n_scales_per_octave = 4;
        const float min_contrast = 0.001f;

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
        pcl::PointCloud<pcl::PointWithScale> result;
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
        sift.setSearchMethod(tree);
        sift.setScales(min_scale, n_octaves, n_scales_per_octave);
        sift.setMinimumContrast(min_contrast);
        sift.setInputCloud(cloud_normals);
        sift.compute(result);

        return result;
    }

    /**
     * Sets new map, recalculates descr and keypoints
     * @param m
     */
    void setMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& m)
    {
        map = m;
        pcl::PointCloud<pcl::PointXYZ> temp;
        pcl::copyPointCloud(getKeypoints(map), temp);
        mapKeys.reset(&temp);

        mapFeatures = getDescriptors(mapKeys);

        /*pcl::visualization::PCLVisualizer viewer("PCL Viewer");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (mapKeys, 0, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (map, 255, 255, 0);

        viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
        viewer.addPointCloud(map, "cloud");
        viewer.addPointCloud(mapKeys, keypoints_color_handler, "keypoints");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

        while(!viewer.wasStopped ())
        {
            viewer.spinOnce();
        }*/
    }

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapKeys;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr mapFeatures;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputKeys;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr inputFeatures;

    pcl::CorrespondencesPtr allCorresp;
    pcl::CorrespondencesPtr goodCorresp;

    Eigen::Matrix4d transform;
};


#endif //FUCKED_MAPREUSE_H
