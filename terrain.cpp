#include "terrain.h"
#include "HoughTransform.h"
#include "hull.h"
#include "cloud.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/octree/octree_search.h>

#include <string>

OctreeTerrain::OctreeTerrain()
{
    m_baseCloud = new Cloud();
    m_vegetation = new Cloud();
    m_terrain = new Cloud();
    m_resolution = 0.1;
}

OctreeTerrain::OctreeTerrain(Cloud input, float resolution)
{
    m_baseCloud = new Cloud();
    m_vegetation = new Cloud();
    m_terrain = new Cloud();
    *m_baseCloud = input;
    m_resolution = resolution;
}

OctreeTerrain::~OctreeTerrain()
{
    delete m_baseCloud;
    delete m_vegetation;
    delete m_terrain;
}

void OctreeTerrain::setResolution(float res)
{
    m_resolution = res;
}

void OctreeTerrain::setBaseCloud(Cloud input)
{
    m_baseCloud->set_Cloud(input.get_Cloud());
}

void OctreeTerrain::setVegetationName(std::string vege_name)
{
    m_vegetation->set_name(vege_name);
}

void OctreeTerrain::setTerrainName(std::string terrain_name)
{
    m_terrain->set_name(terrain_name);
}

void OctreeTerrain::octree(float res, pcl::PointCloud<pcl::PointXYZI>::Ptr input,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr output_ground,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr output_vege)
{

    // udelat octree
    pcl::octree::OctreePointCloud<pcl::PointXYZI> oc(res);
    oc.setInputCloud(input);
    oc.addPointsFromInputCloud();
    // zjistit vsechny voxely
    std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> voxels;
    oc.getOccupiedVoxelCenters(voxels);

    // zjistit rozsah x y osy x podle toho hledat voxely ktere jsou nejníž
    double x_max, x_min, y_max, y_min, z_min, z_max;
    oc.getBoundingBox(x_min, y_min, z_min, x_max, y_max, z_max);

    oc.deleteTree();
    // z voxels udelat mracno bodu
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxels(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_voxels->points.resize(voxels.size());
#pragma omp parallel for
    for (int r = 0; r < voxels.size(); r++)
    {
        cloud_voxels->points.at(r) = voxels.at(r);
    }
    cloud_voxels->width = cloud_voxels->points.size();
    cloud_voxels->height = 1;
    cloud_voxels->is_dense = true;

    // spis boxsearch x pro kazdy voxel najit sousedy v danem boxu, pokud nenajde žadny bod niž než je on sam uložit jeho ID..
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> ocs(res);

    ocs.setInputCloud(cloud_voxels);
    ocs.addPointsFromInputCloud();
    std::vector<int> low_voxels;
    for (int q = 0; q < voxels.size(); q++)
    {
        std::vector<int> ind;
        Eigen::Vector3f low(voxels.at(q).x - res / 2, voxels.at(q).y - res / 2, z_min);
        Eigen::Vector3f high(voxels.at(q).x + res / 2, voxels.at(q).y + res / 2, voxels.at(q).z);
        if (ocs.boxSearch(low, high, ind) < 3)
        {
            if (ind.size() == 0)
                continue;
            // pokud jsou voxely vyskove pouze res od sebe
            if (ind.size() == 1)
                low_voxels.push_back(q);
            else
            {
                if (std::abs(voxels.at(ind.at(0)).z - voxels.at(ind.at(1)).z) < (res * 1.1))
                    low_voxels.push_back(q);
            }
        }
    }
    ocs.deleteTree();

    // get point of lowest voxels
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> ocsearch(res);
    ocsearch.setInputCloud(input);
    ocsearch.addPointsFromInputCloud();
    std::vector<int> low_voxels_indices;
    for (int u = 0; u < low_voxels.size(); u++)
    {
        ocsearch.voxelSearch(voxels.at(low_voxels.at(u)), low_voxels_indices);
    }
    ocsearch.deleteTree();
    // ocs.voxelSearch(voxels.at(q),low_voxels_indices);

    std::shared_ptr<std::vector<int>> indicesptr(new std::vector<int>(low_voxels_indices));
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    // Extract the inliers
    extract.setInputCloud(input);
    extract.setIndices(indicesptr);
    extract.setNegative(false);
    extract.filter(*output_ground);
    extract.setNegative(true);
    extract.filter(*output_vege);
}

void OctreeTerrain::execute()
{
    // qWarning()<<"octree terrain starts";

    // velky cyklus
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp2(new pcl::PointCloud<pcl::PointXYZI>);
    octree(m_resolution * 5, m_baseCloud->get_Cloud(), cloud_tmp, cloud_tmp2);
    cloud_tmp2->points.clear();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp3(new pcl::PointCloud<pcl::PointXYZI>);
    octree(m_resolution / 2, cloud_tmp, cloud_tmp3, cloud_tmp2);
    cloud_tmp2->points.clear();
    cloud_tmp->points.clear();

    // maly cyklus
    octree(m_resolution, m_baseCloud->get_Cloud(), cloud_tmp, cloud_tmp2);
    cloud_tmp2.reset();

    // porovnat maly x velky cyklus
    std::vector<int> pointID_ground;

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud_tmp);

#pragma omp parallel
    {
        std::vector<int> points_ground;
#pragma omp for nowait // fill vec_private in parallel
        for (int i = 0; i < cloud_tmp3->points.size(); i++)
        {
            pcl::PointXYZI searchPointV;
            searchPointV = cloud_tmp3->points.at(i);
            std::vector<int> pointIDv;
            std::vector<float> pointSDv;
            if (kdtree.radiusSearch(searchPointV, 0.001, pointIDv, pointSDv) > 0)
                points_ground.push_back(i);
        }
#pragma omp critical
        {
            if (points_ground.size() > 0)
                pointID_ground.insert(pointID_ground.end(), points_ground.begin(), points_ground.end());
        }
    }

    std::shared_ptr<std::vector<int>> indices_ground(new std::vector<int>(pointID_ground));
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    // Extract the inliers
    extract.setInputCloud(cloud_tmp3);
    extract.setIndices(indices_ground);
    // terrain
    extract.setNegative(false);
    extract.filter(*cloud_ground);
    cloud_ground->width = cloud_ground->points.size();
    cloud_ground->height = 1;
    cloud_ground->is_dense = true;
    m_terrain->set_Cloud(cloud_ground);
    cloud_tmp3.reset();
    cloud_tmp.reset();

    // vegetace
    std::vector<int> pointIDS;
    pcl::KdTreeFLANN<pcl::PointXYZI> k;
    k.setInputCloud(m_baseCloud->get_Cloud());

#pragma omp parallel
    {
        std::vector<int> points_ground;
#pragma omp for nowait // fill vec_private in parallel
        for (int i = 0; i < cloud_ground->points.size(); i++)
        {
            pcl::PointXYZI searchPointV;
            searchPointV = cloud_ground->points.at(i);
            std::vector<int> pointIDv;
            std::vector<float> pointSDv;
            if (k.radiusSearch(searchPointV, 0.001, pointIDv, pointSDv) > 0)
                points_ground.push_back(pointIDv.at(0));
        }
#pragma omp critical
        {
            if (points_ground.size() > 0)
                pointIDS.insert(pointIDS.end(), points_ground.begin(), points_ground.end());
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_vege(new pcl::PointCloud<pcl::PointXYZI>);
    std::shared_ptr<std::vector<int>> indicesptr(new std::vector<int>(pointIDS));
    pcl::ExtractIndices<pcl::PointXYZI> e;
    // Extract the inliers
    e.setInputCloud(m_baseCloud->get_Cloud());
    e.setIndices(indicesptr);
    // vege
    e.setNegative(true);
    e.filter(*cloud_vege);
    cloud_vege->width = cloud_vege->points.size();
    cloud_vege->height = 1;
    cloud_vege->is_dense = true;
    m_vegetation->set_Cloud(cloud_vege);

    cloud_vege.reset();
    cloud_ground.reset();
}

// StatOutlierRemoval
StatOutlierRemoval::StatOutlierRemoval()
{
    m_baseCloud = new Cloud();
    m_output = new Cloud();
    m_mDist = 0.1;
    m_neighbors = 12;
}

StatOutlierRemoval::~StatOutlierRemoval()
{
    delete m_baseCloud;
    delete m_output;
}

void StatOutlierRemoval::setMeanDistance(float distance)
{
    m_mDist = distance;
}

void StatOutlierRemoval::setNeighborhood(int n)
{
    m_neighbors = n;
}

void StatOutlierRemoval::setBaseCloud(Cloud input)
{
    m_baseCloud->set_Cloud(input.get_Cloud());
}

void StatOutlierRemoval::setOutputName(std::string name)
{
    m_output->set_name(name);
}

void StatOutlierRemoval::execute()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudNewTerrain(new pcl::PointCloud<pcl::PointXYZI>);
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(m_baseCloud->get_Cloud());
    sor.setMeanK(m_neighbors);
    sor.setStddevMulThresh(m_mDist);
    sor.filter(*cloudNewTerrain);
    cloudNewTerrain->width = cloudNewTerrain->points.size();
    cloudNewTerrain->height = 1;
    cloudNewTerrain->is_dense = true;

    m_output->set_Cloud(cloudNewTerrain);
}