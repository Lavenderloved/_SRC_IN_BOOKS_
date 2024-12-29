#include "segmentation.h"
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <execution>
#include <mutex>

#include <chrono>
#include <pcl/visualization/pcl_visualizer.h>

Segmentation::Segmentation()
    : m_OctreeVoxel(m_resolution), m_OctreeVegetation(m_resolution), m_OctreeTerrain(m_resolution) {
  m_vegetation = new Cloud();
  m_terrain    = new Cloud();
  // m_centroids = new Cloud();
  m_restCloud = new Cloud();

  // recursive_count=1;
  m_resolution = 0.05;
  // neighborsize = 20;
  //    m_minimal_pnt = 10;
  m_treePrefix = "ID";
  percent      = 0;
  // m_centroids->set_name("centroids");
}

Segmentation::Segmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr vegetation,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr terrain)
    : m_OctreeVoxel(m_resolution), m_OctreeVegetation(m_resolution), m_OctreeTerrain(m_resolution) {
  m_vegetation = new Cloud();
  m_vegetation->set_Cloud(vegetation);
  m_terrain = new Cloud();
  m_terrain->set_Cloud(terrain);
  // m_centroids = new Cloud();
  m_restCloud  = new Cloud();
  percent      = 0;
  m_resolution = 0.05;
  m_treePrefix = "ID";
}

Segmentation::~Segmentation() {
  delete m_vegetation;
  delete m_terrain;
  delete m_restCloud;
  // delete m_centroids;
}

void Segmentation::setVegetation(Cloud input) {
  *m_vegetation = input;
}

void Segmentation::setTerrain(Cloud input) {
  *m_terrain = input;
}

void Segmentation::high_density_extract(float R_s = 0.2, float R_l = 2, float ratio_threshold = 0.035) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr z0_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  *z0_cloud_ptr = *m_vegetation->get_Cloud();

  size_t pointcloud_size = z0_cloud_ptr->points.size();

  std::for_each(std::execution::par_unseq, z0_cloud_ptr->points.begin(), z0_cloud_ptr->points.end(),
                [](pcl::PointXYZI &point) { point.z = 0; });

  pcl::search::KdTree<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(z0_cloud_ptr);

  std::cout << "\033[1;34mRELATIVE_DENSITY_FILTERING: \033[0m" << pointcloud_size << "\033[1;34m POINTS.\033[0m" << std::endl;
  auto start_time = std::chrono::high_resolution_clock::now();
  // std::vector<int> pt_IdxRS_Small;
  // std::vector<float> pt_SquaredDistance_Small;
  // std::vector<int> pt_IdxRS_Large;
  // std::vector<float> pt_SquaredDistance_Large;
  // unsigned long num_small = 0;
  // unsigned long num_large = 1; // ratio must be smaller than 1
  // pcl::PointIndices::Ptr output_indices(new pcl::PointIndices);
  // std::cout << "\033[1;32mRELATIVE_DENSITY_FILTERING: \033[0m" << pointcloud_size << "\033[1;32m points.\033[0m" << std::endl;
  // float sum = 0;
  // for (size_t index = 0; index < pointcloud_size; index++)
  // {
  //     std::cout << "index : " << index + 1 << " in " << pointcloud_size << " points." << std::endl;
  //
  //     num_small = kdtree.radiusSearch(z0_cloud_ptr->points[index], R_s, pt_IdxRS_Small, pt_SquaredDistance_Small);
  //     num_large = kdtree.radiusSearch(z0_cloud_ptr->points[index], R_l, pt_IdxRS_Large, pt_SquaredDistance_Large);
  //
  //     sum = sum + ratio_threshold;
  //     if (num_large > 0 && ratio_threshold < float(num_small) / float(num_large))
  //     {
  //         output_indices->indices.push_back(index);
  //     }
  // }
  // std::cout << "\n\nthe average ratio is " << sum / pointcloud_size << std::endl;
  // std::cout << output_indices->indices.size() << " points filtered out from origin point cloud." << std::endl;
  // // pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // pcl::ExtractIndices <pcl::PointXYZI> extract;
  // extract.setInputCloud(m_vegetation->get_Cloud());
  // extract.setIndices(output_indices);
  // extract.setNegative(false);
  // extract.filter(*m_vegetation->get_Cloud());

  // auto TEMP_VEGE = m_vegetation->get_Cloud();
  // std::vector<bool> valid_points(pointcloud_size, false); // 标记每个点是否有效
// #pragma omp parallel
//   {
//     pcl::search::KdTree<pcl::PointXYZI> kdtree_acc_local;
//     kdtree_acc_local.setInputCloud(z0_cloud_ptr);

//     std::vector<int> pt_IdxRS_Small;
//     std::vector<float> pt_SquaredDistance_Small;
//     std::vector<int> pt_IdxRS_Large;
//     std::vector<float> pt_SquaredDistance_Large;
//     unsigned int num_s;
//     unsigned int num_l;
// #pragma omp for
//     for (size_t idx = 0; idx < pointcloud_size; idx++) {
//       auto &point = TEMP_VEGE->points[idx];

//       pcl::PointXYZI search_point(point.x, point.y, 0, 0);

//       num_s = kdtree_acc_local.radiusSearch(search_point, R_s, pt_IdxRS_Small, pt_SquaredDistance_Small);
//       num_l = kdtree_acc_local.radiusSearch(search_point, R_l, pt_IdxRS_Large, pt_SquaredDistance_Large);

//       float ratio = float(num_s) / float(num_l);
//       if (num_l > 0 && ratio_threshold < ratio) {
//         valid_points[idx] = true;
//       }
//     }
//   }

  std::vector<bool> valid_points(pointcloud_size, false);                   // 标记每个点是否有效
  auto TEMP_VEGE = m_vegetation->get_Cloud();

  pcl::search::KdTree<pcl::PointXYZI> kdtree_acc;
  kdtree_acc.setInputCloud(z0_cloud_ptr);
  std::for_each(std::execution::par_unseq, TEMP_VEGE->points.begin(), TEMP_VEGE->points.end(),
                [R_s, R_l, ratio_threshold, &kdtree_acc, &valid_points, &TEMP_VEGE](const pcl::PointXYZI &point) {
                  size_t idx = &point - &TEMP_VEGE->points[0]; // 获取当前点的索引

                  pcl::PointXYZI search_point(point.x, point.y, 0, 0);
                  std::vector<int> pt_IdxRS_Small;
                  std::vector<float> pt_SquaredDistance_Small;
                  std::vector<int> pt_IdxRS_Large;
                  std::vector<float> pt_SquaredDistance_Large;
                  unsigned long num_s = kdtree_acc.radiusSearch(search_point, R_s, pt_IdxRS_Small, pt_SquaredDistance_Small);
                  unsigned long num_l = kdtree_acc.radiusSearch(search_point, R_l, pt_IdxRS_Large, pt_SquaredDistance_Large);

                  float ratio = float(num_s) / float(num_l);
                  if (num_l > 0 && ratio_threshold < ratio) {
                    valid_points[idx] = true;
                  }
                });

  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>); // 保存truth点，后续替换原始点云
  for (size_t i = 0; i < pointcloud_size; i++) {
    if (valid_points[i]) {
      temp_cloud->points.push_back(TEMP_VEGE->points[i]);
    }
  }
  std::swap(*temp_cloud, *m_vegetation->get_Cloud()); // 替换原始点云

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  float delta   = float(duration.count()) / 1000;
  std::cout << "\033[1;34mRELATIVE_DENSITY_FILTERING_DONE with \033[0m" << delta << "\033[1;34m SECONDES.\033[0m" << std::endl;

  // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
  // pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZI> color(m_vegetation->get_Cloud());
  // viewer->addPointCloud<pcl::PointXYZI>(m_vegetation->get_Cloud(), color, "vegetation");
  // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0, 0, "vegetation");
  // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "vegetation");
  // viewer->spin();

  // std::cout << output_indices->indices.size() << " points filtered out from origin point cloud." << std::endl;
  // // pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // pcl::ExtractIndices <pcl::PointXYZI> extract;
  // extract.setInputCloud(m_vegetation->get_Cloud());
  // extract.setIndices(output_indices);
  // extract.setNegative(false);
  // extract.filter(*m_vegetation->get_Cloud());

  // SOR 去噪，更新提取好的点云
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  // sor.setInputCloud(m_vegetation->get_Cloud());
  // sor.setNegative(false);
  // sor.setMeanK(50);
  // sor.setStddevMulThresh(1.0);
  // sor.filter(*m_vegetation->get_Cloud());

  // viewer->removeAllPointClouds();
  // viewer->addPointCloud<pcl::PointXYZI>(m_vegetation->get_Cloud(), color, "vegetation_2");
  // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1.0, 0, "vegetation_2");
  // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "vegetation_2");
  // viewer->spin();
}

void Segmentation::setResolution(float resolution) {
  m_resolution = resolution;
}

void Segmentation::setDistance(float resolution) {
  m_resolution = resolution;
}

void Segmentation::setMinimalPoint(int minimal_point) {
  m_minimal_pnt = minimal_point;
}

void Segmentation::setRestCloudName(std::string name) {
  m_restCloud->set_name(name);
}

void Segmentation::setTreePrefix(std::string prefix) {
  m_treePrefix = prefix;
}

void Segmentation::setTerrainDistance(float distanceFromTerrain) {
  m_terrainDistance = distanceFromTerrain;
}

void Segmentation::setNumOfElements(int elements) {
  m_numOfElements = elements;
}

void Segmentation::setRange(float value) {
  m_range = value;
}

void Segmentation::setMultiplicator(float multiplicator) {
  m_multiplicator = multiplicator;
}

void Segmentation::setMethod(int method) {
  // 1 for SFFI
  // 2 for instensity
  // 3 for slope
  // 4 for PCA * slope
  m_method = method;
}
/////////////////////////

void Segmentation::setOctreeVoxel() {
  m_OctreeVoxel.setResolution(m_resolution);
  m_OctreeVoxel.setInputCloud(m_voxels->get_Cloud());
  m_OctreeVoxel.addPointsFromInputCloud();
}

void Segmentation::setOctreeVegetation() {
  m_OctreeVegetation.setResolution(m_resolution);
  m_OctreeVegetation.setInputCloud(m_vegetation->get_Cloud());
  m_OctreeVegetation.addPointsFromInputCloud();
}

void Segmentation::setOctreeTerrain() {
  m_OctreeTerrain.setResolution(m_resolution);
  m_OctreeTerrain.setInputCloud(m_terrain->get_Cloud());
  m_OctreeTerrain.addPointsFromInputCloud();
}

void Segmentation::execute() {
  // float radius = std::sqrt(m_resolution * m_resolution + m_resolution * m_resolution + m_resolution * m_resolution) + 0.01;
  // createOctree();
  // computeDescriptor();
  // computeRangePCAValue();
  // mergeClusters();
  // createComponents();
  // findNeighborElements(radius);
  // findStructuralVoxels(radius);
  // findNeighborElements(radius);
  // setTreeElements();
  // std::cout << "tree number: " << getNumTrees() << "\n";
  //
  // float stopCondition = radius * m_multiplicator;
  // // std::cout<< "velikost stopCondition: " << stopCondition << "\n";
  //
  // for (float q = radius; q <= stopCondition; q += radius)
  // {
  //     int i = 0, j = 0;
  //     //        std::cout << "i: ";
  //     while (findStructuralVoxelsTrees(q) == true)
  //     {
  //         i++;
  //         if (i > 15)
  //             break;
  //     }
  //     std::cout << i;
  //     findAnyVoxelNeighborsTrees(radius);
  //     while (findStructuralVoxelsTrees(q) == true)
  //     {
  //         j++;
  //         if (j > 15)
  //             break;
  //     }
  //     //        std::cout << " j: " << j << "\n";
  // }
  // findAnyVoxelNeighborsTrees(radius + radius); // this may be too much
  // findStructuralVoxelsTrees(radius);
  // getNumUsedVoxels();
  // cleanTreeBase();
  // reconstructElements();
  // reconstructVegetationRest();

  high_density_extract(0.2, 2, 0.035);

  pcl::search::KdTree<pcl::PointXYZI>::Ptr Kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
  Kdtree->setInputCloud(m_vegetation->get_Cloud());

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(0.10); // cm
  ec.setMinClusterSize(200);
  ec.setMaxClusterSize(50000);
  ec.setSearchMethod(Kdtree);
  ec.setInputCloud(m_vegetation->get_Cloud());
  // 聚类抽取结果保存在一个数组中，数组中每个元素代表抽取的一个组件点云的下标
  ec.extract(cluster_indices);
  std::cout << "The size of cluster_indices is: " << cluster_indices.size() << std::endl;

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(m_vegetation->get_Cloud());
  extract.setNegative(false);
  for (const auto &cluster_id : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointIndices::Ptr temp_indices(new pcl::PointIndices);
    *temp_indices = cluster_id;
    extract.setIndices(temp_indices);
    extract.filter(*temp_cloud_cluster);
    m_stems.push_back(temp_cloud_cluster);
  }
  std::cout << "The size of EC_TREE_SET is: " << m_stems.size() << std::endl;
}

void Segmentation::createOctree() {
  //    std::cout << "SegmentationOctree::createOctree() \n";
  //    std::time_t result = std::time(nullptr);
  //    std::cout << std::asctime(std::localtime(&result));

  pcl::octree::OctreePointCloud<pcl::PointXYZI> oc(m_resolution);
  oc.setInputCloud(m_vegetation->get_Cloud());
  oc.addPointsFromInputCloud();
  // zjistit vsechny voxely
  std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> voxels;
  oc.getOccupiedVoxelCenters(voxels);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxels(new pcl::PointCloud<pcl::PointXYZI>);

  cloud_voxels->points.resize(voxels.size());
  m_clusters.resize(voxels.size());

  // pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> ocs (m_resolution);
  // ocs.setInputCloud (m_vegetation->get_Cloud());
  // ocs.addPointsFromInputCloud ();
  setOctreeVegetation();

#pragma omp parallel for
  for (int r = 0; r < voxels.size(); r++) {
    std::vector<int> indices;
    cloud_voxels->points.at(r) = voxels.at(r);

    if (m_OctreeVegetation.voxelSearch(voxels.at(r), indices) > 0)
      m_clusters.at(r) = indices;
  }
  cloud_voxels->width    = cloud_voxels->points.size();
  cloud_voxels->height   = 1;
  cloud_voxels->is_dense = true;

  m_voxels.reset(new Cloud(cloud_voxels, "voxely"));
  setOctreeVoxel();
  setOctreeTerrain();
  // setOctreeVoxel();
}

void Segmentation::computeDescriptor() {
  //    std::cout << "Segmentation::ccomputeDescriptor()\n";
  //    std::time_t result = std::time(nullptr);
  //    std::cout << std::asctime(std::localtime(&result));

#pragma omp parallel for
  for (int q = 0; q < m_voxels->get_Cloud()->points.size(); q++) {
    std::vector<int> indices;
    std::vector<float> dist;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (m_OctreeVegetation.radiusSearch(m_voxels->get_Cloud()->points.at(q), m_resolution, indices, dist) > 0) {
      for (int w = 0; w < indices.size(); w++) {
        cloud->points.push_back(m_vegetation->get_Cloud()->points.at(indices.at(w)));
      }
      if (m_method == 1) // PCA
      {
        float x                                       = computePCA(cloud);
        m_voxels->get_Cloud()->points.at(q).intensity = x;
      } else if (m_method == 2) // Intensity
      {
        float x                                       = computeIntensity(cloud, m_voxels->get_Cloud()->points.at(q));
        m_voxels->get_Cloud()->points.at(q).intensity = x;
      } else if (m_method == 3) // Slope
      {
        float x                                       = computeSlope(cloud, m_voxels->get_Cloud()->points.at(q));
        m_voxels->get_Cloud()->points.at(q).intensity = x;
      } else // PCA * slope
      {
        float x                                       = computePCA(cloud);
        float y                                       = computeSlope(cloud, m_voxels->get_Cloud()->points.at(q));
        m_voxels->get_Cloud()->points.at(q).intensity = x * y;
      }
    } else {
      m_voxels->get_Cloud()->points.at(q).intensity = 0;
    }
  }
}

void Segmentation::computeRangePCAValue() {
  //    std::cout << "SegmentationOctree::ComputeRangePCAValue() \n";
  //    std::time_t result = std::time(nullptr);
  //    std::cout << std::asctime(std::localtime(&result));
  float intensityMin = 10000000000000000.0;
  float intensityMax = -10000000000000000.0;
  for (int q = 0; q < m_voxels->get_Cloud()->points.size(); q++) {
    if (m_voxels->get_Cloud()->points.at(q).intensity < intensityMin) {
      intensityMin = m_voxels->get_Cloud()->points.at(q).intensity;
    }
    if (m_voxels->get_Cloud()->points.at(q).intensity > intensityMax) {
      intensityMax = m_voxels->get_Cloud()->points.at(q).intensity;
    }
  }
  std::cout << "intensityMin: " << intensityMin << "\n";
  std::cout << "intensityMax: " << intensityMax << "\n";
  float range = intensityMax - intensityMin;
  m_PCAValue  = intensityMin + range * m_range;
  std::cout << "m_PCAValue: " << m_PCAValue << "\n";
}

void Segmentation::mergeClusters() {
  //    std::cout << "SegmentationOctree::mergeClusters() \n";
  //    std::time_t result = std::time(nullptr);
  //    std::cout << std::asctime(std::localtime(&result));
  float radius =
      std::sqrt(m_resolution * m_resolution + m_resolution * m_resolution + m_resolution * m_resolution) + 0.01;

  // for each cluster find close clusters and if its point minimal distance is up to m_resoltion, join into one segment
  std::vector<int> comp;
  m_components.resize(m_clusters.size(), comp);
  // pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> ocs (m_resolution);
  // ocs.setInputCloud (m_voxels->get_Cloud());
  // ocs.addPointsFromInputCloud ();

#pragma omp parallel for
  for (int q = 0; q < m_clusters.size(); q++) {
    if (m_voxels->get_Cloud()->points.at(q).intensity < m_PCAValue)
      continue;

    // find 10 closest voxels
    std::vector<int> pointIDv;
    std::vector<float> pointSDv;

    if (m_OctreeVoxel.radiusSearch(m_voxels->get_Cloud()->points.at(q), radius, pointIDv, pointSDv) >
        0) // find neighboring voxels
    {
#pragma omp parallel for
      for (int w = 0; w < pointIDv.size(); w++) {
        if (q == pointIDv.at(w)) // finds the same point
          continue;
        if (m_voxels->get_Cloud()->points.at(pointIDv.at(w)).intensity <
            m_PCAValue) // pcaValue is less than limit
          continue;
#pragma omp critical
        m_components.at(q).push_back(pointIDv.at(w));
        m_components.at(pointIDv.at(w)).push_back(q);
      }
    }
  }
  for (int d = 0; d < m_components.size(); d++)
    sortVector(m_components.at(d));
}

void Segmentation::createComponents() {
  //    std::cout << "SegmentationOctree::createComponents() \n";
  //    std::time_t result = std::time(nullptr);
  //    std::cout << std::asctime(std::localtime(&result));
  // for each component find all close voxels and merge them into single vector - element
  std::vector<bool> usedComp(m_components.size(), false);
#pragma omp parallel for
  for (int e = 0; e < m_components.size(); e++) {
    // std::cout<<"pocet sousedu: " << m_components.at(e).size()<< " componenta: " << e << "\n";
    if (usedComp.at(e) == true)
      continue;
    std::vector<int> element;
    // set first element
    element.push_back(e);
    usedComp.at(e) = true;

    for (int t = 0; t < element.size(); t++) {
      usedComp.at(element.at(t)) = true;
      // set neighbor
      for (int r = 0; r < m_components.at(element.at(t)).size(); r++) {
        int voxelID = m_components.at(element.at(t)).at(r);
        if (usedComp.at(voxelID) == false) {
          element.push_back(voxelID);
          usedComp.at(voxelID) = true;
        }
      }
    }
    if (element.size() > 5) {
#pragma omp critical
      m_elements.push_back(element);
    }
  }
  setUsedVoxels();
}

bool Segmentation::findNeighborElements(float radius) {
  bool find = false;
  //    std::cout << "SegmentationOctree::findNeighborElements() \n";
  //    std::time_t result = std::time(nullptr);
  //    std::cout << std::asctime(std::localtime(&result));

  std::vector<std::vector<int>> connection(m_elements.size());

#pragma omp parallel for
  for (int q = 0; q < m_voxels->get_Cloud()->points.size(); q++) {
    // pokud  je pouzity tak  continue, jinak zjistit jestli v jeho okoli jsou jiné elementy
    if (m_usedVoxels.at(q) == -1)
      continue;
    // find 10 closest voxels
    std::vector<int> pointIDv;
    std::vector<float> pointSDv;
    std::vector<int> neighbor;
    int elementID = m_usedVoxels.at(q);

    if (m_OctreeVoxel.radiusSearch(m_voxels->get_Cloud()->points.at(q), radius, pointIDv, pointSDv) >
        0) // find neighboring voxels
    {
      for (int w = 0; w < pointIDv.size(); w++) {
        int elementID2 = m_usedVoxels.at(pointIDv.at(w));
        if (elementID2 == -1 || elementID == elementID2) // finds the same voxel or voxel is not in element
          continue;
          // check distance of clouds
          // if( cloudMinDistance(q, pointIDv.at(w) ) <= m_resolution*4) // points inside voxel are up to distance
          //{n
          // std::cout<< "elementID: " << elementID << " elementID2: "<< elementID2 << "\n";
#pragma omp critical
        connection.at(elementID).push_back(elementID2);
        connection.at(elementID2).push_back(elementID);
        find = true;
        //}
      }
    }
  }
  mergeElements(connection);
  return find;
}

bool Segmentation::findStructuralVoxels(float radius) {
  //    std::cout << "SegmentationOctree::findStructuralVoxels() \n";
  //    std::time_t result = std::time(nullptr);
  //    std::cout << std::asctime(std::localtime(&result));

  bool find = false;
  // for each cluster find close clusters and if its point minimal distance is up to m_resoltion, join into one segment
  // std::vector<std::vector<int>> connection(m_components.size());
#pragma omp parallel for
  for (int q = 0; q < m_voxels->get_Cloud()->points.size(); q++) {
    if (m_voxels->get_Cloud()->points.at(q).intensity < m_PCAValue || m_usedVoxels.at(q) == -1)
      continue;

    std::vector<int> pointIDv;
    std::vector<float> pointSDv;
    std::vector<int> neighbor;
    int elementID = m_usedVoxels.at(q);
    if (m_OctreeVoxel.radiusSearch(m_voxels->get_Cloud()->points.at(q), radius, pointIDv, pointSDv) >
        0) // find neighboring voxels
    {
      for (int w = 0; w < pointIDv.size(); w++) {
        if (q == pointIDv.at(w) || m_voxels->get_Cloud()->points.at(pointIDv.at(w)).intensity < m_PCAValue ||
            m_usedVoxels.at(pointIDv.at(w)) != -1)
          continue;
        m_elements.at(elementID).push_back(pointIDv.at(w));
        m_usedVoxels.at(pointIDv.at(w)) = elementID;
        find                            = true;
      }
    }
  }
  // setUsedVoxels();
  return find;
}

void Segmentation::setTreeElements() {
  //    std::cout << "SegmentationOctree::setTreeElements() \n";
  //    std::time_t result = std::time(nullptr);
  //    std::cout << std::asctime(std::localtime(&result));

  // for each element set if  number of voxels and distnace from terrain
#pragma omp parallel for
  for (int q = 0; q < m_elements.size(); q++) {
    bool used = false;
    if (m_elements.at(q).size() < m_numOfElements)
      continue;
    for (int w = 0; w < m_elements.at(q).size(); w++) {
      if (used == true)
        continue;
      std::vector<int> pointIDv;
      std::vector<float> pointSDv;
      if (m_OctreeTerrain.radiusSearch(m_voxels->get_Cloud()->points.at(m_elements.at(q).at(w)),
                                       m_terrainDistance, pointIDv, pointSDv) > 0) // find neighboring voxels
      {
        // save element into trees
#pragma omp critical
        m_trees.push_back(m_elements.at(q));
        used = true;
      }
    }
  }
  setUsedVoxelsInTrees();
}

bool Segmentation::findStructuralVoxelsTrees(float radius) {
  //     std::cout<< "SegmentationOctree::findStructuralVoxelsTrees() \n";
  // std::time_t result = std::time(nullptr);
  // std::cout << std::asctime(std::localtime(&result));

  bool find = false;
  // float radius = std::sqrt(m_resolution*m_resolution + m_resolution*m_resolution + m_resolution*m_resolution  ) + 0.01;

  // for each cluster find close clusters and if its point minimal distance is up to m_resoltion, join into one segment
  std::vector<std::vector<int>> connection(m_components.size());
#pragma omp parallel for
  for (int q = 0; q < m_voxels->get_Cloud()->points.size(); q++) {
    if (m_voxels->get_Cloud()->points.at(q).intensity < m_PCAValue || m_usedVoxelsTrees.at(q) == -1)
      continue;

    std::vector<int> pointIDv;
    std::vector<float> pointSDv;
    int treeID = m_usedVoxelsTrees.at(q);

    if (m_OctreeVoxel.radiusSearch(m_voxels->get_Cloud()->points.at(q), radius, pointIDv, pointSDv) >
        0) // find neighboring voxels
    {
      for (int w = 0; w < pointIDv.size(); w++) {
        if (q == pointIDv.at(w) || m_voxels->get_Cloud()->points.at(pointIDv.at(w)).intensity < m_PCAValue ||
            m_usedVoxelsTrees.at(pointIDv.at(w)) != -1) // finds the same point or point in element
          continue;
        m_trees.at(treeID).push_back(pointIDv.at(w));
        m_usedVoxelsTrees.at(pointIDv.at(w)) = treeID;
        // find = true;
        //  if found voxel is part of element - add whole element, but check if the leemnt is not tree
        if (m_usedVoxels.at(pointIDv.at(w)) != -1) {
          for (int a = 0; a < m_elements.at(m_usedVoxels.at(pointIDv.at(w))).size(); a++) {
            int vox = m_elements.at(m_usedVoxels.at(pointIDv.at(w))).at(a);
            if (m_usedVoxelsTrees.at(vox) == -1) {
              m_trees.at(treeID).push_back(vox);
              m_usedVoxelsTrees.at(vox) = treeID;
              find                      = true;
            }
          }
        }
      }
    }
  }
  return find;
}

int Segmentation::findAnyVoxelNeighborsTrees(float radius) {
  // bool find=false;
  int numb = 0;
  //      std::cout<< "SegmentationOctree::computeVoxelNeighborsTrees() \n";
  //    std::time_t result = std::time(nullptr);
  //   std::cout << std::asctime(std::localtime(&result));

  // for each voxel find if its neighbor is in element or more elements.
  // if voxel has only one element in neighbor add it to the element.
  // if it has more elements  save both elements ID into connections. for later merge
  // pro kazdy voxel
#pragma omp parallel for
  for (int q = 0; q < m_voxels->get_Cloud()->points.size(); q++) {
    if (m_usedVoxelsTrees.at(q) > -1)
      continue;
    std::vector<int> pointIDv;
    std::vector<float> pointSDv;
    std::vector<int> neighbor;

    if (m_OctreeVoxel.radiusSearch(m_voxels->get_Cloud()->points.at(q), radius, pointIDv, pointSDv) > 1) {
      for (int w = 0; w < pointIDv.size(); w++) {
        if (pointIDv.at(w) == q || m_usedVoxelsTrees.at(pointIDv.at(w)) == -1)
          continue;
        int treeID = m_usedVoxelsTrees.at(pointIDv.at(w));
        neighbor.push_back(treeID);
      }
      sortVector(neighbor);
      if (neighbor.size() == 1) // if only one tree add voxel to the tree
      {
        int treeID              = neighbor.at(0);
        m_usedVoxelsTrees.at(q) = treeID;
#pragma omp critical
        m_trees.at(treeID).push_back(q);
        numb++;
      }
    }
  }
  return numb;
}

float Segmentation::getNumUsedVoxels() {
  int a = 0;
#pragma omp parallel for reduction(+ : a)
  for (int q = 0; q < m_elements.size(); q++) {
    a += m_elements.at(q).size();
  }
  std::cout << a << " of " << m_usedVoxels.size() << " voxels used\n";
  return a;
}

void Segmentation::reconstructElements() {
  for (int a = 0; a < m_trees.size(); a++) {
    reconstructElement(m_trees.at(a));
  }
}

void Segmentation::reconstructVegetationRest() {
  std::vector<int> indices;
  for (int i = 0; i < m_trees.size(); i++) {
    std::vector<int> element = m_trees.at(i);
    for (int a = 0; a < element.size(); a++)
      indices.insert(indices.end(), m_clusters.at(element.at(a)).begin(), m_clusters.at(element.at(a)).end());
  }

  std::shared_ptr<std::vector<int>> indicesptr(new std::vector<int>(indices));
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(m_vegetation->get_Cloud());
  extract.setIndices(indicesptr);
  extract.setNegative(true);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  extract.filter(*cloud);

  cloud->width    = cloud->points.size();
  cloud->height   = 1;
  cloud->is_dense = true;

  // std::string m_treePrefix="vegetation_rest";
  // std::string name = m_restCloud->get_name().toStdString();

  // std::shared_ptr<Tree> stemptr (new Tree(cloud, name));
  m_restCloud->set_Cloud(cloud);
}

void Segmentation::reconstructElement(std::vector<int> element) {
  // create indices for whole elenent
  std::vector<int> indices;
  for (int a = 0; a < element.size(); a++)
    indices.insert(indices.end(), m_clusters.at(element.at(a)).begin(), m_clusters.at(element.at(a)).end());

  if (m_vegetation->get_Cloud()->points.size() < indices.size() || indices.size() == 0)
    return;

  std::shared_ptr<std::vector<int>> indicesptr(new std::vector<int>(indices));
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(m_vegetation->get_Cloud());
  extract.setIndices(indicesptr);
  extract.setNegative(false);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  extract.filter(*cloud);

  cloud->width    = cloud->points.size();
  cloud->height   = 1;
  cloud->is_dense = true;

  // std::string m_treePrefix="ID";
  // std::string name = m_treePrefix + std::to_string(m_stems.size());

  // std::shared_ptr<Tree> stemptr (new Tree(cloud, name));
  m_stems.push_back(cloud);
}

float Segmentation::computePCA(pcl::PointCloud<pcl::PointXYZI>::Ptr input) {
  if (input->points.size() < 3)
    return 0;

  pcl::PointCloud<pcl::PointXYZI> cloud_;
  pcl::PCA<pcl::PointXYZI> pca;
  pca.setInputCloud(input);
  pca.project(*input, cloud_);

  pcl::PointXYZI proj_min, proj_max;
  pcl::getMinMax3D(cloud_, proj_min, proj_max);

  float eL = std::abs(proj_max.x - proj_min.x);
  float eI = std::abs(proj_max.y - proj_min.y);
  float eS = std::abs(proj_max.z - proj_min.z);
  // sorting
  if (eI < eS) {
    float p = eS;
    eS      = eI;
    eI      = p;
  }
  if (eL < eI) {
    float p = eL;
    eL      = eI;
    eI      = p;
  }
  if (eI < eS) {
    float p = eS;
    eS      = eI;
    eI      = p;
  }
  // compute index
  float SFFIx = 100 - (eL * 100 / (eL + eI + eS));
  // float SFFIy = eS/eL;

  return SFFIx;
}

float Segmentation::computeIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointXYZI a) {
  float sklon = 0;
  for (int w = 0; w < input->points.size(); w++) {

    sklon += input->points.at(w).intensity;
  }
  float skl = sklon / input->points.size();
  return skl;
}

float Segmentation::computeSlope(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointXYZI a) {
  if (input->points.size() < 2)
    return 0;
  float slope = 0;
  for (int q = 0; q < input->points.size(); q++) {
    float sklon      = 0;
    pcl::PointXYZI p = input->points.at(q);
    for (int w = 0; w < input->points.size(); w++) {
      if (q == w)
        continue;
      pcl::PointXYZI b = input->points.at(w);
      float dist       = std::sqrt((p.x - b.x) * (p.x - b.x) + (p.y - b.y) * (p.y - b.y) + (p.z - b.z) * (p.z - b.z));
      if (dist == 0)
        dist = 0.000000000000001;
      float x = std::abs(p.z - b.z) / dist;
      sklon += x;
    }
    slope += (sklon * 100 / input->points.size());
  }
  return slope / input->points.size();
}

void Segmentation::cleanTreeBase() {
  // pro kazdy strom
  for (int i = 0; i < m_trees.size(); i++) // pracuju s voxely
  {
    // zjistit nejspodnějši voxel
    int lowVoxel;
    float lowZ = 100000000000000;
    for (int q = 0; q < m_trees.at(i).size(); q++) {
      if (m_voxels->get_Cloud()->points.at(m_trees.at(i).at(q)).z < lowZ) {
        lowZ     = m_voxels->get_Cloud()->points.at(m_trees.at(i).at(q)).z;
        lowVoxel = m_trees.at(i).at(q);
      }
    }

    // select all voxels  up to m_terrainDistance
    std::vector<int> wrongVoxels;
    std::vector<int> goodVoxels;
    for (int q = 0; q < m_trees.at(i).size(); q++) {
      if (m_voxels->get_Cloud()->points.at(m_trees.at(i).at(q)).z < lowZ + m_terrainDistance) {
        // create cloud with point inside voxel
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        std::vector<int> insiders;
        m_OctreeVegetation.voxelSearch(m_voxels->get_Cloud()->points.at(m_trees.at(i).at(q)), insiders);
        for (int r = 0; r < insiders.size(); r++)
          cloud->points.push_back(m_vegetation->get_Cloud()->points.at(insiders.at(r)));
        // pro tyto voxely spocitat sklon
        float x = computeSlope(cloud, m_voxels->get_Cloud()->points.at(m_trees.at(i).at(q)));
        if (x < 45)
          wrongVoxels.push_back(m_trees.at(i).at(q));
        // if(m_voxels->get_Cloud()->points.at(m_trees.at(i).at(q)).intensity < m_PCAValue)
        // wrongVoxels.push_back(m_trees.at(i).at(q));

        else
          goodVoxels.push_back(m_trees.at(i).at(q));
      } else {
        goodVoxels.push_back(m_trees.at(i).at(q));
      }
    }
    m_trees.at(i) = goodVoxels;
  }
}

void Segmentation::getData() {
  std::cout << "rest odeslany" << std::endl;
  for (int i = 0; i < m_stems.size(); i++) {
    //    QString x = QString(m_treePrefix);
    std::string name = m_treePrefix + std::to_string(i + 1);
    //        QString name = QString("%1%2").arg(QString::fromStdString(m_treePrefix)).arg(i);
    Cloud *c = new Cloud(m_stems.at(i), name);
  }
  // m_restCloud->set_Cloud(m_voxels->get_Cloud());
  // m_restCloud->set_name("voxels");
  // emit sendingCentr( m_restCloud);

  std::cout << "stromy odeslany" << std::endl;
}

void Segmentation::getRestcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr output) {
  *output = *m_restCloud->get_Cloud();
}

int Segmentation::get_treeSize() {
  return m_stems.size();
}

void Segmentation::getTree(int i, pcl::PointCloud<pcl::PointXYZI>::Ptr output) {
  if (i < m_stems.size())
    *output = *m_stems.at(i);
}

void Segmentation::setUsedVoxels() {
  m_usedVoxels.clear();
  m_usedVoxels.resize(m_voxels->get_Cloud()->points.size(), -1);
#pragma omp parallel for
  for (int q = 0; q < m_elements.size(); q++) {
    for (int w = 0; w < m_elements.at(q).size(); w++) {
      m_usedVoxels.at(m_elements.at(q).at(w)) = q;
    }
  }
}

void Segmentation::setUsedVoxelsInTrees() {
  m_usedVoxelsTrees.clear();
  m_usedVoxelsTrees.resize(m_voxels->get_Cloud()->points.size(), -1);

#pragma omp parallel for
  for (int q = 0; q < m_trees.size(); q++) {
    for (int w = 0; w < m_trees.at(q).size(); w++) {
      m_usedVoxelsTrees.at(m_trees.at(q).at(w)) = q;
    }
  }
}

void Segmentation::sortVector(std::vector<int> &vec) {
  if (vec.size() > 1) {
    std::sort(vec.begin(), vec.end());
    vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
  }
}

bool Segmentation::mergeElements(std::vector<std::vector<int>> &vec) {
  // input is vector that has size of elements and each subvector contains ID of connecting element
  std::vector<bool> usedEl(m_elements.size(), false);
  std::vector<std::vector<int>> newElCon(m_elements.size());
  // for each element find all connecting elements, even for its neighbor ( ie.e if element(1) should be connected to element(2) add also all connection sthat shoul be for element(2), if element is already connected do not connect it.
  for (int n = 0; n < vec.size(); n++)
    sortVector(vec.at(n));

  for (int s = 0; s < vec.size(); s++) {
    if (usedEl.at(s) == true)
      continue;
    std::vector<int> newEl = vec.at(s);
    usedEl.at(s)           = true;
    for (int d = 0; d < newEl.size(); d++) {
      if (usedEl.at(newEl.at(d)) == true)
        continue;
      newEl.insert(newEl.end(), vec.at(newEl.at(d)).begin(),
                   vec.at(newEl.at(d)).end()); // tady sa pridabvaji zpetne vazby
      usedEl.at(newEl.at(d)) = true;
    }
    sortVector(newEl); // mela by byt jina velikost
    newElCon.at(s) = newEl;
  }
  // std::cout << "veliksot newELcon: " << newElCon.size() <<" veliksot elemntu:  "<< m_elements.size() << "\n";

  // merge elements and save into m_elements
  // for new element assign all voxels that belong to this element and merge voxels from connecting element. Connecting element is not created.
  std::vector<std::vector<int>> new_m_elements;
  std::vector<bool> usedElem(m_elements.size(), false);
  for (int f = 0; f < newElCon.size(); f++) {
    if (usedElem.at(f) == true)
      continue;
    std::vector<int> newElement = m_elements.at(f);
    usedElem.at(f)              = true;
    for (int g = 0; g < newElCon.at(f).size(); g++) {
      int ElementID = newElCon.at(f).at(g);
      if (usedElem.at(ElementID) == true)
        continue;
      newElement.insert(newElement.end(), m_elements.at(ElementID).begin(), m_elements.at(ElementID).end());
      usedElem.at(ElementID) = true;
    }
    new_m_elements.push_back(newElement);
  }
  // std::cout<<"pocet stromu1: " << m_elements.size();

  m_elements.swap(new_m_elements);
  // std::cout<<" pocet stromu2: " << m_elements.size() << "\n";
  setUsedVoxels();
  return true;
}

int Segmentation::getNumTrees() {
  return m_trees.size();
}
