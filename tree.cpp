#include "tree.h"

#include "ComputeSortiment.h"
#include "HoughTransform.h"
#include "LeastSquareRegression.h"
#include "hull.h"
#include "qsm.h"
#include "skeleton.h"
#include "sortiment.h"
// #include <QtWidgets/QMessageBox>

#include <pcl/common/distances.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <execution>
// #include <pcl/sample_consensus/sac_model_circle2d.h>

#include <pcl/filters/conditional_removal.h>
// #include <pcl/ModelCoefficients.h>
#include <pcl/common/pca.h>

// Tree
Tree::Tree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string name)
        : Cloud(cloud, name)
{
    pcl::getMinMax3D(*get_Cloud(), m_minp, m_maxp);
    std::string a = m_name + "_DBH";
    //    std::string x = QString("%1_dbh").arg(m_name);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
    m_dbhCloud = new Cloud(cloud_, a);
    std::string aaaa = m_name + "_skeleton";
    //    std::string aaaa = QString("%1_skeleton").arg(m_name);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_4(new pcl::PointCloud<pcl::PointXYZI>);
    //    m_skeleton = new Cloud(cloud_4, aaaa);

    pcl::PointXYZI bod;
    bod.x = -1;
    bod.y = -1;
    bod.z = -1;

    m_pose = bod;
    m_dbh_RHT = {-1, -1, -1, -1, -0.5};
    m_dbh_LSR = {-1, -1, -1, -1, -0.5};
    m_dbh_QSM = {-1, -1, -1, -1, -0.5};
    m_height = -1;
    m_lenght = -1;
    m_convexhull = 0;
    m_concavehull = 0;
    //    m_crown = 0;
    m_triangulatedConcaveHull = new pcl::PolygonMesh;
}

Tree::Tree(Cloud cloud)
        : Cloud(cloud)
{
    pcl::getMinMax3D(*get_Cloud(), m_minp, m_maxp);
    std::string a = m_name + "_DBH";
    //    std::string x = QString("%1_dbh").arg(m_name);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
    m_dbhCloud = new Cloud(cloud_, a);
    std::string aaaa = m_name + "_skeleton";
    //    std::string aaaa = QString("%1_skeleton").arg(m_name);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_4(new pcl::PointCloud<pcl::PointXYZI>);
    //    m_skeleton = new Cloud(cloud_4, aaaa);

    pcl::PointXYZI bod;
    bod.x = -1;
    bod.y = -1;
    bod.z = -1;

    m_pose = bod;
    m_dbh_RHT = {-1, -1, -1, -1, -0.5};
    m_dbh_LSR = {-1, -1, -1, -1, -0.5};
    m_dbh_QSM = {-1, -1, -1, -1, -0.5};
    m_height = -1;
    m_lenght = -1;
    m_convexhull = 0;
    m_concavehull = 0;
    //    m_crown = 0;
    m_triangulatedConcaveHull = new pcl::PolygonMesh;
    set_length();
}

// Tree& Tree::operator=(Tree &copy)
// {
//     Tree t(copy);
//     t.m_Cloud = copy.m_Cloud;
//     t.m_dbhCloud = copy.m_dbhCloud;
//     t.m_name = copy.m_name;
//     t.m_pose = copy.m_pose;
//     t.m_dbh_RHT = copy.m_dbh_RHT;
//     t.m_dbh_LSR = copy.m_dbh_LSR;
//     t.m_height = copy.m_height;
//     t.m_lenght = copy.m_lenght;
//     t.m_areaconvex = copy.m_areaconvex;
//     t.m_areaconcave = copy.m_areaconcave;
//     t.m_minp = copy.m_minp;
//     t.m_maxp = copy.m_maxp;
//     t.m_lmax = copy.m_lmax;
//     t.m_lmin = copy.m_lmin;
//     t.m_stemCurvature = copy.m_stemCurvature;
//     t.m_convexhull = copy.m_convexhull;
//     t.m_concavehull = copy.m_concavehull;
//     t.m_triangulatedConcaveHull = copy.m_triangulatedConcaveHull;
//     //    t.m_crown = copy.m_crown;
//     return t;
// }

//定义一个 运算符重载 =
Tree &Tree::operator=(const Tree &copy)
{
    if (this == &copy)
    {
        return *this;
    }
    this->m_Cloud = copy.m_Cloud;
    this->m_dbhCloud = copy.m_dbhCloud;
    this->m_name = copy.m_name;
    this->m_pose = copy.m_pose;
    this->m_dbh_RHT = copy.m_dbh_RHT;
    this->m_dbh_LSR = copy.m_dbh_LSR;
    this->m_height = copy.m_height;
    this->m_lenght = copy.m_lenght;
    this->m_areaconvex = copy.m_areaconvex;
    this->m_areaconcave = copy.m_areaconcave;
    this->m_minp = copy.m_minp;
    this->m_maxp = copy.m_maxp;
    this->m_lmax = copy.m_lmax;
    this->m_lmin = copy.m_lmin;
    this->m_stemCurvature = copy.m_stemCurvature;
    this->m_convexhull = copy.m_convexhull;
    this->m_concavehull = copy.m_concavehull;
    this->m_triangulatedConcaveHull = copy.m_triangulatedConcaveHull;
    //    this->m_crown = copy.m_crown;
    this->m_trunk_axis = copy.m_trunk_axis;
    this->m_trunk_height = copy.m_trunk_height;
    this->m_trunk_base_point = copy.m_trunk_base_point;
    this->m_trunk_maxP = copy.m_trunk_maxP;
    this->m_trunk_minP = copy.m_trunk_minP;
    this->m_trunk_status = copy.m_trunk_status;
    this->m_DBH_Value = copy.m_DBH_Value;
    return *this;
}

void Tree::set_height() // check if tree is connected to terrain!!!
{
    if (m_pose.x == -1 && m_pose.y == -1 && m_pose.z == -1)
        m_height = -1;
    else
        m_height = m_maxp.z - m_pose.z;
}

void Tree::set_dbhCloud()
{
    // m_dbhCloud = new Cloud();
    // points 10 CM around 1.3 m
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_dbhCloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
    {
        if (ith->z > (m_pose.z + 1.25) && ith->z < (m_pose.z + 1.35))
        {
            pcl::PointXYZI bod;
            bod.x = ith->x;
            bod.y = ith->y;
            bod.z = ith->z;
            bod.intensity = ith->intensity;

            temp_dbhCloud->points.push_back(bod);
        }
    }

    if (temp_dbhCloud->points.size() > 50)
    {
        // voxelize
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_fil(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(temp_dbhCloud);
        sor.setLeafSize(0.001f, 0.001f, 0.001f);
        sor.filter(*cloud_fil);

        m_dbhCloud->set_Cloud(cloud_fil);
        cloud_fil.reset();
    } else
    {
        m_dbhCloud->set_Cloud(temp_dbhCloud);
    }

    temp_dbhCloud.reset();
}

void Tree::set_dbhCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    m_dbhCloud->set_Cloud(cloud);
}

void Tree::set_dbhRHT(int i)
{
    if (m_pose.x == -1 && m_pose.y == -1 && m_pose.z == -1)
        return;

    if (m_dbhCloud->get_Cloud()->points.size() > 2)
    {
        HoughTransform ht = m_dbhCloud->get_Cloud();
        ht.set_iterations(i); // TODO: 多层选择
        ht.compute();
        m_dbh_RHT = ht.get_circle();
    } else
    {
        m_dbh_RHT = {-1, -1, -1, -1, -0.5};
    }
}

stred Tree::get_dbhRHT()
{
    return m_dbh_RHT;
}

stred Tree::get_dbhLSR()
{
    return m_dbh_LSR;
}

void Tree::set_dbhLSR()
{
    if (m_pose.x == -1 && m_pose.y == -1 && m_pose.z == -1)
        return;

    if (m_dbhCloud->get_Cloud()->points.size() > 2)
    {
        LeastSquaredRegression lsr;
        lsr.setCloud(m_dbhCloud->get_Cloud());
        lsr.compute();
        // m_dbh_LSR = lsr.kamihoDBH();
        m_dbh_LSR = lsr.getCircle();
    } else
    {
        m_dbh_LSR = {1, -1, -1, -1, -0.5};
    }
}

void Tree::set_position(int height)
{
    float h = (float) height / 100;

    if (m_dbhCloud->get_Cloud()->points.size() > 0)
        m_dbhCloud->get_Cloud()->points.clear();
    std::vector<float> x_coor;
    std::vector<float> y_coor;

    for (auto ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
    {
        if (ith->z < (m_minp.z + h))
        {
            x_coor.push_back(ith->x);
            y_coor.push_back(ith->y);
        }
    }

    if (x_coor.size() > 1)
    {
        std::sort(x_coor.begin(), x_coor.end());
        std::sort(y_coor.begin(), y_coor.end());

        m_pose.x = x_coor.at(x_coor.size() / 2);
        m_pose.y = y_coor.at(y_coor.size() / 2);
        m_pose.z = m_minp.z;
        m_pose.intensity = 1;
    } else
    {
        m_pose = m_minp;
    }
    set_length();
    set_dbhCloud();

    if (m_dbh_RHT.x != -1 && m_dbh_RHT.y != -1 && m_dbh_RHT.r != -1)
        set_dbhRHT();
    if (m_dbh_LSR.x != -1 && m_dbh_LSR.y != -1 && m_dbh_LSR.r != -1)
        set_dbhLSR();
    if (m_height != -1)
        set_height();
    if (!m_stemCurvature.empty())
        set_stemCurvature();
}

void Tree::set_position(Cloud terrain, int num_points, int height)
{
    float h = (float) height / 100;
    if (m_dbhCloud->get_Cloud()->points.size() > 0)
        m_dbhCloud->get_Cloud()->points.clear();
    std::vector<float> x_coor;
    std::vector<float> y_coor;

    pcl::PointXYZI posit;
    posit = m_minp;
    for (auto ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
    {
        if (ith->z < (m_minp.z + h))
        {
            x_coor.push_back(ith->x);
            y_coor.push_back(ith->y);
        }
    }

    if (x_coor.size() > 1)
    {
        std::sort(x_coor.begin(), x_coor.end());
        std::sort(y_coor.begin(), y_coor.end());

        m_pose.x = x_coor.at(x_coor.size() / 2);
        m_pose.y = y_coor.at(y_coor.size() / 2);
        m_pose.z = m_minp.z;
        m_pose.intensity = 1;
    } else
    {
        m_pose = m_minp;
    }

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(terrain.get_Cloud());
    std::vector<int> pointId(num_points);
    std::vector<float> pointSD(num_points);
    if (kdtree.nearestKSearch(m_pose, num_points, pointId, pointSD) > 0)
    {
        float med_Z = 0;
        for (int i = 0; i < num_points; i++)
        {
            med_Z += terrain.get_Cloud()->points.at(pointId.at(i)).z;
        }
        m_pose.z = med_Z / num_points;
    }
    // set_length();
    set_dbhCloud();
    // if (m_dbh_RHT.x != -1 && m_dbh_RHT.y != -1 && m_dbh_RHT.r != -1)
    //     set_dbhRHT();
    // if (m_dbh_LSR.x != -1 && m_dbh_LSR.y != -1 && m_dbh_LSR.r != -1)
    //     set_dbhLSR();
    // if (m_height != -1)
    //     set_height();
    // if (!m_stemCurvature.empty())
    //     set_stemCurvature();
}

pcl::PointXYZI Tree::get_pose()
{
    return m_pose;
}

float Tree::get_height()
{
    if (m_height == -1)
        return m_height;
    float AA = ceilf(m_height * 100) / 100; // zaokrouhleni
    return AA;
}

void Tree::set_length()
{

    pcl::PointXYZI pmin, pmax;
    m_lmin.x = 9000000;
    m_lmin.y = 9000000;
    m_lmin.z = 9000000;
    m_lmax.x = -9000000;
    m_lmax.y = -9000000;
    m_lmax.z = -9000000;
    // najdi nejdelsi osu
    // X axis
    if (std::abs(m_maxp.x - m_minp.x) > std::abs(m_maxp.y - m_minp.y) &&
        std::abs(m_maxp.x - m_minp.x) > std::abs(m_maxp.z - m_minp.z))
    {

        for (int j = 0; j < m_Cloud->points.size(); j++)
        {

            // pokud je rozdil bodu x m_maxp mensi nez 1
            if (m_maxp.x - m_Cloud->points.at(j).x < 1 && m_Cloud->points.at(j).x > m_lmax.x)
            {
                m_lmax = m_Cloud->points.at(j);
            }

            if (m_Cloud->points.at(j).x - m_minp.x < 1 && m_Cloud->points.at(j).x < m_lmin.x)
            {
                m_lmin = m_Cloud->points.at(j);
            }
        }
    }
        // Y axis
    else if ((m_maxp.y - m_minp.y) > (m_maxp.x - m_minp.x) && (m_maxp.y - m_minp.y) > (m_maxp.z - m_minp.z))
    {
        for (int j = 0; j < m_Cloud->points.size(); j++)
        {

            // pokud je rozdil bodu x m_maxp mensi nez 10
            if (m_maxp.y - m_Cloud->points.at(j).y < 1 && m_Cloud->points.at(j).y > m_lmax.y)
            {
                m_lmax = m_Cloud->points.at(j);
            }

            if (m_Cloud->points.at(j).y - m_minp.y < 1 && m_Cloud->points.at(j).y < m_lmin.y)
            {
                m_lmin = m_Cloud->points.at(j);
            }
        }
    } else // Z axis
    {      // nejdelsi je osa z

        for (int j = 0; j < m_Cloud->points.size(); j++)
        {
            // pokud je rozdil bodu x m_maxp mensi nez 10
            if (m_maxp.z - m_Cloud->points.at(j).z < 1 && m_Cloud->points.at(j).z > m_lmax.z)
            {
                m_lmax = m_Cloud->points.at(j);
            }

            if (m_Cloud->points.at(j).z - m_minp.z < 1 && m_Cloud->points.at(j).z < m_lmin.z)
            {
                m_lmin = m_Cloud->points.at(j);
            }
        }
    }
    // compute lenght
    m_lenght = sqrt((m_lmax.x - m_lmin.x) * (m_lmax.x - m_lmin.x) + (m_lmax.y - m_lmin.y) * (m_lmax.y - m_lmin.y) +
                    (m_lmax.z - m_lmin.z) * (m_lmax.z - m_lmin.z));
}

float Tree::get_length()
{
    if (m_lenght == -1)
        return m_lenght;
    float AA = ceilf(m_lenght * 100) / 100; // zaokrouhleni
    return AA;
}

pcl::PointXYZI Tree::get_lpoint(bool low)
{
    if (low == false)
    {
        return m_lmin;
    } else
    {
        return m_lmax;
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Tree::get_dbhCloud()
{
    return m_dbhCloud->get_Cloud();
}

// CONVEX & CONCAVE HULL
void Tree::setConvexhull()
{
    if (m_pose.x == -1 && m_pose.y == -1 && m_pose.z == -1)
        return;
    std::string aa = m_name + "_convex";
    //    std::string aa = QString("%1_convex").arg(m_name);
    m_convexhull = new ConvexHull(CloudOperations::getCloudCopy(m_Cloud));
}

ConvexHull &Tree::getConvexhull()
{
    if (m_convexhull == 0)
    {
        setConvexhull();
    }
    return *m_convexhull;
}

void Tree::setConcavehull(float searchDist)
{
    if (m_pose.x == -1 && m_pose.y == -1 && m_pose.z == -1)
        return;
    if (m_concavehull != 0)
    {
        delete m_concavehull;
    }
    std::string aa = m_name + "_concave";
    //    std::string aa = QString("%1_concave").arg(m_name);
    m_concavehull = new ConcaveHull(CloudOperations::getCloudCopy(m_Cloud), aa, searchDist);

    // TriangulatedPolygon *t = new TriangulatedPolygon(CloudOperations::getCloudCopy(m_concavehull->getPolygon().get_Cloud()));
    //  TrianglesToPclMeshTransformation *m = new TrianglesToPclMeshTransformation(m_concavehull->getPolygon().get_Cloud());

    *m_triangulatedConcaveHull = CloudOperations::PolygonToMesh(m_concavehull->getPolygon().get_Cloud());
}

ConcaveHull &Tree::getConcavehull()
{
    if (m_concavehull == 0)
        setConcavehull(1.0);

    return *m_concavehull;
}

pcl::PolygonMesh Tree::getTriangulatedConcaveHull()
{
    return *m_triangulatedConcaveHull;
}

float Tree::getConvexAreaToInfoLine()
{
    if (m_convexhull == 0)
        return 0;
    else
        return m_convexhull->getPolygonArea();
}

float Tree::getConcaveAreaToInfoLine()
{
    if (m_concavehull == 0)
        return 0;
    else
        return m_concavehull->getPolygonArea();
}

// Skeleton
void Tree::set_skeleton()
{
}

void Tree::set_skeleton(Cloud c)
{
    // m_skeleton->set_Cloud(c.get_Cloud());
}

Cloud Tree::get_skeleton()
{
    return *m_dbhCloud;
}

void Tree::set_positionHT(int iter)
{
    if (m_dbhCloud->get_Cloud()->points.size() > 0)
        m_dbhCloud->get_Cloud()->points.clear();
    // vypocitat stred v 1,3 (m_dbh) x v 0,65 m nad pozici

    stred c13;
    bool c13_exist, c065_exist;
    c13_exist = c065_exist = false;
    stred c065;
    pcl::PointXYZI posit;

    if (m_pose.x == -1 && m_pose.y == -1 && m_pose.z == -1)
        posit = m_minp;
    else
        posit = m_pose;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud13(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto it = m_Cloud->points.begin(); it != m_Cloud->points.end(); it++)
    {
        if (it->z > (posit.z + 1.2) && it->z < (posit.z + 1.4))
        {
            pcl::PointXYZI bod;
            bod.x = it->x;
            bod.y = it->y;
            bod.z = it->z;
            bod.intensity = it->intensity;

            cloud13->points.push_back(bod);
        }
    }

    if (cloud13->points.size() > 5)
    {
        HoughTransform ht13;
        ht13.set_Cloud(cloud13);
        ht13.set_iterations(iter);
        ht13.compute();
        c13 = ht13.get_circle();
        c13_exist = true;
    }

    // set cloud in 065 m above position
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud065(new pcl::PointCloud<pcl::PointXYZI>);

    for (auto ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
    {
        if (ith->z > (posit.z + 0.6) && ith->z < (posit.z + 0.7))
        {
            pcl::PointXYZI bod;
            bod.x = ith->x;
            bod.y = ith->y;
            bod.z = ith->z;
            bod.intensity = ith->intensity;

            cloud065->points.push_back(bod);
        }
    }

    // calculate circle
    if (cloud065->points.size() > 5)
    {
        HoughTransform ht065;
        ht065.set_Cloud(cloud065);
        ht065.set_iterations(iter);
        ht065.compute();
        c065 = ht065.get_circle();
        c065_exist = true;
    }

    float vx, vy, vz;
    // if exist both circles
    if (c13_exist == true && c065_exist == true)
    {
        // urcit prusecik
        // parametricka primka
        vx = c065.x - c13.x;
        vy = c065.y - c13.y;
        vz = c065.z - c13.z;
    } else if (c13_exist != true && c065_exist == true)
    {
        pcl::PointXYZI pose;
        pose.x = c065.x;
        pose.y = c065.y;
        pose.z = m_minp.z;

        m_pose = pose;
        set_length();
        set_dbhCloud();
        return;
    } else if (c13_exist == true && c065_exist != true)
    {
        pcl::PointXYZI pose;
        pose.x = c13.x;
        pose.y = c13.y;
        pose.z = m_minp.z;

        m_pose = pose;
        set_length();
        set_dbhCloud();
        return;
    } else
    {
        std::cout << "Warning! " << m_name
                  << " Too few points were found for estimation of position using Hough Transform. Using lowest points method."
                  << std::endl;
        // QMessageBox::information(0, m_name, ("Too few points were found for estimation of position using Randomized Hough Transform. Using lowest points method."));
        set_position();
        return;
    }
    // rovina
    // najit nejnizsi bod stromu x prolozit rovinu
    pcl::PointXYZ A, B, C;
    A.x = m_minp.x + 1;
    A.y = m_minp.y + 1;
    A.z = m_minp.z;

    B.x = m_minp.x - 1;
    B.y = m_minp.y + 2;
    B.z = m_minp.z;

    C.x = m_minp.x;
    C.y = m_minp.y;
    C.z = m_minp.z;

    // vytvo�it rovnici roviny
    pcl::PointXYZ AB;
    AB.x = B.x - A.x;
    AB.y = B.y - A.y;
    AB.z = B.z - A.z;

    pcl::PointXYZ AC;
    AC.x = C.x - A.x;
    AC.y = C.y - A.y;
    AC.z = C.z - A.z;

    float a = (AB.y * AC.z) - (AB.z * AC.y);
    float b = (AB.z * AC.x) - (AB.x * AC.z);
    float c = (AB.x * AC.y) - (AB.y * AC.x);
    float d = -(a * A.x) - (b * A.y) - (c * A.z);
    float up = -d - (a * c13.x) - (b * c13.y) - (c * c13.z);

    float down = a * vx + b * vy + c * vz;
    if (down == 0)
    {
        // QMessageBox::information(0, ("df"), ("rovnobezne"));
        return;
    }
    float t = up / down;
    // dosadit do rovnic primky
    pcl::PointXYZI pos;

    pos.x = c13.x + t * vx;
    pos.y = c13.y + t * vy;
    pos.z = m_minp.z;
    pos.intensity = 1;
    m_pose = pos;

    set_length();
    set_dbhCloud();
    if (m_dbh_RHT.x != -1 && m_dbh_RHT.y != -1 && m_dbh_RHT.r != -1)
        set_dbhRHT();
    if (m_dbh_LSR.x != -1 && m_dbh_LSR.y != -1 && m_dbh_LSR.r != -1)
        set_dbhLSR();
    if (m_height != -1)
        set_height();
    if (!m_stemCurvature.empty())
        set_stemCurvature();
}

void Tree::set_positionHT(Cloud terrain, int iter, int num_points)
{
    if (m_dbhCloud->get_Cloud()->points.size() > 0)
        m_dbhCloud->get_Cloud()->points.clear();
    // vypocitat stred v 1,3 (m_dbh) x v 0,65 m nad pozici
    stred c13, c065;
    bool c13_exist, c065_exist;
    pcl::PointXYZI posit;
    c13_exist = c065_exist = false;

    if (m_pose.x == -1 && m_pose.y == -1 && m_pose.z == -1)
        posit = m_minp;
    else
        posit = m_pose;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud13(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto it = m_Cloud->points.begin(); it != m_Cloud->points.end(); it++)
    {
        if (it->z > (posit.z + 1.25) && it->z < (posit.z + 1.45))
        {
            pcl::PointXYZI bod;
            bod.x = it->x;
            bod.y = it->y;
            bod.z = it->z;
            bod.intensity = it->intensity;

            cloud13->points.push_back(bod);
        }
    }
    if (cloud13->points.size() > 5)
    {
        HoughTransform ht13;
        ht13.set_Cloud(cloud13);
        ht13.set_iterations(iter);
        ht13.compute();
        c13 = ht13.get_circle();
        c13_exist = true;
    }
    cloud13.reset();
    // set cloud in 065 m above position
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud065(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto ith = get_Cloud()->points.begin(); ith != get_Cloud()->points.end(); ith++)
    {
        if (ith->z > (posit.z + 0.6) && ith->z < (posit.z + 0.7))
        {
            pcl::PointXYZI bod;
            bod.x = ith->x;
            bod.y = ith->y;
            bod.z = ith->z;
            bod.intensity = ith->intensity;

            cloud065->points.push_back(bod);
        }
    }

    // calculate circle
    if (cloud065->points.size() > 5)
    {
        HoughTransform ht065;
        ht065.set_Cloud(cloud065);
        ht065.set_iterations(iter);
        ht065.compute();
        c065 = ht065.get_circle();
        c065_exist = true;
    }
    cloud065.reset();

    float vx, vy, vz;
    pcl::PointXYZI pose;
    // if exist both circles
    if (c13_exist == true && c065_exist == true)
    {
        // urcit prusecik
        // parametricka primka
        vx = c065.x - c13.x;
        vy = c065.y - c13.y;
        vz = c065.z - c13.z;
    } else if (c13_exist != true && c065_exist == true)
    {
        pose.x = c065.x;
        pose.y = c065.y;
        pose.intensity = 1;
    } else if (c13_exist == true && c065_exist != true)
    {
        pose.x = c13.x;
        pose.y = c13.y;
        pose.intensity = 1;
    } else
    {
        std::cout << "Warning! " << m_name
                  << " Too few points were found for estimation of position using Hough Transform. Using lowest points method."
                  << std::endl;
        //        QMessageBox::information(0, m_name, ("Too few points were found for estimation of position using Hough Transform. Using lowest points method."));
        set_position(terrain);
        return;
    }
    // rovina
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);
    for (int e = 0; e < terrain.get_Cloud()->points.size(); e++)
    {
        pcl::PointXYZI a;
        a.x = terrain.get_Cloud()->points.at(e).x;
        a.y = terrain.get_Cloud()->points.at(e).y;
        a.z = m_minp.z;
        cloud_tmp->points.push_back(a);
    }
    // najit t�i nejbli��� body ter�nu
    pcl::PointXYZ A, B, C;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud_tmp);

    std::vector<int> pointId(num_points);
    std::vector<float> pointSD(num_points);
    float z = 0;
    if (kdtree.nearestKSearch(posit, num_points, pointId, pointSD) > 0)
    {
        for (int i = 0; i < num_points; i++)
        {
            z += terrain.get_Cloud()->points.at(pointId.at(i)).z;
        }
        z /= num_points;

        A.x = m_minp.x + 1;
        A.y = m_minp.y + 1;
        A.z = z;

        B.x = m_minp.x - 1;
        B.y = m_minp.y + 2;
        B.z = z;

        C.x = m_minp.x;
        C.y = m_minp.y;
        C.z = z;
    } else
    {
        // QMessageBox::information(0, ("tr"), ("No terrain point found in selected cloud. using calculation without terrain cloud."));
        set_positionHT();
        return;
    }

    // vytvo�it rovnici roviny
    pcl::PointXYZ AB;
    AB.x = B.x - A.x;
    AB.y = B.y - A.y;
    AB.z = B.z - A.z;

    pcl::PointXYZ AC;
    AC.x = C.x - A.x;
    AC.y = C.y - A.y;
    AC.z = C.z - A.z;

    float a = (AB.y * AC.z) - (AB.z * AC.y);
    float b = (AB.z * AC.x) - (AB.x * AC.z);
    float c = (AB.x * AC.y) - (AB.y * AC.x);
    float d = -(a * A.x) - (b * A.y) - (c * A.z);

    if (c13_exist == true && c065_exist == true)
    {
        // vypocitat t
        float up = -d - (a * c13.x) - (b * c13.y) - (c * c13.z);
        float down = a * vx + b * vy + c * vz;
        if (down == 0)
        {
            // QMessageBox::information(0, ("df"), ("rovnobezne"));
            return;
        }
        float t = up / down;
        // dosadit do rovnic primky
        pcl::PointXYZI pos;
        pos.x = c13.x + t * vx;
        pos.y = c13.y + t * vy;

        pos.z = (A.z + B.z + C.z) / 3;
        pos.intensity = 1;

        m_pose = pos;
        set_length();
        set_dbhCloud();
    } else if (c13_exist != true && c065_exist == true)
    {
        pose.z = (A.z + B.z + C.z) / 3;
        m_pose = pose;
        set_length();
        set_dbhCloud();
    } else if (c13_exist == true && c065_exist != true)
    {
        pose.z = (A.z + B.z + C.z) / 3;

        m_pose = pose;
        set_length();
        set_dbhCloud();
    }
    if (m_dbh_RHT.x != -1 && m_dbh_RHT.y != -1 && m_dbh_RHT.r != -1)
        set_dbhRHT();
    if (m_dbh_LSR.x != -1 && m_dbh_LSR.y != -1 && m_dbh_LSR.r != -1)
        set_dbhLSR();
    if (m_height != -1)
        set_height();
    if (!m_stemCurvature.empty())
        set_stemCurvature();
}

void Tree::set_stemCurvature(int iter, int height)
{
    if (m_pose.x == -1 && m_pose.y == -1 && m_pose.z == -1)
        return;
    m_stemCurvature.clear();
    float section = (float) height / 100.0;
    for (float g = 0.00; g < (m_maxp.z - m_minp.z); g += section)
    {

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
        // vybrat body do mracna ktere jsou
        for (int i = 0; i < get_Cloud()->points.size(); i++)
        {
            pcl::PointXYZI ith;
            ith = get_Cloud()->points.at(i);
            if (ith.z > (m_pose.z + g - 0.035) && ith.z < (m_pose.z + g +
                                                           0.035)) // && m_dbh_RHT.x -ith.x <5 &&  ith.x - m_dbh_RHT.x < 5&& m_dbh_RHT.y -ith.y <5 &&   ith.y - m_dbh_RHT.y < 5)
            {
                cloud_->points.push_back(ith);
            }
        }
        stred res;
        // if the cloud is empty
        if (cloud_->points.size() > 5)
        {
            HoughTransform *ht = new HoughTransform();
            ht->set_Cloud(cloud_);
            ht->set_iterations(iter);
            ht->compute();
            res = ht->get_circle();
            delete ht;
        } else
            res = {-1, -1, -1, -1, -0.5};

        // if the circle is two times greater than previous two circles
        if (m_stemCurvature.size() >= 2 && res.r > (2 * m_stemCurvature.at(m_stemCurvature.size() - 2).r) &&
            res.r > (2 * m_stemCurvature.at(m_stemCurvature.size() - 1).r))
            res = {-1, -1, -1, -1, -0.5};

        m_stemCurvature.push_back(res);
        cloud_.reset();
    }
}

std::vector<stred> Tree::get_stemCurvature()
{
    return m_stemCurvature;
}

// CROWN
// void Tree::set_TreeCrownAutomatic()
//{
//     if ((m_pose.x == -1 && m_pose.y == -1 && m_pose.z == -1) || m_height == -1)
//         return;
//
//     if (m_crown != 0) { delete m_crown; }
//     //create new crown object and set as tree crown
//     CrownAutomaticDetection *ad = new CrownAutomaticDetection(CloudOperations::getCloudCopy(m_Cloud), get_dbhLSR(),
//                                                               m_pose);
//     std::string name = QString("%1_crown").arg(m_name);
//     m_crown = new Crown(ad->getCrown(), name, m_pose, ad->getStemHighestPoint());
// }
//
// void Tree::set_TreeCrownManual(pcl::PointCloud<pcl::PointXYZI>::Ptr crown, pcl::PointCloud<pcl::PointXYZI>::Ptr stem)
//{
//     if (m_crown != 0) { delete m_crown; }
//     //create new crown object and set as tree crown
//     std::string name = QString("%1_crown").arg(m_name);
//     cloudHighestAndLowestZValue hl = GeomCalc::findHighestAndLowestPoints(stem);
//     m_crown = new Crown(crown, name, m_pose, hl.highestPoint);
// }
//
// Crown &Tree::get_TreeCrown()
//{
//     return *m_crown;
// }

// bool Tree::isCrownExist()
//{
//     if (m_crown == 0) {
//         return false;
//     } else return true;
// }

// PRIVATE

void Tree::compute_PCA()
{
    // compute PCA ang project the result to origin cloud
    pcl::PCA<pcl::PointXYZI> pca;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    pca.setInputCloud(m_Cloud);
    pca.project(*m_Cloud, *cloud_out);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZI> color_t(cloud_out);
    viewer->addPointCloud(cloud_out, color_t, "cloud_out");
    viewer->spin();
}


void Tree::setTrunkInfo()
{
    // 预处理: SOR去除离群点
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(m_Cloud);
    sor.setMeanK(50);            // 设置用于统计的邻居点数量
    sor.setStddevMulThresh(1.0); // 设置标准偏差倍数阈值
    sor.filter(*m_Cloud);        // 更新自身点云

    // 预处理: 降采样
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(m_Cloud);
    voxel_filter.setLeafSize(0.04f, 0.04f, 0.04f);
    voxel_filter.filter(*cloud_downsampled);

    // pcl::PCA<pcl::PointXYZI> pca;
    // pca.setInputCloud(filtered_cloud);
    // m_AxisVector = pca.getEigenVectors().col(0).cast<float>();
    // if (m_AxisVector[2] < 0)
    // {
    //     m_AxisVector = -m_AxisVector;
    // }

    // 拟合直线: 表示树干
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setInputCloud(cloud_downsampled);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.03);
    seg.segment(*inliers, *coefficients); // 内点索引 + 点向参数

    // // 显示树干的直线的参数
    // std::cout << "\nLine coefficients: " << coefficients->values[0] << " "
    //           << coefficients->values[1] << " "
    //           << coefficients->values[2] << " "
    //           << coefficients->values[3] << " "
    //           << coefficients->values[4] << " "
    //           << coefficients->values[5] << std::endl;

    auto p_on_line = Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    m_trunk_axis = Eigen::Vector3f(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    if (m_trunk_axis[2] < 0)
    {
        m_trunk_axis = -m_trunk_axis; // 保证方向向上
    }
    m_trunk_axis.normalize(); // 单位化

    Eigen::Vector4f Centroid;
    pcl::compute3DCentroid(*m_Cloud, Centroid);
    pcl::PointXYZI temp_max_point = pcl::PointXYZI(Centroid[0], Centroid[1], Centroid[2]); // 最高点
    pcl::PointXYZI temp_min_point = pcl::PointXYZI(Centroid[0], Centroid[1], Centroid[2]); // 最低点

    for (const auto &pt: m_Cloud->points)
    {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        Eigen::Vector3f Projection = p_on_line + (p - p_on_line).dot(m_trunk_axis) * m_trunk_axis;
        // projected_cloud->points.push_back(pcl::PointXYZ(Projection[0], Projection[1], Projection[2]));

        if (Projection[2] > temp_max_point.z)
        {
            temp_max_point = pcl::PointXYZI(Projection[0], Projection[1], Projection[2], pt.intensity); // 更新最高点
        }
        if (Projection[2] < temp_min_point.z)
        {
            temp_min_point = pcl::PointXYZI(Projection[0], Projection[1], Projection[2], pt.intensity); // 更新最低点
        }
    }

    m_trunk_maxP = temp_max_point;
    m_trunk_minP = temp_min_point;

    m_trunk_base_point = m_trunk_minP;
    m_pose = m_trunk_minP;

    m_trunk_height = pcl::euclideanDistance(m_trunk_minP, m_trunk_maxP);


    // pcl::PointCloud<pcl::PointXYZI>::Ptr MAX_POINT(new pcl::PointCloud<pcl::PointXYZI>);
    // MAX_POINT->points.push_back(m_trunk_maxP);
    //
    // pcl::PointCloud<pcl::PointXYZI>::Ptr MIN_POINT(new pcl::PointCloud<pcl::PointXYZI>);
    // MIN_POINT->points.push_back(m_trunk_minP);
    //
    // pcl::PointCloud<pcl::PointXYZI>::Ptr PointsOnLine(new pcl::PointCloud<pcl::PointXYZI>);
    // PointsOnLine->points.push_back(pcl::PointXYZI(p_on_line[0], p_on_line[1], p_on_line[2]));
    // // PointsOnLine->points.push_back(m_trunk_minP);
    // // PointsOnLine->points.push_back(m_trunk_maxP);
    //
    // pcl::visualization::PCLVisualizer Treeself_Viewer("TreeSelf Visualizer");
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_tree(m_Cloud, 255, 255, 255);
    // Treeself_Viewer.addPointCloud(m_Cloud, color_tree, "TreeSelf");
    //
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_min_point(MIN_POINT, 255, 0, 0);
    // Treeself_Viewer.addPointCloud(MIN_POINT, color_min_point, "MinPoint");
    // Treeself_Viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "MinPoint");
    //
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_line_points(PointsOnLine, 0, 255, 0);
    // Treeself_Viewer.addPointCloud(PointsOnLine, color_line_points, "Line");
    // Treeself_Viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "Line");
    //
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_max_point(MAX_POINT, 0, 0, 255);
    // Treeself_Viewer.addPointCloud(MAX_POINT, color_max_point, "MaxPoint");
    // Treeself_Viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "MaxPoint");
    //
    // Treeself_Viewer.addArrow(m_trunk_maxP, m_trunk_minP, 0, 0, 1.0, true, "trunk_axis");
    // Treeself_Viewer.addCoordinateSystem(1.0);
    // Treeself_Viewer.spin();

    // pcl::PointCloud<pcl::PointXYZI>::Ptr SELF_COPY(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::copyPointCloud(*m_Cloud, *SELF_COPY);
    // for(auto pt : m_Cloud->points)
    // {
    //     SELF_COPY->points.push_back(pt);
    // }
}


pcl::PointXYZI Tree::getTrunkBasePoint() const
{
    return m_trunk_base_point;
}

Eigen::Vector3f Tree::getTrunkAxis() const
{
    return m_trunk_axis;
}

float Tree::getTrunkHeight() const
{
    return m_trunk_height;
}

pcl::PointXYZI Tree::getTrunkMaxPoint() const
{
    return m_trunk_maxP;
}

pcl::PointXYZI Tree::getTrunkMinPoint() const
{
    return m_trunk_minP;
}

float Tree::get_DBH_Value() const
{
    return m_DBH_Value;
}


void Tree::computeDBH()
{
    using PointType = pcl::PointXYZ;

    Eigen::Vector3f trans(-m_trunk_base_point.x, -m_trunk_base_point.y, -m_trunk_base_point.z); // 平移向量, from Tree to Map_frame
    Eigen::Vector3f axis = m_trunk_axis.cross(Eigen::Vector3f(0, 0, 1)).normalized();           // 旋转轴, from Tree to Map_frame
    double angle = acos(m_trunk_axis.dot(Eigen::Vector3f(0, 0, 1)));                    // 旋转角(弧度), from Tree to Map_frame
    Eigen::AngleAxisf rotation(angle, axis);
    Eigen::Matrix3f rotation_matrix = rotation.toRotationMatrix();

    pcl::PointCloud<PointType>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &pt: m_Cloud->points)
    {
        Eigen::Vector3f temp_point = Eigen::Vector3f(pt.x, pt.y, pt.z);
        Eigen::Vector3f p_transformed = rotation_matrix * (temp_point + trans);
        cloud_transformed->points.push_back(PointType(p_transformed[0], p_transformed[1], p_transformed[2]));
    }
    std::map<std::string, std::pair<pcl::PointCloud<PointType>::Ptr, float>> cloud_map; // first is cloud, second is DBH
    cloud_map["1.10-1.20"].first = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    cloud_map["1.15-1.25"].first = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    cloud_map["1.20-1.30"].first = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    cloud_map["1.25-1.35"].first = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    cloud_map["1.30-1.40"].first = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    cloud_map["1.35-1.45"].first = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    cloud_map["1.40-1.50"].first = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    for (const auto &pt: cloud_transformed->points)
    {
        if (1.10 <= pt.z && pt.z <= 1.20) cloud_map["1.10-1.20"].first->points.push_back(PointType(pt.x, pt.y, 0));
        if (1.15 <= pt.z && pt.z <= 1.25) cloud_map["1.15-1.25"].first->points.push_back(PointType(pt.x, pt.y, 0));
        if (1.20 <= pt.z && pt.z <= 1.30) cloud_map["1.20-1.30"].first->points.push_back(PointType(pt.x, pt.y, 0));
        if (1.25 <= pt.z && pt.z <= 1.35) cloud_map["1.25-1.35"].first->points.push_back(PointType(pt.x, pt.y, 0));
        if (1.30 <= pt.z && pt.z <= 1.40) cloud_map["1.30-1.40"].first->points.push_back(PointType(pt.x, pt.y, 0));
        if (1.35 <= pt.z && pt.z <= 1.45) cloud_map["1.35-1.45"].first->points.push_back(PointType(pt.x, pt.y, 0));
        if (1.40 <= pt.z && pt.z <= 1.50) cloud_map["1.40-1.50"].first->points.push_back(PointType(pt.x, pt.y, 0));
    }

    std::map<std::string, std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, float>> cloud_map_RHT_radius; // first is cloud, second is radius
    cloud_map_RHT_radius["1.10-1.20"].first = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_map_RHT_radius["1.15-1.25"].first = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_map_RHT_radius["1.20-1.30"].first = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_map_RHT_radius["1.25-1.35"].first = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_map_RHT_radius["1.30-1.40"].first = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_map_RHT_radius["1.35-1.45"].first = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_map_RHT_radius["1.40-1.50"].first = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto &pt: cloud_transformed->points)
    {
        if (1.10 <= pt.z && pt.z <= 1.20) cloud_map_RHT_radius["1.10-1.20"].first->points.push_back(pcl::PointXYZI(pt.x, pt.y, pt.z));
        if (1.15 <= pt.z && pt.z <= 1.25) cloud_map_RHT_radius["1.15-1.25"].first->points.push_back(pcl::PointXYZI(pt.x, pt.y, pt.z));
        if (1.20 <= pt.z && pt.z <= 1.30) cloud_map_RHT_radius["1.20-1.30"].first->points.push_back(pcl::PointXYZI(pt.x, pt.y, pt.z));
        if (1.25 <= pt.z && pt.z <= 1.35) cloud_map_RHT_radius["1.25-1.35"].first->points.push_back(pcl::PointXYZI(pt.x, pt.y, pt.z));
        if (1.30 <= pt.z && pt.z <= 1.40) cloud_map_RHT_radius["1.30-1.40"].first->points.push_back(pcl::PointXYZI(pt.x, pt.y, pt.z));
        if (1.35 <= pt.z && pt.z <= 1.45) cloud_map_RHT_radius["1.35-1.45"].first->points.push_back(pcl::PointXYZI(pt.x, pt.y, pt.z));
        if (1.40 <= pt.z && pt.z <= 1.50) cloud_map_RHT_radius["1.40-1.50"].first->points.push_back(pcl::PointXYZI(pt.x, pt.y, pt.z));
    }

    for (auto map_it: cloud_map_RHT_radius) // compute DBH_RHT of selected DBH_CLOUD
    {
        this->set_dbhCloud(map_it.second.first);
        this->set_dbhRHT(400);
        map_it.second.second = this->get_dbhRHT().r; // store the radius in map and will be chosen later
    }

    // choosing
    float mean_radius = 0, std_radius = 0;
    for (auto map_it: cloud_map_RHT_radius)
    {
        mean_radius += map_it.second.second;
    }
    mean_radius /= cloud_map_RHT_radius.size();

    for (auto map_it: cloud_map_RHT_radius)
    {
        std_radius += pow(map_it.second.second - mean_radius, 2);
    }
    std_radius = sqrt(std_radius / cloud_map_RHT_radius.size());

    if (abs(cloud_map_RHT_radius.find("1.25-1.35")->second.second - mean_radius) <= 2 * std_radius)
    {
        this->m_dbh_RHT.r = cloud_map_RHT_radius.find("1.25-1.35")->second.second;
        this->m_DBH_Value = this->m_dbh_RHT.r * 2;
        return;
    } else
    {
        std::vector<float> useful_DBH;
        for (auto map_it: cloud_map_RHT_radius)
        {
            if (abs(map_it.second.second - mean_radius) <= 2 * std_radius) // Only useful DBH values
                useful_DBH.push_back(map_it.second.second);
        }
        this->m_dbh_RHT.r = std::accumulate(useful_DBH.begin(), useful_DBH.end(), 0.0) / useful_DBH.size(); // Mean of useful DBH values
        this->m_DBH_Value = this->m_dbh_RHT.r * 2;
        return;
    }

    //show the transformed cloud
    // pcl::visualization::PCLVisualizer treeself_Viewer_("TreeSelf Visualizer");
    // pcl::visualization::PointCloudColorHandlerCustom<PointType> color_transformed(cloud_transformed, 255, 255, 255);
    // treeself_Viewer_.addPointCloud(cloud_transformed, color_transformed, "TreeSelf");

    // pcl::PointCloud<PointType>::Ptr BASE_POINT(new pcl::PointCloud<PointType>);
    // Eigen::Vector3f base_p             = Eigen::Vector3f(this->m_trunk_base_point.x, this->m_trunk_base_point.y, this->m_trunk_base_point.z);
    // Eigen::Vector3f base_p_transformed = rotation_matrix * (base_p + trans);
    // BASE_POINT->points.push_back(PointType(base_p_transformed[0], base_p_transformed[1], base_p_transformed[2]));
    // pcl::visualization::PointCloudColorHandlerCustom<PointType> color_base(BASE_POINT, 255, 0, 0);
    // treeself_Viewer_.addPointCloud(BASE_POINT, color_base, "BasePoint");
    // treeself_Viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "BasePoint");// 设置点的大小
    // treeself_Viewer_.addCoordinateSystem(1.0);
    // treeself_Viewer_.resetCamera();
    // treeself_Viewer_.spin();
    //
    // for (auto it = cloud_map.begin(); it != cloud_map.end(); it++)
    // {
    //     //
    //     cout << "\nThe size of point cloud of " << it->first << " : " << it->second.first->points.size() << endl;
    //     // show the result cloud in order to check
    //     pcl::visualization::PCLVisualizer _Treeself_Viewer("TreeSelf Visualizer");
    //     pcl::visualization::PointCloudColorHandlerCustom<PointType> color_cloud(it->second.first, 255, 255, 255);
    //     _Treeself_Viewer.addPointCloud(it->second.first, color_cloud, "TreeSelf");
    //     _Treeself_Viewer.addCoordinateSystem(1.0);
    //     _Treeself_Viewer.spin();
    // }


    DBH_RANSAC_CIRCLE(cloud_map); // 分别拟合圆，并挑选最佳值
    CHOOSING(cloud_map);          // 选择最佳值

    // // find branch with order 0
    // float distance = 1000000000;
    // stred dbh;
    // for (int i = 0; i < m_branches.size(); i++)
    // {
    //     if (m_branches.at(i)->getOrder() == 0)
    //     {
    //         // get height
    //         for (int q = 0; q < m_branches.at(i)->getCylinders().size(); q++)
    //         {
    //             if (std::abs(m_branches.at(i)->getCylinders().at(q)->values.at(2) - (m_pose.z + 1.3)) < distance)
    //             {
    //                 distance = std::abs(m_branches.at(i)->getCylinders().at(q)->values.at(2) - (m_pose.z + 1.3));
    //                 dbh.x = m_branches.at(i)->getCylinders().at(q)->values.at(0);
    //                 dbh.y = m_branches.at(i)->getCylinders().at(q)->values.at(1);
    //                 dbh.z = m_branches.at(i)->getCylinders().at(q)->values.at(2);
    //                 dbh.r = m_branches.at(i)->getCylinders().at(q)->values.at(6) * 100;
    //             }
    //         }
    //         break;
    //     }
    // }
    // return dbh;
}

void Tree::DBH_RANSAC_CIRCLE(std::map<std::string, std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, float>> &_cloud_map)
{
    using PointType = pcl::PointXYZ;
    std::for_each(std::execution::par_unseq, _cloud_map.begin(), _cloud_map.end(), [](auto &selected_cloud)
    {
        if (selected_cloud.second.first->points.size() < 5)
        { //如果选择的点云里面点的个数小于5，则标记为异常
            selected_cloud.second.second = -1;
            return;
        }

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr model_coefficients(new pcl::ModelCoefficients);
        pcl::SACSegmentation<PointType> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CIRCLE2D);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(selected_cloud.second.first);
        seg.segment(*inliers, *model_coefficients);

        selected_cloud.second.second = model_coefficients->values[3] * 2; // Radius
    });
}

void Tree::CHOOSING(std::map<std::string, std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, float>> &_cloud_map)
{
    // 选择最佳的DBH值
    std::vector<float> DBH_values;
    for (auto &it: _cloud_map)
    {
        DBH_values.push_back(it.second.second);
    }
    std::sort(DBH_values.begin(), DBH_values.end());
    DBH_values.erase(DBH_values.begin()); // 删除第一个元素（最小）
    DBH_values.pop_back();                // 删除最后一个元素（最大）


    double sum = std::accumulate(DBH_values.begin(), DBH_values.end(), 0.0);
    float mean = sum / DBH_values.size();
    // float BEST_DBH_VALUE = DBH_values[DBH_values.size() / 2]; // 中位数
    // float BEST_DBH_VALUE;

    m_DBH_Value = mean;
}

void Tree::setStatus(bool signal)
{
    m_trunk_status = signal;
}

bool Tree::getStatus() const
{
    return m_trunk_status;
}


pcl::PointXYZI Tree::getMinP()
{
    return m_minp;
}

pcl::PointXYZI Tree::getMaxP()
{
    return m_maxp;
}

pcl::PointXYZI Tree::getMinL()
{
    return m_lmin;
}

pcl::PointXYZI Tree::getMaxL()
{
    return m_lmax;
}

void Tree::setSkeleton(float voxelSize, float multiplicator)
{
    Skeleton *s = new Skeleton(m_Cloud, voxelSize, multiplicator);
    s->setPosition(get_pose());
    s->compute();
    setSegments(s->getSegments());
    set_Cloud(s->getCloud());
    delete s;
}

void Tree::setSegments(std::vector<std::shared_ptr<Segment>> branches)
{
    m_branches = branches;
}

std::vector<std::shared_ptr<Segment>> Tree::getBranches()
{
    return m_branches;
}

void Tree::setQSM(int iterations, float CylinderHeight, float limit, float branchLength, int order, bool stemCurve)
{
    CylinderModel *c = new CylinderModel(getBranches(), iterations, CylinderHeight);
    c->setTreecloud(m_Cloud);
    c->setLimit(limit);
    c->setTreeHeight(get_length());
    c->setTreePosition(get_pose());
    c->setBranchLength(branchLength);
    c->setOrder(order);
    c->compute();
    setCylinders(c->getCylinders());
    setVolume();
    setDBH_QSM();
    if (stemCurve == true)
    {
        m_stemCurvature.clear();
        m_stemCurvature = c->getStemCurve();
    }
    delete c;
}

void Tree::setDBH_QSM()
{
    // find branch with order 0
    float distance = 1000000000;
    for (int i = 0; i < m_branches.size(); i++)
    {
        if (m_branches.at(i)->getOrder() == 0)
        {
            // get height
            for (int q = 0; q < m_branches.at(i)->getCylinders().size(); q++)
            {
                if (std::abs(m_branches.at(i)->getCylinders().at(q)->values.at(2) - (m_pose.z + 1.3)) < distance)
                {
                    distance = std::abs(m_branches.at(i)->getCylinders().at(q)->values.at(2) - (m_pose.z + 1.3));
                    // std::cout<<"distance " << distance << " q: "<< q <<" z celkoveho poctu " <<m_branches.at(i)->getCylinders().size()<<  "\n";
                    m_dbh_QSM.x = m_branches.at(i)->getCylinders().at(q)->values.at(0);
                    m_dbh_QSM.y = m_branches.at(i)->getCylinders().at(q)->values.at(1);
                    m_dbh_QSM.z = m_branches.at(i)->getCylinders().at(q)->values.at(2);
                    m_dbh_QSM.r = m_branches.at(i)->getCylinders().at(q)->values.at(6) * 100;
                }
            }
            break;
        }
    }
}

stred Tree::getDBH_QSM()
{
    return m_dbh_QSM;
}

void Tree::setSortiment(bool finalSortiment)
{
    ComputeSortiment *s = new ComputeSortiment(getBranches(), finalSortiment);
    s->compute();
    // std::cout<<"pocet cylindru: " << getCylinderSize()<< "\n";
    //    if(finalSortiment == false)
    //    {
    //        setCylinders(s->getCylinders());
    //        setSortiments(s->getSortiments());
    //    }
    //    else{
    //        for(int i=1; i < 7; i++)
    //        {
    //            setSortimentVolume(i, s->getSortimentVolume(i));
    //            setSortimentLenght(i, s->getSortimentLenght(i));
    //        }
    //       // setCylinders(s->getCylinders());
    //        //setSortiments(s->getSortiments());
    //    }

    // std::cout<<"pocet cylindru: " << getCylinderSize()<< "\n";
    // std::cout<<"pocet sortimentu: " << getSortimentsSize()<< "\n";

    delete s;
}

void Tree::setSortiments(std::vector<int> s)
{
    m_sortiments = s;
}

int Tree::getSortimentsSize()
{
    int vel = 0;
    for (int i = 0; i < m_branches.size(); i++)
        vel += m_branches.at(i)->getSortiments().size();
    return vel;
}

std::vector<std::shared_ptr<Sortiment>> Tree::getSortiments()
{
    std::vector<std::shared_ptr<Sortiment>> sortiments;
    for (int i = 0; i < m_branches.size(); i++)
    {
        sortiments.insert(sortiments.end(), m_branches.at(i)->getSortiments().begin(),
                          m_branches.at(i)->getSortiments().end());
    }
    return sortiments;
}

int Tree::getSortiment(int i)
{
    return m_sortiments.at(i);
}

void Tree::setVolume()
{
    m_volume = 0;
    // pro kazdy cylinder compute volume, and add to the m_volume
    for (int i = 0; i < m_cylinders.size(); i++)
    {
        m_volume += cylinderVolume(m_cylinders.at(i));
    }
    std::cout << "strom: " << get_name() << " volume: " << m_volume << "\n";
    //    std::cout << "strom: " << get_name().toStdString() << " volume: " << m_volume << "\n";
}

float Tree::cylinderVolume(pcl::ModelCoefficients::Ptr model)
{
    // m_PI * radius* radius * v

    float v = 0;
    float volume = 0;
    v = std::sqrt(
            (model->values.at(3) * model->values.at(3)) +
            (model->values.at(4) * model->values.at(4)) +
            (model->values.at(5) * model->values.at(5)));

    volume = (M_PI * (model->values.at(6) * model->values.at(6)) * v);

    if (std::isnan(v))
    {
        v = 0;
        volume = 0;
    }
    //  std::cout<<"valce hodnoty: x: "<<model->values.at(3) << " y: " << model->values.at(4)<< " z: " << model->values.at(5)<< "\n";
    // std::cout<< "prumer: " << model->values.at(6) << " vyska: "<< v <<" volume: "<<  volume<< " m_volume: "<< m_volume<< "\n";

    return volume;
}

float Tree::getVolume()
{
    return m_volume;
}

void Tree::setCylinder(pcl::ModelCoefficients::Ptr cylinder)
{
    m_cylinders.push_back(cylinder);
}

pcl::ModelCoefficients::Ptr Tree::getCylinder(int i)
{
    return m_cylinders.at(i);
}

int Tree::getCylinderSize()
{
    return m_cylinders.size();
}

void Tree::setCylinders(std::vector<pcl::ModelCoefficients::Ptr> cylinders)
{
    m_cylinders.clear();
    m_cylinders = cylinders;
}

float Tree::getSortimentVolume(int sortimentID)
{
    switch (sortimentID)
    {
        case 1:
            return m_sortiment1_volume;
            break;
        case 2:
            return m_sortiment2_volume;
            break;
        case 3:
            return m_sortiment3_volume;
            break;
        case 4:
            return m_sortiment4_volume;
            break;
        case 5:
            return m_sortiment5_volume;
            break;
        case 6:
            return m_sortiment6_volume;
            break;

        default:
            return -1;
            break;
    }
}

void Tree::setSortimentVolume(int sortimentID, float volume)
{
    switch (sortimentID)
    {
        case 1:
            m_sortiment1_volume = volume;
            break;
        case 2:
            m_sortiment2_volume = volume;
            break;
        case 3:
            m_sortiment3_volume = volume;
            break;
        case 4:
            m_sortiment4_volume = volume;
            break;
        case 5:
            m_sortiment5_volume = volume;
            break;
        case 6:
            m_sortiment6_volume = volume;
            break;

        default:
            m_sortiment1_lenght = -1;
            break;
    }
}

float Tree::getSortimentLenght(int sortimentID)
{
    switch (sortimentID)
    {
        case 1:
            return m_sortiment1_lenght;
            break;
        case 2:
            return m_sortiment2_lenght;
            break;
        case 3:
            return m_sortiment3_lenght;
            break;
        case 4:
            return m_sortiment4_lenght;
            break;
        case 5:
            return m_sortiment5_lenght;
            break;
        case 6:
            return m_sortiment6_lenght;
            break;

        default:
            return -1;
            break;
    }
}

void Tree::setSortimentLenght(int sortimentID, float lenght)
{
    switch (sortimentID)
    {
        case 1:
            m_sortiment1_lenght = lenght;
            break;
        case 2:
            m_sortiment2_lenght = lenght;
            break;
        case 3:
            m_sortiment3_lenght = lenght;
            break;
        case 4:
            m_sortiment4_lenght = lenght;
            break;
        case 5:
            m_sortiment5_lenght = lenght;
            break;
        case 6:
            m_sortiment6_lenght = lenght;
            break;

        default:
            m_sortiment1_lenght = -1;
            break;
    }
}

float Tree::getQSMDBH()
{
    return m_dbh_QSM.r * 20;
}

float Tree::getQSMVolume()
{
    float volume = 0;
    for (int q = 0; q < m_branches.size(); q++)
        volume += m_branches.at(q)->getTotalVolume();
    return volume;
}

float Tree::getQSMVolumeHroubi()
{
    float volume = 0;
    for (int q = 0; q < m_branches.size(); q++)
        volume += m_branches.at(q)->getHroubiVolume();
    return volume;
}
