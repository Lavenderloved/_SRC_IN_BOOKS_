#include "cloud.h"

// CLOUD
Cloud::Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string name)
{
    m_name = name;
    m_Cloud = cloud;
    std::srand(time(0));
    m_PointSize = 1;
}

Cloud::Cloud()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    m_name = "";
    m_Cloud = cloud;
    m_PointSize = 1;
}

Cloud::Cloud(const Cloud &copy) // 拷贝构造函数
{
    m_Cloud = copy.m_Cloud;
    m_name = copy.m_name;
    m_PointSize = copy.m_PointSize;
}

Cloud::~Cloud()
{
    m_Cloud.reset();
}

Cloud Cloud::operator=(Cloud &copy)
{
    Cloud t;
    t.set_name(copy.get_name());
    t.set_Cloud(copy.get_Cloud());
    t.set_Psize(copy.get_Psize());
    return t;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud::get_Cloud()
{
    return m_Cloud;
}

std::string Cloud::get_name()
{
    return m_name;
}

void Cloud::set_Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    *m_Cloud = *cloud;
}

void Cloud::set_name(std::string name)
{
    m_name = name;
}

void Cloud::set_Psize(int p)
{
    m_PointSize = p;
}

int Cloud::get_Psize()
{
    return m_PointSize;
}
