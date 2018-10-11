#pragma once

//#ifndef Q_MOC_RUN
//// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#endif

class InitialPoly
{
public:
    InitialPoly(){
        vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
        simplified_vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
        normal << 0, 0, 0;
        scale = 0;
    }

    // ����Ķ�������
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices;
    // ����η�����
    Eigen::Vector3f normal;
    // ����γ߶�
    float scale;

    // ���㻯��
    // ����֮��Ķ�������
    pcl::PointCloud<pcl::PointXYZ>::Ptr simplified_vertices;
    // ���㷨����
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices_normals;
    // ��������ʱ��⵽���߶�����
    std::vector<std::vector<int>> line_segs;
    // ���յ��߶�����
    std::vector<int> final_line_segs;
};
