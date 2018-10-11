#pragma once

//PCL
#include <pcl\point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>

//STL
#include <iostream>
#include <string>
#include <vector>

// CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Triangulation_conformer_2.h>

struct FaceInfo2
{
  FaceInfo2(){}
  int nesting_level;
  bool in_domain(){ 
    return nesting_level%2 == 1;
  }
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel       K;
typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2,K>    Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K,Fbb>        Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>               TDS;
typedef CGAL::Exact_predicates_tag                                Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>  CDT;
typedef CDT::Point                                                Point;
typedef CGAL::Polygon_2<K>                                        Polygon_2;

class ConstrainedDelaunayTriangulation
{
public:
	ConstrainedDelaunayTriangulation(void);
	~ConstrainedDelaunayTriangulation(void);

	Polygon_2 m_ply;

    /***************************************ֻ�ж���α߽��ϵĵ�*****************************************/
	// �������ζ�������
	void setInputPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ply);
	// ִ��CDT���ǻ�(�������µĶ���)
	// triangles: ����μ���(N-2����NΪ����ζ������)
	// tri_index_with_min_angle: ������С�ڽǵ�����������
	// min_angle: �����ε���С�ڽ�
    void triangulate(std::vector<std::vector<int>> &triangles, std::vector<float> &min_angles);
    /**********************************����α߽��ϵĵ�+������ڲ��Ĺ�����*********************************/
    // �������ζ��������Լ���������
    // ���й�������Ĭ�Ͼ�λ�ڶ���ε��ڲ�
    void setInputPointCloud_and_triangulate(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int ply_size,
                                             std::vector<std::vector<int>> &triangles, std::vector<float> &min_angles);

private:
	void mark_domains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border);
	void mark_domains(CDT& cdt);
	// ִ��CDT���ǻ�(�������µĶ���)
	void triangulate(std::vector<std::vector<int>> &triangles);
};

