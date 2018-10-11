#pragma once
#include "initialPoly.h"
#include <pcl/segmentation/sac_segmentation.h>

class SimplifyVerticesSize
{
public:
	SimplifyVerticesSize(void);
    ~SimplifyVerticesSize(void);
    inline
    void setParams(float T1, float T2, float T3, float T4, float T5){
        T_angle_betPointAndPoint = T1;
        T_angle_betPointAndLine = T2;
        T_dist_betPointAndLine = T3;
        T_lenRatio = T4;
        T_angle_betLines = T5;

        // �Ƕ�ת����
        double PI = 3.14159265358979;
        T_angle_betPointAndPoint = T_angle_betPointAndPoint * PI / 180.0f;
        T_angle_betPointAndLine = T_angle_betPointAndLine * PI / 180.0f;
        T_angle_betLines = T_angle_betLines * PI / 180.0f;
    }

    // �������ζ��㷨����(Ĭ�϶��������Ѿ��������ַ���)
	// ������ǰ�󶥵��Լ���ǰ����ȷ����ǰ����ķ�����
	void computeVerticesNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &contour, 
	                        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &vertices_normals,
							Eigen::Vector3f &plane_normal);
	// �������������ʼ���߶μ���
	void constructInitLineSegs( pcl::PointCloud<pcl::PointXYZ>::Ptr &contour,
								std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &vertices_normals,
								Eigen::Vector3f &poly_normal,
								std::vector<std::vector<int>> &line_segs );
	std::vector<int> final_line_segs;
	// �����Ż�����εĶ�������
	void refineLineSegs( pcl::PointCloud<pcl::PointXYZ>::Ptr &contour,			// ԭʼ����
						 pcl::PointCloud<pcl::PointXYZ>::Ptr &final_vertices,	// ���ն���
						 std::vector<std::vector<int>> &retrieved_line_segs, // ������������ȡ���߶�����
						 std::vector<int> &final_line_segs );	 // ���յ��߶�����

    //void simplifyPolyVertices(MyPoly &poly);

	void vectorSort(std::vector<float> &v, std::vector<int> &indices);

protected:
	void getSeedIndex( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
					   int &seed_index,
					   std::vector<bool> &is_processed,
					   std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &vertices_normals );
	float distP2Line( pcl::PointXYZ &point,
					  Eigen::Vector3f &p_base,	// ֱ��ģ�͵Ļ���
					  Eigen::Vector3f &line_dir );
	float distP2P(pcl::PointXYZ &p1, pcl::PointXYZ &p2);

private:
    float T_angle_betPointAndPoint;	// ѡȡ���ӵ�ʱ�����ӵ㷨���������ڶ��㷨���������нǣ���λ�����ȣ�
    float T_angle_betPointAndLine;	// ��ķ�������ֱ�߷����������нǣ���λ�����ȣ�
    float T_dist_betPointAndLine;	// �㵽ֱ�ߵ�������
    float T_lenRatio;               // ȷ�����Ա���������С�߶�ռƽ���߶γ��ȵ�������
    float T_angle_betLines;         // �ɱ��ϲ��������߶ε����нǣ���λ�����ȣ�
};

