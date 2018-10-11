#ifndef EARCLIP_H
#define EARCLIP_H
#include <pcl/point_cloud.h>
#include <vector>
#include "snappingvertices.h"

class EarClip
{
public:
    EarClip();
    // ��������Ķ����
    void setInputPoly(POLYGON *pPoly);
    // �Զ���ν������ǻ�
    bool triangulatePoly();

private:
    POLYGON * m_poly;
    vector<Vertex *> vertices;  // ����ζ���λ��˳��洢

    double PI;

    // ��ȡ����i��ǰ�����
    int getPrev(std::vector<int> &indices, int i);
    // ��ȡ����i�ĺ�̽��
    int getNext(std::vector<int> &indices, int i);
    // ɾ��vector�еĵ�i��Ԫ��
    void del_vertex(std::vector<int> &indices, int i);
    // ĳ���Ƿ����������ڲ�
    // Ŀ�궥�������: p_index
    // �����ζ�������: tri_v1, tri_v2, tri_v3
    bool isPointInTriangle(int p_index, int tri_v1, int tri_v2, int tri_v3);
    // ��ȡ����ζ�����ڽ�
    // vertices: �������У���ʱ������
    // normal: ����η�������ָ�������ⲿ
    // index: Ŀ�궥������
    // prev: index��ǰ��
    // next: index�ĺ��
    double getInnerAngle(vector<Vertex *> &vertices, Eigen::Vector3f &normal, int index, int prev, int next);
    // ��-����뺯��
    float distP2P(pcl::PointXYZ &p1, pcl::PointXYZ &p2);
};

#endif // EARCLIP_H
