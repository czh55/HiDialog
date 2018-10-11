#ifndef SNAPPINGVERTICES_H
#define SNAPPINGVERTICES_H

#include "initialPoly.h"
//#include "earclip.h"

using namespace std;

struct MatchingPoly
{
    int poly_id;
    vector<void *> vertices;
};

// �������ݽṹ
struct Vertex
{
    pcl::PointXYZ point;	// ����λ��

    float radius;			// �����뾶

    Vertex* next_point;     // ǰ���ڵ�
    Vertex* prev_point;     // ��̽ڵ�

    int neighbor_poly_id;        // �ھӶ����id
    bool is_vertex_neighbor;    // ����������Ƕ��㻹�Ǳ�
    Vertex* neighbor_vertex;    // �ھӶ���
    Vertex* neighbor_edge;      // �ھӱ�

    vector<MatchingPoly> maching_polys; // ��õ���С���-�㡱ƥ���ϵ�Ķ���ζ���(����һ������ε��ļ������������ƥ���ϵ)

    bool is_updated;

	bool is_intersectPoint;	// �߶��ཻ����Ǹö����Ƿ�Ϊ����
};

// ������
struct Triangle
{
    /*Vertex *v0;
    Vertex *v1;
    Vertex *v2;*/
    Vertex* v[3];
};

class POLYGON
{
private:
    int size;

public:
    Vertex *start_point;        // ��������
    Vertex *cur_point;
    Eigen::Vector4f coeff;      // �����ƽ�淽��(a,b,c,d: ax+by+cz+d=0)

    vector<Triangle> triangles; // �Զ���ν������ǻ�֮���С�����μ���

    POLYGON(){
        start_point = cur_point = NULL;
        size = 0;
        coeff << 0, 0, 0, 0;
    }

	~POLYGON(){
        //clear();
	}

	// ��ն����
	void clear(){
		Vertex *pv, *pv_next;
		for(int i = 0; i < size; ++i){
			pv = i == 0 ? this->start_point : pv_next;
			pv_next = pv->next_point;
			delete pv;
		}
		size = 0;
		coeff << 0, 0, 0, 0;
		this->start_point = this->cur_point = NULL;
	}

    int getSize(){return size;}

    // �����һ������
    void insertFirstVertex(Vertex *v){
        start_point = v;
        cur_point = v;
        ++size;
    }
    // �����м䶥��
    void insertMiddleVertex(Vertex *v){
        cur_point->next_point = v;
        v->prev_point = cur_point;
        // ��v��Ϊ��ǰ����
        cur_point = v;
        ++size;
    }
    // �������һ������
    void insertLastVertex(Vertex *v){
        cur_point->next_point = v;
        v->prev_point = cur_point;
        v->next_point = start_point;
        start_point->prev_point = v;
        ++size;
    }

    // ɾ������
    void delVertex(Vertex *v){
        if(isVertexInPoly(v)){
            --size;

            // ���v����ʼ���㣬�������ʼ����
            if(v == this->start_point){
                this->start_point = v->next_point;
            }

            Vertex* prev_v, * next_v;
            prev_v = v->prev_point;
            next_v = v->next_point;
            prev_v->next_point = next_v;
            next_v->prev_point = prev_v;
            delete v;
            v = NULL;
        }else{
            cout << "vertex is not a member of this Polygons" << endl;
        }
    }

    // �ڶ���v�������һ���µĶ���new_v
    bool insertVertex(Vertex *v, Vertex *new_v){
        if(!isVertexInPoly(v)) return false;

		// ����new_v�������뾶
		new_v->radius = v->radius;

        Vertex *v_next = v->next_point;
        v->next_point = new_v;
        new_v->prev_point = v;
        new_v->next_point = v_next;
        v_next->prev_point = new_v;
        ++size;
        return true;
    }

    // �жϵ��Ƿ��ڶ������
    bool isVertexInPoly(Vertex *v){
        if(size == 0) return false;

        for(int i = 0; i < size; ++i){
            cur_point = i == 0 ? start_point : cur_point->next_point;
            if(cur_point == v){
                return true;
            }
        }
        return false;
    }

    bool operator==(const POLYGON& poly) const
    {
       return poly.start_point == this->start_point;
    }
};

// ���㲻�����߶ν�������ݽṹ
// �豣���������������߶��ཻ���Լ�������Ϣ
struct IntersectingPoint{
    int edge_id1;   // ��id
    int edge_id2;   // ��id
    pcl::PointXYZ point;
	Vertex *edge_id1_begin;
	Vertex *edge_id2_begin;
};

class SnappingVertices
{
public:
    SnappingVertices();

    float ratio_of_scale;              // �߶ȵı������ӣ�������������뾶
    float T_ratio_lineSeg_middle_area; // �߶��в�����ռ���߶γ��ȵı���
    float T_dist_proj_to_lineSegEnd;   // ͶӰ�����߶ζ˵��������
    float T_maxAngle_bet_two_polys;    // ����ƽ�ж���εķ��������н�

    void setParams(float &T1, float &T2, float &T3, float &T4);

    // ����νṹ
    vector<POLYGON, Eigen::aligned_allocator<POLYGON>> polygons;
    // ����ƥ���ϵ�Ķ����
    vector<vector<Vertex*>> matchingVerticesClusters;
    // ����ƥ���ϵ�Ķ����
    vector<vector<Vertex*>> clusters;

    // �������μ���
    void setPolygons(vector<InitialPoly> &initial_polys);
    // Ѱ�Ҷ����P�Ķ���v������������������
    void getNeighborPolygon(int poly_id/*P*/, Vertex* v);
    // Ϊ����ε�ÿ���������������ϵ
    void setNeighborsForEachPoly();
    // �ڻ�ȡ�ھӶ���ε�ʱ��Ϊÿ�����㹹���˵����ƥ���ϵ
    // �ú���������˫���ƥ���ϵ
    void setMatchingRelations();
    // �ڶ���v��ƥ�������м���Ƿ���ڱ��Ϊid�Ķ���Σ��������������λ��
    bool findPoly(Vertex *v, int poly_id, int &poly_id_in_v);
    // ƥ����Ƿ����
    bool findVertex(Vertex *v, int poly_id_in_v, Vertex *neighbor_v);

    // ����ƥ���ϵ����Ҫ�������������
    // ����v���С���-�ߡ�ƥ���ϵ��ͬʱ��Ķ���v_��v�־���"��-�㡱ƥ��
    // ������ԣ����ž�����ʵ�ֵ�-��ƥ���ԭ��ȥ��v�ġ���-�ߡ�ƥ��
    void refineMachingRelations();

    // ��ȡƥ��Ķ����
    void getMatchingVerticesClusters(vector<vector<Vertex*>> &clusters);

    // �����-��ƥ��
    // �ϲ�����ƥ���ϵ�ĵ㼯
    void mergeMatchingVerticesClusters(vector<vector<Vertex*>> &clusters);
    void dealWithEachCluster(vector<Vertex*> &cluster);
    // ������ֱ�������������εĶ����
    void dealWithEachCluster2(vector<Vertex*> &cluster);
    // ������������εĽ���
    bool getIntersectLine(Eigen::Vector4f &coeff1, Eigen::Vector4f &coeff2,
                          Eigen::Vector3f &p_base, Eigen::Vector3f &line_dir);
    // ��ȡ����ֱ���ϵ�ͶӰ��
    void getProjPointOnLine(pcl::PointXYZ &p, pcl::PointXYZ &p_proj,
                            Eigen::Vector3f &p_base, Eigen::Vector3f &line_dir);

    // ������������εĽ���
    void getIntersectPointOfTriplePolys(Eigen::Vector4f &coeff1,
                                        Eigen::Vector4f &coeff2,
                                        Eigen::Vector4f &coeff3,
                                        pcl::PointXYZ &p);

    // ����ƽ�淨�����ļн�
    float getIncludedAngleOfTwoPolys(int poly_a, int poly_b);

    // �ж�����������Ƿ�ƽ��
    bool areTwoPolysParallel(int poly_a, int poly_b);

    // ��ȡ�����߶��ϵ�ͶӰ��
    // e1, e2: �߶ζ˵�
    void getProjPointOnLineSeg(pcl::PointXYZ &p, pcl::PointXYZ &p_proj, pcl::PointXYZ &e1, pcl::PointXYZ &e2);
    // �ж����������Ƿ��ϸ����
    bool isEqual(pcl::PointXYZ &p1, pcl::PointXYZ &p2){
        return (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z);
    }

    // ���뺯��
    // ��-��
    float distP2P(pcl::PointXYZ &p1, pcl::PointXYZ &p2);
    // ��-��
    // �����p��ͶӰ�������߶��ڲ����򷵻�p��ͶӰ��ľ��룻���򷵻�FLT_MAX
    float distP2E(pcl::PointXYZ &p, pcl::PointXYZ &e1, pcl::PointXYZ &e2);

    // ������-�ߡ�ƥ��
    void dealWithVertexEdgeMatchingRelation();

    // ���������˻����
    void dealWithPolysDegeneration(vector<POLYGON, Eigen::aligned_allocator<POLYGON>> &polygons);   // ���庯��
    void dealWithSinglePolyDegeneration(POLYGON &poly);  // ��������εĴ�����
    // ����������(���ڶ���λ���غϣ������ڶ���λ���غϣ�
    void dealWithVerticesRedundancy(POLYGON &poly, pcl::PointCloud<pcl::PointXYZ>::Ptr &vertices);
    void dealWithLineSegCollineation(POLYGON &poly, pcl::PointCloud<pcl::PointXYZ>::Ptr &vertices);         // ���������߶ι���
    void dealWithLineSegIntersect(POLYGON &poly, pcl::PointCloud<pcl::PointXYZ>::Ptr &vertices);        // ����ǽ����߶ε��ཻ
    void cleanMatchingRelaion();    // ����ÿ�������ƥ���ϵ

    // �Ӻ���
    // ���㶥��index1��index2����·�����߶εĳ���֮��
    float lineSegDist(int index1, int index2, vector<float> &line_lens);

    // ���������߶εĽ���
    // ֻ�н�������������߶��ڲ����Ż᷵�غϷ���ֵ
    bool getIntersectPointOfTwoLineSeg(pcl::PointXYZ &line1_begin, pcl::PointXYZ &line1_end,
                                       pcl::PointXYZ &line2_begin, pcl::PointXYZ &line2_end,
                                       pcl::PointXYZ &intersect_point);
    // ����һ������������в������߶εĽ���
    void getIntersectPointsOfPoly(pcl::PointCloud<pcl::PointXYZ>::Ptr &vertices,
                                  vector<IntersectingPoint, Eigen::aligned_allocator<IntersectingPoint>> &intersect_points);
    // ���ݱ���ж������߶��Ƿ�����
    bool isConnectTwoLineSegs(int size, int edge_id1, int edge_id2);

	// ������ܳ�����
	float getCircumferenceOfPoly(pcl::PointCloud<pcl::PointXYZ>::Ptr &border);

	// ��������뵽�������
	void insertIPVertexIntoPoly(Vertex *v, POLYGON &poly,const int edge_id, vector<Vertex*> &poly_vertices);

    // ���ö���������뾶
    void setSearchRadiusForEachVertex(vector<InitialPoly> &initial_polys);

    // һ��ִ�к���
    // ���������Լ�����׼������ǰ����
    void perform(vector<InitialPoly> &initial_polys);

	// �ھ��нǶԱ߹�ϵ���߶��ϲ��붥��
	void insertVertexOnEdgeWithAngleToEdgeRelation();
	void checkLineSegThatPassThroughVertex(int poly_id, Vertex *pv);
	bool isVertexOnEdge(Vertex *pv, Vertex *edge_begin);

    // ��鵱ǰģ���Ƿ�Ϸ�
    // �����⵽�Ƿ�������򷵻����ⶥ��
    bool is_model_valid(Vertex *&prob_v);
    // ���Ϊpoly_id�Ķ���εĶ���v�Ƿ�������������εı���
    bool is_vertex_on_edge(int poly_id, Vertex* v);
    // ���Ϊpoly_id�Ķ���εĶ���v�Ƿ�������������ε��ڲ�
    bool is_vertex_in_poly(int poly_id, Vertex* v);
    // ĳ���Ƿ����������ڲ�
    bool is_vertex_in_triangle(pcl::PointXYZ &point, Triangle &tri);
    // �߶�[e1, e2]�Ƿ񴩹���������ε��ڲ�
    bool is_edge_pass_through_poly(int poly_id, Vertex* e1, Vertex* e2);
};

#endif // SNAPPINGVERTICES_H
