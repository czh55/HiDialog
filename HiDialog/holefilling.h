#ifndef HOLEFILLING_H
#define HOLEFILLING_H

#include "snappingvertices.h"
#include <pcl\point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

struct POLYGON_RELEVANT
{
    void *poly;
	void *prev_point;
	void *next_point;
};

struct Vertex2
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Vector3d pos;		// ����λ��
	Eigen::Vector3d proj_pos;	// �ֲ�ͶӰ��λ��
	Eigen::Vector4d coeff;		// �ֲ�ͶӰƽ�淽��
	double state_angle;			// ״̬�Ƕ�
	vector<POLYGON_RELEVANT> relevant_polys;	// �����Ķ������Ϣ
    int index_in_m_vertices;
};

struct Triangle2
{
	Vertex2* v[3];
};

class POLYGON2
{
public:
    int size;

public:
	// http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vertex2 *start_point;        // ��������
    Vertex2 *cur_point;
    Eigen::Vector4d coeff;      // �����ƽ�淽��(a,b,c,d: ax+by+cz+d=0)

    vector<Triangle2> triangles; // �Զ���ν������ǻ�֮���С�����μ���
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> discontinuity_vertices;

	bool is_tri;	// �Ƿ���Ҫִ�����ǻ�
    bool is_discontinuity;  // ������Ƿ��ǲ�������

    POLYGON2(){
        start_point = cur_point = NULL;
        size = 0;
        coeff << 0, 0, 0, 0;
		is_tri = true;
        is_discontinuity = false;
    }

	~POLYGON2(){
        //clear();
	}

	// ��ն����
	void clear(){
        // ���ȴ�����Ĺ����������Ϣ
        Vertex2* pv2, *next_pv2;
        int poly_index_in_v;
        for(int i = 0; i < size; ++i){
            if(i == 0){
                pv2 = this->start_point;
            }else{
                pv2 = next_pv2;
            }
            poly_index_in_v = this->getPolyIndex(pv2);
            next_pv2 = (Vertex2*)pv2->relevant_polys[poly_index_in_v].next_point;

            auto iter = pv2->relevant_polys.begin();
            for(int j = 0; j <= poly_index_in_v; ++j){
                if(j == 0)
                    iter = pv2->relevant_polys.begin();
                else
                    ++iter;
            }
            pv2->relevant_polys.erase(iter);
        }

		size = 0;
		coeff << 0, 0, 0, 0;
		this->start_point = this->cur_point = NULL;
        triangles.clear();
        is_tri = true;
        discontinuity_vertices.clear();
	}

    int getSize(){return size;}

	int getPolyIndex(Vertex2 *v){
		int poly_index = -1;
		for(int i = 0; i < v->relevant_polys.size(); ++i){
			if(v->relevant_polys[i].poly == this){
				poly_index = i;
				break;
			}
		}
		return poly_index;
	}

    ///////////////////////////����ν����׶�-begin////////////////////////////////
    // �����һ������
    void insertFirstVertex2(Vertex2 *v){
        start_point = v;
        cur_point = v;
        ++size;

		POLYGON_RELEVANT pr;
		pr.poly = this;
		pr.next_point = pr.prev_point = NULL;
		v->relevant_polys.push_back(pr);
    }
    // �����м䶥��
    void insertMiddleVertex2(Vertex2 *v){
		int poly_index = getPolyIndex(cur_point);
		cur_point->relevant_polys[poly_index].next_point = v;
		POLYGON_RELEVANT pr;
		pr.poly = this;
		pr.prev_point = cur_point;
		pr.next_point = NULL;
		v->relevant_polys.push_back(pr);

        // ��v��Ϊ��ǰ����
        cur_point = v;
        ++size;
    }
    // �������һ������
    void insertLastVertex2(Vertex2 *v){
		int poly_index = getPolyIndex(cur_point);
		cur_point->relevant_polys[poly_index].next_point = v;
		POLYGON_RELEVANT pr;
		pr.poly = this;
		pr.prev_point = cur_point;
		pr.next_point = start_point;
		v->relevant_polys.push_back(pr);
		poly_index = getPolyIndex(start_point);
		start_point->relevant_polys[poly_index].prev_point = v;

        ++size;
    }
    ///////////////////////////����ν����׶�-end////////////////////////////////

    // ɾ������
    void delVertex2(Vertex2 *v){
        is_tri = true;
        if(isVertex2InPoly(v)){
            if(size == 3){
                cerr<<"��ǰ�����size == 3, �޷�ִ�ж���ɾ������." << endl;
                return;
            }

            --size;
			int poly_index = getPolyIndex(v);

            // ���v����ʼ���㣬�������ʼ����(����ʼ�������һ��������Ϊ��ʼ����)
            if(v == this->start_point){
				this->start_point = (Vertex2 *)v->relevant_polys[poly_index].next_point;
            }

            Vertex2* prev_v, * next_v;
			prev_v = (Vertex2 *)v->relevant_polys[poly_index].prev_point;
			next_v = (Vertex2 *)v->relevant_polys[poly_index].next_point;
			int prev_v_poly_index = getPolyIndex(prev_v);
			prev_v->relevant_polys[prev_v_poly_index].next_point = next_v;
			int next_v_poly_index = getPolyIndex(next_v);
			next_v->relevant_polys[next_v_poly_index].prev_point = prev_v;
            /*delete v;
            v = NULL;*/
			for(auto iter = v->relevant_polys.begin(); iter != v->relevant_polys.end(); ++iter){
				if(iter->poly == this){
					v->relevant_polys.erase(iter);
					break;
				}
            }
        }else{
            cout << "vertex is not a member of this Polygon" << endl;
        }
    }

    // �ڶ���v�������һ���µĶ���new_v
	// ǰ�᣺�ѽ����ö����
    bool insertVertex2(Vertex2 *v, Vertex2 *new_v){
        is_tri = true;
        // ������֤new_v�Ƿ�λ�ڶ���ε�֧��ƽ����
        double d =  new_v->pos[0]*coeff[0] +
                    new_v->pos[1]*coeff[1] +
                    new_v->pos[2]*coeff[2] +
                                  coeff[3];
        if(abs(d) > 0.00001){
            cerr << "new_v ��ƽ��ľ���d = " << d << endl;
            return false;
        }

        // ��ǰ������������ж������
        if(!isVertex2InPoly(v)){
            cerr << "vertex v is not in ply" << endl;
            return false;
        }

        // �¶���������Ѿ������ڵ�ǰ�������
        if(isVertex2InPoly(new_v)){
            cerr << "vertex new_v is in ply!" << endl;
            return false;
        }

        // �ҵ���ǰ�������һ������
        int ply_index_in_v = getPolyIndex(v);
        Vertex2* next_v = (Vertex2*)v->relevant_polys[ply_index_in_v].next_point;

        // ����ǰ����ĺ��ָ��new_v
        v->relevant_polys[ply_index_in_v].next_point = new_v;
        // ����ǰ��һ�������ǰ��ָ��new_v
        int ply_index_in_next_v = getPolyIndex(next_v);
        next_v->relevant_polys[ply_index_in_next_v].prev_point = new_v;

        // Ϊ�¶��������µĹ�����ϵ
        POLYGON_RELEVANT ply_rel;
		ply_rel.poly = this;
        ply_rel.prev_point = v;
        ply_rel.next_point = next_v;
        new_v->relevant_polys.push_back(ply_rel);

        ++size;
    }

    // ��ȡ��ǰ�����ǰ��
    Vertex2* getPrevVertex(Vertex2* v){
        int ply_index = this->getPolyIndex(v);
        if(ply_index != -1)
            return (Vertex2*)v->relevant_polys[ply_index].prev_point;
        else
            return NULL;
    }

    // ��ȡ��ǰ����ĺ��
    Vertex2* getNextVertex(Vertex2* v){
        int ply_index = this->getPolyIndex(v);
        if(ply_index != -1)
            return (Vertex2*)v->relevant_polys[ply_index].next_point;
        else
            return NULL;
    }

    // �ж϶����Ƿ��ڶ������
    bool isVertex2InPoly(Vertex2 *v){
        if(size == 0) return false;

        for(int i = 0; i < size; ++i){
            //cur_point = i == 0 ? start_point : cur_point->next_point;
			if(i == 0)
				cur_point = start_point;
			else{
				int poly_index = getPolyIndex(cur_point);
				cur_point = (Vertex2 *)cur_point->relevant_polys[poly_index].next_point;
			}

            if(cur_point == v){
                return true;
            }
        }
        return false;
    }
};


class HoleFilling
{
public:
    HoleFilling();

	// �ⲿ�Ķ������Ϣ
	vector<POLYGON, Eigen::aligned_allocator<POLYGON>> *sv_polygons;

	// ȫ�ֶ�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_vertices_cloud;
	pcl::KdTreeFLANN<pcl::PointXYZ> m_vertices_kdtree;
	vector<Vertex2*> m_vertices;
	// ����μ���
	vector<POLYGON2*> m_polygons;

    // ����ȷ�������id
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_for_indentify_poly_id;
    pcl::KdTreeFLANN<pcl::PointXYZ> m_cloud_for_indentify_poly_id_kdtree;

	// �������ݽṹ
	void clear();

	// step1: �Զ�������ݽ��и�ʽת��
	void setInputPolygons(vector<POLYGON, Eigen::aligned_allocator<POLYGON>> *p_polygons);

    // step2: ����m_cloud_for_indentify_poly_id
    void set_cloud_for_indentify_poly_id();

	// ���¶�������(�������Ӧ�Ķ������ͬ�����£�
	void updateVertices();

    // ��ȡһ�����㵥��ƥ���ϵ������
    int getSingleRelationCount(Vertex2* v);

	// ���ڶ���ε����ǻ��ʷֽ������������Ƭ���ĵ���
	void updateTriCentroids();

	// ���¶������(������ɾ��û�й�����ϵ�Ķ��㣬���¶�����ƺͶ�Ӧ��kdtree��
	void updateVerCloud();
};

#endif // HOLEFILLING_H
