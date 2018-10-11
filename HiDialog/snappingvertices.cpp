#include "snappingvertices.h"
#include "earclip.h"
#include <queue>
#include <set>
#include <pcl/kdtree/kdtree_flann.h>
/*
 SnappingVerticesʹ�÷�����
    ��һ����
    setParams
    �ڶ�����
    // ���µ����ݽṹ��������
    this->m_sv.setPolygons(this->initial_polys);
    ��������
    // Ϊ����ε�ÿ���������������ϵ
    this->m_sv.setNeighborsForEachPoly();
    ���Ĳ���
    // �������ƥ���ϵת��Ϊ˫���ƥ���ϵ
    m_sv.setMatchingRelations();
    ���岽��ƥ����Ϊһ����ͨ��ϵ����������ƥ���ϵ�Ķ�������һ��
    // ��ȡ����ƥ���ϵ�Ķ����
    m_sv.getMatchingVerticesClusters(clusters);
    �������������е�-��ƥ��Ķ���ƥ���ϵ���Ƿ��б�ĵ�����ƥ��ת��Ϊ��-��ƥ��
    // ����ƥ���ϵ
    refineMachingRelations
    ���߲���(����λ�÷����ı�)
    // ����ÿ�����С���-�㡱ƥ���ϵ�Ķ����
    mergeMatchingVerticesClusters
    �ڰ˲���������-�ߡ�ƥ��
    dealWithVertexEdgeMatchingRelation()
    �ھŲ����������ε��˻����
    dealWithPolysDegeneration()
*/

SnappingVertices::SnappingVertices()
{
}

void SnappingVertices::setParams(float &T1, float &T2, float &T3, float &T4){
    ratio_of_scale = T1;
    T_ratio_lineSeg_middle_area = T2;
    T_dist_proj_to_lineSegEnd = T3;
    T_maxAngle_bet_two_polys = T4;
    float PI = 3.14159265358979;
    // �Ƕ�ת����
    T_maxAngle_bet_two_polys = T_maxAngle_bet_two_polys * PI / 180.0;
}

// �������μ���
void SnappingVertices::setPolygons(vector<InitialPoly> &initial_polys){
    this->polygons.clear();

    for(int i = 0; i < initial_polys.size(); ++i){
        POLYGON poly;
        for(int j = 0; j < initial_polys[i].simplified_vertices->size(); ++j){
            Vertex *p_vertex = new Vertex();
            p_vertex->is_updated = false;
            p_vertex->radius = ratio_of_scale*initial_polys[i].scale;   // ���ö���������뾶
            // ��i������εĵ�j������
            p_vertex->point = initial_polys[i].simplified_vertices->points[j];
            p_vertex->point.data[3] = i;    // data[3]�洢�õ��Ӧ�Ķ���ε�����
            p_vertex->next_point = p_vertex->prev_point = NULL;

            p_vertex->neighbor_poly_id = -1;    // �ھӶ���α��Ϊ-1����û��ƥ���ϵ
            p_vertex->neighbor_vertex = p_vertex->neighbor_edge = NULL;

            // ���
            if(j == 0){
                poly.insertFirstVertex(p_vertex);
            }else if(j == initial_polys[i].simplified_vertices->size() - 1){
            // �յ�
                poly.insertLastVertex(p_vertex);
            }else{
            // �м�ĵ�
                poly.insertMiddleVertex(p_vertex);
            }
        }

        // �����ƽ�����
        pcl::PointXYZ *p;
        InitialPoly *pPoly = &initial_polys[i];
        p = &initial_polys[i].simplified_vertices->points[0];
        float d = -1 * (p->x*pPoly->normal[0] + p->y*pPoly->normal[1] +
                        p->z*pPoly->normal[2]);
        poly.coeff << pPoly->normal[0],
                      pPoly->normal[1],
                      pPoly->normal[2],
                      d;

        polygons.push_back(poly);
    }
}

// Ѱ�Ҷ����P��һ������v������������������
void SnappingVertices::getNeighborPolygon(int poly_id/*P*/, Vertex* v){
    float dist = FLT_MAX;

    for(int i = 0; i < polygons.size(); ++i){
        if(i == poly_id) continue;

        // ���ȿ����㣬������v��������Ķ���
        float vertex_dist = FLT_MAX;
        Vertex *neighbor_v = NULL; // ������v��������Ķ���
        Vertex *cur_v;
        for(int j = 0; j < polygons[i].getSize(); ++j){ // ������i������ε����ж���
            cur_v = j == 0 ? polygons[i].start_point : cur_v->next_point;
            float d_ = distP2P(v->point, cur_v->point);
            if(d_ < vertex_dist){
                vertex_dist = d_;
                neighbor_v = cur_v;
            }
        }

        // Ȼ�󿴶������߶�
        float edge_dist = FLT_MAX;
        Vertex *neighbor_e = NULL; // ������v����������߶�
        Vertex *cur_e;
        for(int j = 0; j < polygons[i].getSize(); ++j){ // ������i������ε����б�
            cur_e = j == 0 ? polygons[i].start_point : cur_e->next_point;
            float d_ = distP2E(v->point, cur_e->point, cur_e->next_point->point);
            if(d_ < edge_dist){
                edge_dist = d_;
                neighbor_e = cur_e;
            }
        }

        float dist_ = vertex_dist <= edge_dist ? vertex_dist : edge_dist;

        if(dist <= dist_) continue; // ���������������ȵ�ǰ�����С��������һ�������

        dist = dist_;               // ���򣬸��µ�ǰ����
        v->neighbor_poly_id = i;    // �����¾�������Ķ���α��

        // ��v���������Ƕ���
        if(neighbor_e == NULL || vertex_dist <= edge_dist){
            v->is_vertex_neighbor = true;
            v->neighbor_vertex = neighbor_v;
			v->neighbor_edge = NULL;
        }else{ // ��v�����������߶�
            v->is_vertex_neighbor = false;
            v->neighbor_edge = neighbor_e;
			v->neighbor_vertex = NULL;
		}
    }

    // �����ǰ���볬���˶���v�������뾶���򶥵�vû���κ�ƥ��
    if(dist > v->radius){
        v->neighbor_edge = v->neighbor_vertex = NULL;
        v->neighbor_poly_id = -1;
        return;
    }

    // ��ǰ����û�г�������v�������뾶
    // ƥ���ϵΪ��-��
    if(v->is_vertex_neighbor){
        if(dist > v->neighbor_vertex->radius){  // ����ͬ�����ܴ���ƥ�䶥��������뾶
            v->neighbor_edge = v->neighbor_vertex = NULL;
            v->neighbor_poly_id = -1;
            return;
        }
        return;
    }

    // ƥ���ϵΪ��-�ߣ����Ҫ��΢����һЩ����ѭ�Ļ���ԭ���ܾ���ƥ���Ͳ�ƥ���
    pcl::PointXYZ e_begin/*�߶����*/, e_end/*�߶��յ�*/, e_middle/*�߶��м�ĵ�*/;
    e_begin = v->neighbor_edge->point;
    e_end = v->neighbor_edge->next_point->point;
    e_middle.x = (e_begin.x + e_end.x)/2.0;
    e_middle.y = (e_begin.y + e_end.y)/2.0;
    e_middle.z = (e_begin.z + e_end.z)/2.0;

    pcl::PointXYZ p_proj;   // v��e�ϵ�ͶӰ��
    getProjPointOnLineSeg(v->point, p_proj, e_begin, e_end);

    float lineSeg_len = distP2P(e_begin, e_end);
    float half_lineSeg_len = lineSeg_len * 0.5;

    pcl::PointXYZ e_nearer = distP2P(p_proj, e_begin) <= distP2P(p_proj, e_end) ? e_begin : e_end;

    if(distP2P(p_proj, e_middle) <= T_ratio_lineSeg_middle_area * half_lineSeg_len){    // ͶӰ�������߶��в�����
        // ��-��ƥ��
        if(dist > v->neighbor_edge->radius){    // �㵽�ߵľ���ͬ�����ܳ����ߵ������뾶
            v->neighbor_poly_id = -1;
            v->neighbor_edge = v->neighbor_vertex = NULL;
        }
        return;
    }else if(distP2P(p_proj, e_nearer) <= T_dist_proj_to_lineSegEnd &&
             distP2P(e_nearer, v->point) <= v->radius &&
             distP2P(e_nearer, v->point) <= v->neighbor_edge->radius){
        // ��-��ƥ��
        v->is_vertex_neighbor = true;
        v->neighbor_vertex = isEqual(e_nearer, e_begin) ? v->neighbor_edge : v->neighbor_edge->next_point;
        v->neighbor_edge = NULL;
    }else{
        // ��-��ƥ��
        if(dist > v->neighbor_edge->radius){    // �㵽�ߵľ���ͬ�����ܳ����ߵ������뾶
            v->neighbor_poly_id = -1;
            v->neighbor_edge = v->neighbor_vertex = NULL;
        }
        return;
    }
}

// Ϊ����ε�ÿ���������������ϵ
void SnappingVertices::setNeighborsForEachPoly(){
    for(int i = 0; i < polygons.size(); ++i){
        POLYGON *pPoly = &polygons[i];
        Vertex* pv;
        for(int j = 0; j < pPoly->getSize(); ++j){
            pv = j == 0 ? pPoly->start_point : pv->next_point;
            getNeighborPolygon(i, pv);
        }
    }
}

// ���뺯��
// ��-��
float SnappingVertices::distP2P(pcl::PointXYZ &p1, pcl::PointXYZ &p2){
    return pow((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z), 0.5f);
}

// ��-�߾���
// �����p��ͶӰ�������߶��ڲ����򷵻�p��ͶӰ��ľ��룻���򷵻�FLT_MAX
float SnappingVertices::distP2E(pcl::PointXYZ &p, pcl::PointXYZ &e1, pcl::PointXYZ &e2){
    Eigen::Vector3f p_base, line_dir;
    p_base << e1.x, e1.y, e1.z;
    line_dir << e2.x - e1.x,
                e2.y - e1.y,
                e2.z - e1.z;
    line_dir.normalize();

    Eigen::Vector3f v_p, delta;
    v_p << p.x, p.y, p.z;
    delta = p_base - v_p;
    float lambda = -1.0 * line_dir.dot(delta);

    Eigen::Vector3f p_proj;
    p_proj = p_base + lambda * line_dir;

    Eigen::Vector3f lambda1/*������e1e2*/, lambda2/*������e1p_proj*/;
    lambda1 << e2.x - e1.x,
               e2.y - e1.y,
               e2.z - e1.z;
    lambda2 << p_proj - p_base;

    bool is_in_lineSeg;
    if(lambda1.dot(lambda2) < 0){
        is_in_lineSeg = false;
    }else{
        if(lambda2.norm()/lambda1.norm() >= 1.0){
            is_in_lineSeg = false;
        }else{
            is_in_lineSeg = true;
        }
    }

    if(is_in_lineSeg){
        return (v_p-p_proj).norm();
    }

    return FLT_MAX;
}

// ��ȡ�����߶��ϵ�ͶӰ��
void SnappingVertices::getProjPointOnLineSeg(pcl::PointXYZ &p, pcl::PointXYZ &p_proj, pcl::PointXYZ &e1, pcl::PointXYZ &e2){
    Eigen::Vector3f p_base, line_dir;
    p_base << e1.x, e1.y, e1.z;
    line_dir << e2.x - e1.x,
                e2.y - e1.y,
                e2.z - e1.z;
    line_dir.normalize();

    Eigen::Vector3f v_p, delta;
    v_p << p.x, p.y, p.z;
    delta = p_base - v_p;
    float lambda = -1.0 * line_dir.dot(delta);

    Eigen::Vector3f p_proj_v;
    p_proj_v = p_base + lambda * line_dir;
    p_proj.x = p_proj_v[0];
    p_proj.y = p_proj_v[1];
    p_proj.z = p_proj_v[2];
}

// �ڻ�ȡ�ھӶ���ε�ʱ��Ϊÿ�����㹹���˵����ƥ���ϵ
// �ú���������˫���ƥ���ϵ
void SnappingVertices::setMatchingRelations(){
    for(int i = 0; i < polygons.size(); ++i){
        POLYGON *pPoly = &polygons[i];
        Vertex *pv;
        for(int j = 0; j < pPoly->getSize(); ++j){
            // pv: ��ǰ����
            pv = j == 0 ? pPoly->start_point : pv->next_point;
            // ����ǡ���-�ߡ�ƥ���û��ƥ��������һ������
            if(!pv->is_vertex_neighbor || pv->neighbor_poly_id == -1) continue;

            // ����-�㡱ƥ��
            if(pv->neighbor_poly_id == -1){// �쳣����
                std::cerr << "pv->neighbor_poly_id == -1 in SnappingVertices::setMatchingRelations()" << std::endl;
                continue;
            }

            // ƥ����Ӧ�Ķ�����Ƿ��Ѵ�����v��ƥ��������
            int poly_id_in_v;
            if(findPoly(pv, pv->neighbor_poly_id, poly_id_in_v)){ // �������δ���
                // ƥ����Ƿ��Ѵ���
                if(findVertex(pv, poly_id_in_v, pv->neighbor_vertex)){ // ����������
                    // do nothing
                }else{ // ������㲻����
                    pv->maching_polys[poly_id_in_v].vertices.push_back((void *)pv->neighbor_vertex);
                }
            }else{ // ����β�����
                MatchingPoly mp;
                mp.poly_id = pv->neighbor_poly_id;
                mp.vertices.push_back((void *)pv->neighbor_vertex);
                pv->maching_polys.push_back(mp);
            }

            // ��ƥ����ƥ���ϵ������v
            int poly_id_in_neighbor;
            if(findPoly(pv->neighbor_vertex, i, poly_id_in_neighbor)){
                if(findVertex(pv->neighbor_vertex, poly_id_in_neighbor, pv)){
                }else{
                    pv->neighbor_vertex->maching_polys[poly_id_in_neighbor].vertices.push_back((void*)pv);
                }
            }else{
                MatchingPoly mp;
                mp.poly_id = i;
                mp.vertices.push_back((void*) pv);
                pv->neighbor_vertex->maching_polys.push_back(mp);
            }
        }
    }
}

// �ڶ���v��ƥ�������м���Ƿ���ڱ��Ϊid�Ķ���Σ��������������λ��
bool SnappingVertices::findPoly(Vertex *v, int poly_id, int &poly_id_in_v){
    for(int i = 0; i < v->maching_polys.size(); ++i){
        if(v->maching_polys[i].poly_id == poly_id){
            poly_id_in_v = i;
            return true;
        }
    }
    poly_id_in_v = -1;
    return false;
}

// ƥ����Ƿ����
bool SnappingVertices::findVertex(Vertex *v, int poly_id_in_v, Vertex *neighbor_v){
    for(int i = 0; i < v->maching_polys[poly_id_in_v].vertices.size(); ++i){
        if((Vertex *)(v->maching_polys[poly_id_in_v].vertices[i]) == neighbor_v)
            return true;
    }
    return false;
}

// ����ƥ���ϵ����Ҫ�������������
// ����v���С���-�ߡ�ƥ���ϵ��ͬʱ��Ķ���v_��v�־���"��-�㡱ƥ��
// ������ԣ����ž�����ʵ�ֵ�-��ƥ���ԭ��ȥ��v�ġ���-�ߡ�ƥ��
void SnappingVertices::refineMachingRelations(){
    POLYGON *pPoly;
    Vertex *pv;
    for(int i = 0; i < polygons.size(); ++i){
        pPoly = &polygons[i];
        for(int j = 0; j < pPoly->getSize(); ++j){
            pv = j == 0 ? pPoly->start_point : pv->next_point;
            // ����ǡ���-�ߡ�ƥ��
            if(!pv->is_vertex_neighbor && pv->neighbor_poly_id != -1){
                // ��pv�Ƿ��С���-�㡱ƥ���ϵ
                if(pv->maching_polys.size() > 0){
                    pv->neighbor_edge = NULL;
                    pv->neighbor_poly_id = pv->maching_polys[0].poly_id;
                    pv->neighbor_vertex = (Vertex *)pv->maching_polys[0].vertices[0];
                    pv->is_vertex_neighbor = true;
                }
            }
        }
    }
}

// ��ȡƥ��Ķ����
void SnappingVertices::getMatchingVerticesClusters(vector<vector<Vertex*>> &clusters)
{
    // �������ݽṹ
    if(clusters.size() > 0){
        for(int i = 0; i < clusters.size(); ++i){
            clusters[i].clear();
        }
    }
    clusters.clear();

    // ����ÿ������ı�����״̬(is_updated)
    POLYGON *pPoly;
    Vertex *pv;
    for(int i = 0; i < this->polygons.size(); ++i){
        pPoly = &this->polygons[i];
        for(int j = 0; j < pPoly->getSize(); ++j){
            pv = j == 0 ? pPoly->start_point : pv->next_point;
            pv->is_updated = false;
        }
    }

    // Ѱ�Ҵ�
    for(int i = 0; i < this->polygons.size(); ++i){
        pPoly = &this->polygons[i];
        for(int j = 0; j < pPoly->getSize(); ++j){
            pv = j == 0 ? pPoly->start_point : pv->next_point;

            if(pv->is_updated) continue;

            if(pv->maching_polys.size() == 0){
                pv->is_updated = true;
                continue;
            }

            // pv->maching_polys.size() > 0
            vector<Vertex*> cluster;
            std::queue<Vertex*> Q;
            Q.push(pv);
            while(!Q.empty()){
                Vertex *cur_v = Q.front();
                Q.pop();
                cluster.push_back(cur_v);
                cur_v->is_updated = true;

                Vertex *pv2;
                for(int k = 0; k < cur_v->maching_polys.size(); ++k){
                    for(int l = 0; l < cur_v->maching_polys[k].vertices.size(); ++l){
                        pv2 = (Vertex *)cur_v->maching_polys[k].vertices[l];
                        if(pv2->is_updated) continue;
                        Q.push(pv2);
                    }
                }
            }
            clusters.push_back(cluster);
        }
    }
}

// �ϲ�����ƥ���ϵ�ĵ㼯
void SnappingVertices::mergeMatchingVerticesClusters(vector<vector<Vertex *> > &clusters){
    // ���ö���ĸ���״̬
    POLYGON *pPoly;
    Vertex *pv;
    for(int i = 0; i < polygons.size(); ++i){
        pPoly = &polygons[i];
        for(int j = 0; j < pPoly->getSize(); ++j){
            pv = j == 0 ? pPoly->start_point : pv->next_point;
            pv->is_updated = false;
        }
    }

    for(int i = 0; i < clusters.size(); ++i){
        dealWithEachCluster(clusters[i]);
    }
}
// mergeMatchingVerticesClusters�Ӻ���
void SnappingVertices::dealWithEachCluster(vector<Vertex*> &cluster){
    // �������Ķ��������
    set<int> polys_set;
    for(int i = 0; i < cluster.size(); ++i){
        int poly_id = floor(cluster[i]->point.data[3]+0.5f);
        polys_set.insert(poly_id);
    }

    // �����Ķ�����������ó���3��Ҳ���ܵ���2�������ִ������Ҫ����
    if(polys_set.size() > 3 || polys_set.size() < 2){
        cerr << "associative polys' number exceeds 3 or less than 2!" << endl;
        cerr << "polys_set.size() = " << polys_set.size() << endl;
        for(auto iter = polys_set.begin(); iter != polys_set.end(); ++iter){
            cout << *iter << "; ";
        }
        cout << endl;
        return;
    }

    // �����Ķ������2��
    if(polys_set.size() == 2){
        // ��ȡ�����id
        int poly_a, poly_b;
        auto iter = polys_set.begin();
        poly_a = *iter;
        ++iter;
        poly_b = *iter;

        // ������������εĽ���
        Eigen::Vector3f p_base, line_dir;
        getIntersectLine(polygons[poly_a].coeff, polygons[poly_b].coeff, p_base, line_dir);
        // ������Щ�����ڽ����ϵ�ͶӰ
        Vertex*pv;
        pcl::PointXYZ p_proj, p_proj_mean;
        p_proj_mean.x = p_proj_mean.y = p_proj_mean.z = 0;
        for(int i = 0; i < cluster.size(); ++i){
            pv = cluster[i];
            getProjPointOnLine(pv->point, p_proj, p_base, line_dir);
            p_proj_mean.x += p_proj.x;
            p_proj_mean.y += p_proj.y;
            p_proj_mean.z += p_proj.z;
        }
        // ����ͶӰ�������
        p_proj_mean.x /= cluster.size();
        p_proj_mean.y /= cluster.size();
        p_proj_mean.z /= cluster.size();

        // ���¶���λ��
        for(int i = 0; i < cluster.size(); ++i){
            pv = cluster[i];
            if(distP2P(pv->point, p_proj_mean) <= pv->radius){
                float pv_poly_index = pv->point.data[3];    // pv���������id
                pv->point = p_proj_mean;
                pv->point.data[3] = pv_poly_index;          // ����pv��data3����
                pv->is_updated = true;
            }
        }
        return;
    }

    // �����Ķ������3��
    // ��ȡ�����id
    vector<int> poly_indices;
    for(auto iter = polys_set.begin(); iter != polys_set.end(); ++iter) poly_indices.push_back(*iter);
    pcl::PointXYZ p_intersect;
    // ���㽻��
    getIntersectPointOfTriplePolys(polygons[poly_indices[0]].coeff,
            polygons[poly_indices[1]].coeff,
            polygons[poly_indices[2]].coeff,
            p_intersect);
    // ���Ը���λ��
    bool are_all_dists_within_radius = true;
    Vertex *pv;
    for(int i = 0; i < cluster.size(); ++i){
        pv = cluster[i];
        if(distP2P(pv->point, p_intersect) > pv->radius) {
            are_all_dists_within_radius = false;
            break;
        }
    }
    if(are_all_dists_within_radius){
        for(int i = 0; i < cluster.size(); ++i) {
            pv = cluster[i];
            float pv_poly_index = pv->point.data[3];    // pv���������id
            pv->point = p_intersect;
            pv->point.data[3] = pv_poly_index;          // ����pv��data[3]����
            pv->is_updated = true;
        }
        return;
    }

    // ���е�����˵������ƽ��Ľ��㲻���滻ȫ���Ķ���
    // ������������η������������н�
    float n01, n02, n12;
    n01 = getIncludedAngleOfTwoPolys(poly_indices[0], poly_indices[1]);
    n02 = getIncludedAngleOfTwoPolys(poly_indices[0], poly_indices[2]);
    n12 = getIncludedAngleOfTwoPolys(poly_indices[1], poly_indices[2]);
    int i0, i1, i2;// i0��i1������С�нǵĶ����������i2���������Ǿ��ɽϴ�нǵĶ��������
    float min_angle = FLT_MAX, angle1, angle2;// angle1��angle2�������С�������н�
    if(min_angle > n01){    // n01����С�ļн�
        min_angle = n01;
        angle1 = n02;
        angle2 = n12;
        i0 = poly_indices[0];
        i1 = poly_indices[1];
        i2 = poly_indices[2];
    }
    if(min_angle > n02){    // n02����С�ļн�
        min_angle = n02;
        angle1 = n01;
        angle2 = n12;
        i0 = poly_indices[0];
        i1 = poly_indices[2];
        i2 = poly_indices[1];
    }
    if(min_angle > n12){    // n12����С�ļн�
        min_angle = n12;
        angle1 = n01;
        angle2 = n02;
        i0 = poly_indices[1];
        i1 = poly_indices[2];
        i2 = poly_indices[0];
    }

    // �ֱ���cluster1��cluster2
    vector<Vertex*> cluster1;   // i0��ȫ������+��i0����������i2�еĶ���
    vector<Vertex*> cluster2;   // i1��ȫ������+��i1����������i2�еĶ���
    for(int i = 0; i < cluster.size(); ++i){
        int poly_id = floor(cluster[i]->point.data[3]+0.5f);
        if(poly_id == i0)   // i0�еĶ���
            cluster1.push_back(cluster[i]);
        else if(poly_id == i1)  // i1�еĶ���
            cluster2.push_back(cluster[i]);
        else{                   // i2�еĶ���
            // Ѱ���뵱ǰ��������Ķ����
            float min_dist = FLT_MAX;
            int index = -1; // �����������Ķ��������
            for(int j = 0; j < cluster[i]->maching_polys.size(); ++j){
                for(int k = 0; k < cluster[i]->maching_polys[j].vertices.size(); ++k){
                    pv = (Vertex*)(cluster[i]->maching_polys[j].vertices[k]);
                    if(min_dist > distP2P(cluster[i]->point, pv->point)){
                        min_dist = distP2P(cluster[i]->point, pv->point);
                        index = j;
                    }
                }
            }
            if(index = i0){
                cluster1.push_back(cluster[i]);
            }else{
                cluster2.push_back(cluster[i]);
            }
        }
    }// ����cluster1��cluster2����

    // ����cluster1:
    dealWithEachCluster2(cluster1);
    // ����cluster2:
    dealWithEachCluster2(cluster2);
}

// ������ֱ�������������εĶ����
void SnappingVertices::dealWithEachCluster2(vector<Vertex*> &cluster)
{
    // �������Ķ��������
    set<int> polys_set;
    for(int i = 0; i < cluster.size(); ++i){
        int poly_id = floor(cluster[i]->point.data[3]+0.5f);
        polys_set.insert(poly_id);
    }
    if(polys_set.size() == 1){
        return;
    }
    if(polys_set.size() == 2){
        // ��ȡ�����id
        int poly_a, poly_b;
        auto iter = polys_set.begin();
        poly_a = *iter;
        ++iter;
        poly_b = *iter;

        // ������������εĽ���
        Eigen::Vector3f p_base, line_dir;
        getIntersectLine(polygons[poly_a].coeff, polygons[poly_b].coeff, p_base, line_dir);
        // ������Щ�����ڽ����ϵ�ͶӰ
        Vertex*pv;
        pcl::PointXYZ p_proj, p_proj_mean;
        p_proj_mean.x = p_proj_mean.y = p_proj_mean.z = 0;
        for(int i = 0; i < cluster.size(); ++i){
            pv = cluster[i];
            getProjPointOnLine(pv->point, p_proj, p_base, line_dir);
            p_proj_mean.x += p_proj.x;
            p_proj_mean.y += p_proj.y;
            p_proj_mean.z += p_proj.z;
        }
        // ����ͶӰ�������
        p_proj_mean.x /= cluster.size();
        p_proj_mean.y /= cluster.size();
        p_proj_mean.z /= cluster.size();

        // ���¶���λ��
        for(int i = 0; i < cluster.size(); ++i){
            pv = cluster[i];
            if(distP2P(pv->point, p_proj_mean) <= pv->radius){
                float pv_poly_index = pv->point.data[3];    // pv���������id
                pv->point = p_proj_mean;
                pv->point.data[3] = pv_poly_index;          // ����pv��data[3]����
                pv->is_updated = true;
            }
        }
    }else{
        cerr << "SnappingVertices::dealWithEachCluster2(vector<Vertex*> &cluster):" << "error use!" << endl;
    }
}

// ������������εĽ���
void SnappingVertices::getIntersectPointOfTriplePolys(Eigen::Vector4f &coeff1,
                                                      Eigen::Vector4f &coeff2,
                                                      Eigen::Vector4f &coeff3,
                                                      pcl::PointXYZ &p)
{
    Eigen::Matrix3f A;
    Eigen::Vector3f x, b;
    A << coeff1[0], coeff1[1], coeff1[2],
         coeff2[0], coeff2[1], coeff2[2],
         coeff3[0], coeff3[1], coeff3[2];
    b << -1*coeff1[3], -1 *coeff2[3], -1*coeff3[3];
    x = A.colPivHouseholderQr().solve(b);
    p.x = x[0];
    p.y = x[1];
    p.z = x[2];
}

// ������������εĽ���
bool SnappingVertices::getIntersectLine(Eigen::Vector4f &coeff1, Eigen::Vector4f &coeff2,
                                        Eigen::Vector3f &p_base, Eigen::Vector3f &line_dir)
{
    Eigen::Vector3f n1, n2;
    n1 << coeff1[0], coeff1[1], coeff1[2];
    n2 << coeff2[0], coeff2[1], coeff2[2];

    // ���������ƽ��
    if(abs(n1.dot(n2)) >= 0.99999){
        p_base << 0, 0, 0;
        line_dir << 0, 0, 0;
        return false;
    }

    // �������������Ĳ�˻�ȡ���ߵķ�������
    line_dir = n1.cross(n2);
    line_dir.normalize();

    // ��ϵ������ֵ��С������Ӧ�ı���ֵ��Ϊ0
    int index = 0;
    float min = coeff1[0];
    if(abs(min) > abs(coeff1[1]))
    {
        min = coeff1[1];
        index = 1;
    }
    if(abs(min) > abs(coeff1[2]))
    {
        min = coeff1[2];
        index = 2;
    }
    Eigen::Matrix2d A;
    Eigen::Vector2d x, b;
    p_base[index] = 0;
    switch(index)
    {
    case 0:
        A << coeff1[1], coeff1[2],
                coeff2[1], coeff2[2];
        b << coeff1[3], coeff2[3]; b *= -1;
        x = A.inverse() * b;
        p_base[1] = x[0];
        p_base[2] = x[1];
        break;
    case 1:
        A << coeff1[0], coeff1[2],
                coeff2[0], coeff2[2];
        b << coeff1[3], coeff2[3]; b *= -1;
        x = A.inverse() * b;
        p_base[0] = x[0];
        p_base[2] = x[1];
        break;
    case 2:
        A << coeff1[0], coeff1[1],
                coeff2[0], coeff2[1];
        b << coeff1[3], coeff2[3]; b *= -1;
        x = A.inverse() * b;
        p_base[0] = x[0];
        p_base[1] = x[1];
        break;
    }
    return true;
}

// ��ȡ����ֱ���ϵ�ͶӰ��
void SnappingVertices::getProjPointOnLine(pcl::PointXYZ &p, pcl::PointXYZ &p_proj,
                        Eigen::Vector3f &p_base, Eigen::Vector3f &line_dir){
    Eigen::Vector3f v_p, delta;
    v_p << p.x, p.y, p.z;
    delta = p_base - v_p;
    float lambda = -1.0 * line_dir.dot(delta);

    Eigen::Vector3f p_proj_v;
    p_proj_v = p_base + lambda * line_dir;
    p_proj.x = p_proj_v[0];
    p_proj.y = p_proj_v[1];
    p_proj.z = p_proj_v[2];
}

// ����ƽ�淨�����ļн�
float SnappingVertices::getIncludedAngleOfTwoPolys(int poly_a, int poly_b){
    Eigen::Vector3f n1, n2;
    n1 << polygons[poly_a].coeff[0],
          polygons[poly_a].coeff[1],
          polygons[poly_a].coeff[2];
    n2 << polygons[poly_b].coeff[0],
          polygons[poly_b].coeff[1],
          polygons[poly_b].coeff[2];
    float cos_val = n1.dot(n2);
    float rad_angle = acos(cos_val);
    return rad_angle * 180.0 / 3.141592653589793;
}

// �ж�����������Ƿ�ƽ��
bool SnappingVertices::areTwoPolysParallel(int poly_a, int poly_b){
    // ������������η������ļн�
    Eigen::Vector3f normal_a, normal_b;
    POLYGON *pPoly = &this->polygons[poly_a];
    normal_a << pPoly->coeff[0], pPoly->coeff[1], pPoly->coeff[2];
    pPoly = &this->polygons[poly_b];
    normal_b << pPoly->coeff[0], pPoly->coeff[1], pPoly->coeff[2];

    float cos_val = normal_a.dot(normal_b);

    cout << "angle:" << acos(cos_val)/3.1415927*180.0 << endl;

    cos_val = abs(cos_val);

    float T_cos_val = cos(T_maxAngle_bet_two_polys);

    return cos_val >= T_cos_val;
}

// ������-�ߡ�ƥ��
void SnappingVertices::dealWithVertexEdgeMatchingRelation()
{
    POLYGON *pPoly;
    Vertex *pv;
    for(int i = 0; i < polygons.size(); ++i){
        pPoly = &polygons[i];
        for(int j = 0; j < pPoly->getSize(); ++j){
            pv = j == 0 ? pPoly->start_point : pv->next_point;

            if(pv->neighbor_poly_id != -1 && !pv->is_vertex_neighbor){// ����-��ƥ�䡱
                // ����L��p_base, line_dir
                Eigen::Vector3f p_base, line_dir;
                int poly_a, poly_b;
                poly_a = i;
                poly_b = pv->neighbor_poly_id;
                this->getIntersectLine(polygons[poly_a].coeff, polygons[poly_b].coeff, p_base, line_dir);
                // pv��L�ϵ�ͶӰ��
                pcl::PointXYZ p_proj;
                this->getProjPointOnLine(pv->point, p_proj, p_base, line_dir);
                // �ж�pv��ͶӰ��ľ���
                if(distP2P(pv->point, p_proj) <= pv->radius &&
                        distP2P(pv->point, p_proj) <= pv->neighbor_edge->radius){
                    float pv_poly_index = pv->point.data[3];    // pv���������id
                    pv->point = p_proj; // ����pv��λ��
                    pv->point.data[3] = pv_poly_index;          // ����pv��data[3]����
                    pv->is_updated = true;
                    // ���Ӧ�����һ������
                    Vertex* new_v = new Vertex();
                    new_v->point = p_proj;
                    new_v->point.data[3] = poly_b;
                    new_v->radius = pv->neighbor_edge->radius;
                    new_v->neighbor_poly_id = -1;
                    new_v->neighbor_vertex = new_v->neighbor_edge = NULL;
                    if(!polygons[poly_b].insertVertex(pv->neighbor_edge, new_v)){
                        cerr << "�������ʧ�ܣ�" << endl;
                    }
                }
            }
        }
    }
}

// ���������˻����
// ���庯��
void SnappingVertices::dealWithPolysDegeneration(vector<POLYGON, Eigen::aligned_allocator<POLYGON>> &polygons)
{
    for(int i = 0; i < polygons.size(); ++i){
        dealWithSinglePolyDegeneration(polygons[i]);
    }
}
// ��������εĴ�����
void SnappingVertices::dealWithSinglePolyDegeneration(POLYGON &poly)
{
    // ���춥�����
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices (new pcl::PointCloud<pcl::PointXYZ>);
    Vertex *pv;
    vertices->reserve(poly.getSize());
    for(int i = 0; i < poly.getSize(); ++i){
        pv = i == 0 ? poly.start_point : pv->next_point;
        vertices->push_back(pv->point);
    }

	// ע��vertices�еĶ�����poly�еĶ����������һ��һ�Ĺ�ϵ�������θ��������ֹ�ϵҲ�ǵ��Ա��ֵ�

    // ���ȴ���������
    dealWithVerticesRedundancy(poly, vertices);
    // Ȼ�����߶��۷�
    dealWithLineSegCollineation(poly, vertices);
    // ����������߶ε��ཻ����
    dealWithLineSegIntersect(poly, vertices);
    // ����ÿ�������ƥ���ϵ
    cleanMatchingRelaion();
}

// ����ÿ�������ƥ���ϵ
void SnappingVertices::cleanMatchingRelaion()
{
    for(int i = 0; i < polygons.size(); ++i){
        POLYGON *pPoly = &polygons[i];
        Vertex*pv;
        for(int j = 0; j < pPoly->getSize(); ++j){
            pv = j == 0 ? pPoly->start_point : pv->next_point;
            pv->neighbor_poly_id = -1;
            pv->neighbor_edge = pv->neighbor_vertex = NULL;
            pv->maching_polys.clear();
            pv->is_intersectPoint = pv->is_updated = false;
        }
    }
}

// �Ӻ���
// ���㶥��index1��index2����·�����߶εĳ���֮��
float SnappingVertices::lineSegDist(int index1, int index2, vector<float> &line_lens){
    float d = line_lens[index1];
    int next = index1 == line_lens.size()-1 ? 0 : index1+1;
    while(next != index2){
        next = next == line_lens.size()-1 ? 0 : next+1;
        d += line_lens[next];
    }
    return d;
}

// ����������(���ڶ���λ���غϣ������ڶ���λ���غϣ�
void SnappingVertices::dealWithVerticesRedundancy(POLYGON &poly, pcl::PointCloud<pcl::PointXYZ>::Ptr &vertices)
{
    // �������㼯�ϵ�kdtree
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(vertices);
    vector<int> indices;
    vector<float> dists;

    vector<vector<int>> clusters;   // �������ඥ������

    vector<bool> is_processed;
	is_processed.resize(vertices->size());
    for(auto iter = is_processed.begin(); iter != is_processed.end(); ++iter) *iter = false;

    // ����kdtree�ҵ�������
    for(int i = 0; i < vertices->size(); ++i){
        if(is_processed[i]) continue;

        tree.radiusSearch(vertices->points[i], 0.001f, indices, dists);
        if(indices.size() > 1){
            vector<int> cluster;
            cluster.reserve(indices.size());
            for(int j = 0; j < indices.size(); ++j){
                cluster.push_back(indices[j]);
                is_processed[indices[j]] = true;
            }
            clusters.push_back(cluster);
        }
    }

    // ÿ���߶γ����������ܳ�
    vector<float> line_lens;
    float poly_circumference = 0;
    line_lens.reserve(vertices->size());
    for(int i = 0; i < vertices->size(); ++i){
        int next = i == vertices->size()-1 ? 0 : i+1;
        float d = distP2P(vertices->points[i], vertices->points[next]);
        line_lens.push_back(d);
        poly_circumference += d;
    }

    // ��Ƕ����Ƿ���Ҫɾ��
    vector<bool> is_del;
    is_del.resize(vertices->size());
    for(auto iter = is_del.begin(); iter != is_del.end(); ++iter) *iter = false;

    // ����ÿ��������
    for(int i = 0; i < clusters.size(); ++i){
        vector<int>*pCluster = &clusters[i];
        // ��������
        std::sort(pCluster->begin(), pCluster->end());
        int j,k;// ע��j,k�����˵�صı߽磬���ʼ�

        j = (*pCluster)[0]; // ��һ��
        k = (*pCluster)[pCluster->size()-1];  // ���һ��

        for(int l = 0; l < pCluster->size(); ++l){
            int l_next = l == pCluster->size()-1 ? 0 : l+1;
            // �����l����l_next�ϳ�����ô��l_next����l�͸���
            // ���l_next��Ϊ��㣻��l��Ϊ�յ�
            if(lineSegDist((*pCluster)[l], (*pCluster)[l_next], line_lens) > 0.5*poly_circumference){
                j = (*pCluster)[l_next];
                k = (*pCluster)[l];
                break;
            }
        }

        // ������������j_next��k֮��Ķ���ȫ��ɾ��
        int j_next = j == vertices->size()-1 ? 0 : j+1;
        for(int m = j_next; m != k; ){
            is_del[m] = true;
            m = m == vertices->size()-1 ? 0 : m+1;
        }
        is_del[k] = true;
    }

    // ����is_delȷ�������ɾ������
    vector<int> off_set;
    off_set.resize(is_del.size());
    int del_count = 0;
    for(int i = 0; i < is_del.size(); ++i){
        if(is_del[i]){
            off_set[i] = -1;
            ++del_count;
        }else{
            off_set[i] = del_count;
        }
    }

    // ����vertices
    for(int i = 0; i < is_del.size(); ++i){
        if(off_set[i] != -1){
            vertices->points[i-off_set[i]] = vertices->points[i];
        }
    }

	for(int i = 0; i < del_count; ++i){ vertices->points.pop_back(); }

    // ����poly
    Vertex *pv;
    for(int i = 0; i < is_del.size(); ++i){
        pv = i == 0 ? poly.start_point : pv->next_point;

        // ��Ҫɾ����ǰ���㣬��Ҫ���浱ǰ�����ǰ���ڵ�
        if(is_del[i]){
            Vertex *pv_prev = pv->prev_point;
            poly.delVertex(pv);
            pv = pv_prev;
        }
    }
}
// ���������߶ι���
void SnappingVertices::dealWithLineSegCollineation(POLYGON &poly, pcl::PointCloud<pcl::PointXYZ>::Ptr &vertices)
{
    vector<bool> is_del; is_del.resize(vertices->size(), false);

    // ��ǰ����"ǰ���߶�"��"����߶�"�ķ�������
    Eigen::Vector3f prev_line_dir, next_line_dir;

    // ��������
    for(int i = 0; i < vertices->size(); ++i){
        int prev = i == 0 ? vertices->size()-1 : i-1;
        int next = i == vertices->size()-1 ? 0 : i+1;

        prev_line_dir << vertices->points[i].x - vertices->points[prev].x,
                         vertices->points[i].y - vertices->points[prev].y,
                         vertices->points[i].z - vertices->points[prev].z;

        next_line_dir << vertices->points[next].x - vertices->points[i].x,
                         vertices->points[next].y - vertices->points[i].y,
                         vertices->points[next].z - vertices->points[i].z;

        prev_line_dir.normalize();
        next_line_dir.normalize();

        float cos_val = prev_line_dir.dot(next_line_dir);
        float abs_cos_val = abs(cos_val);

        if(abs(abs_cos_val - 1.0) < 0.000001f){
            is_del[i] = true;
        }
    }

    // ����is_delȷ�������ɾ������
    vector<int> off_set;
    off_set.resize(is_del.size());
    int del_count = 0;
    for(int i = 0; i < is_del.size(); ++i){
        if(is_del[i]){
            off_set[i] = -1;
            ++del_count;
        }else{
            off_set[i] = del_count;
        }
    }

    // ����vertices
    for(int i = 0; i < is_del.size(); ++i){
        if(off_set[i] != -1){
            vertices->points[i-off_set[i]] = vertices->points[i];
        }
    }

    for(int i = 0; i < del_count; ++i){ vertices->points.pop_back(); }

    // ����poly
    Vertex *pv;
    for(int i = 0; i < is_del.size(); ++i){
        pv = i == 0 ? poly.start_point : pv->next_point;

        // ��Ҫɾ����ǰ���㣬��Ҫ���浱ǰ�����ǰ���ڵ�
        if(is_del[i]){
            Vertex *pv_prev = pv->prev_point;
            poly.delVertex(pv);
            pv = pv_prev;
        }
    }
}

// ������v���뵽����ε�edge_id������ĺ�����
void SnappingVertices::insertIPVertexIntoPoly(Vertex *v, POLYGON &poly,const int edge_id, vector<Vertex*> &poly_vertices){
	// Ŀ���߶ε���ʼ����
	Vertex *begin_v = poly_vertices[edge_id];
	// ��λ��Ŀ�궥���Լ�Ŀ�궥�����һ������
	Vertex *cur_v = poly_vertices[edge_id];
	Vertex *next_v = cur_v->next_point;
	float d1, d2;
	// ���㱻���붥����Ŀ���߶����ľ���
	d1 = this->distP2P(v->point, begin_v->point);
	do{
		// ������һ���������Ŀ���߶����ľ���
		d2 = this->distP2P(next_v->point, begin_v->point);
		// v��Ҫ�ܲ��뵽cur_v��next_v֮�䣬������d1 < d2
		if(d1 < d2){
			poly.insertVertex(cur_v, v);
			return;
		}else{
			// ���d1>d2����ζ�ű����붥����Ҫ���Զ��λ�ô��ƶ�����
			cur_v = cur_v->next_point;
			next_v = cur_v->next_point;
		}
	}while(true);
}

// ����ǽ����߶ε��ཻ
void SnappingVertices::dealWithLineSegIntersect(POLYGON &poly, pcl::PointCloud<pcl::PointXYZ>::Ptr &vertices)
{
    // ��һ�����������в������߶εĽ���
	vector<IntersectingPoint, Eigen::aligned_allocator<IntersectingPoint>> intersect_points;	// �������潻����Ϣ
	getIntersectPointsOfPoly(vertices, intersect_points);
    if(intersect_points.size() == 0) return;	// ���û�н�����ֱ�ӷ���

	// �ڶ�������������뵽�������
	// ���õ�ǰ�����"�Ƿ�Ϊ����"����Ϊfalse
	Vertex *pv;
	for(int i = 0; i < poly.getSize(); ++i){
		pv = i == 0 ? poly.start_point : pv->next_point;
		pv->is_intersectPoint = false;
	}

	// �����еĶ���˳��洢
	vector<Vertex*> poly_vertices;
	for(int i = 0; i < poly.getSize(); ++i){
		pv = i == 0 ? poly.start_point : pv->next_point;
		poly_vertices.push_back(pv);
	}

	// ���������������
	for(int i = 0; i < intersect_points.size(); ++i){
		int edge_id1 = intersect_points[i].edge_id1;
		int edge_id2 = intersect_points[i].edge_id2;
		// �ȴ����һ���߶�
		Vertex *v1 = new Vertex();
		v1->is_intersectPoint = true;
		v1->point = intersect_points[i].point;
        v1->point.data[3] = poly.start_point->point.data[3];
		insertIPVertexIntoPoly(v1, poly, edge_id1, poly_vertices);
		// �ٴ���ڶ����߶�
		Vertex *v2 = new Vertex();
		v2->is_intersectPoint = true;
		v2->point = intersect_points[i].point;
        v2->point.data[3] = poly.start_point->point.data[3];
		insertIPVertexIntoPoly(v2, poly, edge_id2, poly_vertices);
	}

	// �ö�����ж������vertices
	vertices->clear();
	for(int i = 0; i < poly.getSize(); ++i){
		pv = i == 0 ? poly.start_point : pv->next_point;
		vertices->push_back(pv->point);
		if(pv->is_intersectPoint){
			// ��point�еĵ��ĸ�Ԫ�ر����õ��Ƿ�Ϊ���㣬���ֵ�ǣ���Сֵ��
			vertices->points[vertices->size()-1].data[3] = FLT_MAX;
		}else{
			vertices->points[vertices->size()-1].data[3] = FLT_MIN;
		}
	}

	// �������������еĽ��㼯�е�һ��������
	// ip_indices: �������н��������
	vector<int> ip_indices;	 // ip: abbv for intersect_point 
	for(int i = 0; i < vertices->size(); ++i){
		if(vertices->points[i].data[3] == FLT_MAX){
			ip_indices.push_back(i);
		}
	}
	// ������Ǹý����Ƿ��ѱ������
	vector<bool> is_processed;
	is_processed.resize(ip_indices.size(), false);

	// ���Ĳ��������µ��Ӷ����
	vector<vector<int>> sub_polys;	// ���������´������Ӷ���Σ�������Ӧvertices

	// ��ÿ�����㿪ʼ����
	for(int i = 0; i < ip_indices.size(); ++i){
		if(is_processed[i]) continue;	// �����ǰ�����ѱ��������������

		is_processed[i] = true;			// ���õ�ǰ�����ѱ������

		int ip_index = ip_indices[i];	// ��ȡ��ǰ���������	

		vector<int> sub_poly;			// ׼�����������
		sub_poly.push_back(ip_index);	// ����ǰ������Ϊ�¶���ε����

		int next = ip_index == vertices->size()-1 ? 0 : ip_index+1;	// ��ǰ������ԭ������е���һ�����������
		do{
			// ��ͨ����
			if(vertices->points[next].data[3] == FLT_MIN){
				// ֱ�Ӽ�������
				sub_poly.push_back(next);
			}else if(vertices->points[next].data[3] == FLT_MAX &&
				    !isEqual(vertices->points[ip_index], vertices->points[next])){
				// ��ip_index�Ķ�ż����
				// �����ҵ�next�Ķ�ż����dual_next�������ǻ𳵱��һ��
				int dual_next;
				for(int j = 0; j < ip_indices.size(); ++j){
					if(is_processed[j]) continue;
					if(ip_indices[j] == next) continue;
					if(isEqual(vertices->points[next], vertices->points[ip_indices[j]])){// �ҵ��˶�ż��
						dual_next = ip_indices[j];
						break;
					}
				} // forj

				// Ȼ��dual_next��������
				sub_poly.push_back(dual_next);

				// ���������dual_next�ѱ������
				auto iter1 = std::find(ip_indices.begin(), ip_indices.end(), dual_next);
				int dual_next_index = iter1 - ip_indices.begin();
				is_processed[dual_next_index] = true;

				// ������next
				next = dual_next;
			}else{
				// ip_index�Ķ�ż����
				sub_polys.push_back(sub_poly);
				break;	// �������һ������Σ��˳�whileѭ��
			}
			// ָ����һ������
			next = next == vertices->size()-1 ? 0 : next+1;
		}while(true);
	}// for i

	// ���岽 ����ÿ������ε��ܳ�
	vector<float> poly_c;
	float circumference;
	for(int i = 0; i < sub_polys.size(); ++i){
		pcl::PointCloud<pcl::PointXYZ>::Ptr poly_border (new pcl::PointCloud<pcl::PointXYZ>);
		for(int j = 0; j < sub_polys[i].size(); ++j){
			poly_border->push_back(vertices->points[sub_polys[i][j]/*sub_polys�е�i��������е�j��������vertices�е�����*/]);
		}

		// ����poly_border���ܳ�
		circumference = this->getCircumferenceOfPoly(poly_border);
		poly_c.push_back(circumference);
	}

	// ����������poly_c/sub_polys��Ѱ���ܳ�����Ӷ���ε�����
	float max_c = FLT_MIN;
	int poly_index = -1;
	for(int i = 0; i < poly_c.size(); ++i){
		if(poly_c[i] > max_c){
			max_c = poly_c[i];
			poly_index = i;
		}
	}

	// ���߲��������ܳ�����Ӷ������ÿ�������ɾ��״̬Ϊ����������������������ɾ��
	vector<bool> is_del;
	is_del.resize(vertices->size(), true);
	for(int i = 0; i < sub_polys[poly_index].size(); ++i){
		is_del[sub_polys[poly_index][i]] = false;
	}

	// ����poly_vertices;
	poly_vertices.clear();
	for(int i = 0; i < poly.getSize(); ++i){
		pv = i == 0 ? poly.start_point : pv->next_point;
		poly_vertices.push_back(pv);
	}

    // ���ݶ�����������poly_vertices�ҵ�����λ��
	for(int i = 0; i < is_del.size(); ++i){
		if(!is_del[i]) continue;
		poly.delVertex(poly_vertices[i]);
	}

	// �ٴ��ö�����ж������vertices
	vertices->clear();
	for(int i = 0; i < poly.getSize(); ++i){
		pv = i == 0 ? poly.start_point : pv->next_point;
		vertices->push_back(pv->point);
	}
}

// ���ݱ���ж������߶��Ƿ�����
// ������һ���߶��Ƿ�����һ���߶ε�ǰ���߶λ����߶�
bool SnappingVertices::isConnectTwoLineSegs(int size, int edge_id1, int edge_id2)
{
    int prev = edge_id1 == 0 ? size-1 : edge_id1-1;
    int next = edge_id1 == size-1 ? 0 : edge_id1+1;

    return (prev == edge_id2) || (next == edge_id2);
}

// ����һ������������в������߶εĽ���
void SnappingVertices::getIntersectPointsOfPoly(pcl::PointCloud<pcl::PointXYZ>::Ptr &vertices,
                              vector<IntersectingPoint, Eigen::aligned_allocator<IntersectingPoint> > &intersect_points)
{
    if(intersect_points.size() > 0) intersect_points.clear();

    int size = vertices->size();

    pcl::PointXYZ intersect_point;

    for(int i = 0; i < size-1; ++i){
        for(int j = i+1; j < size; ++j){
            if(isConnectTwoLineSegs(size, i, j)) continue;

            int i_next = i == size-1 ? 0 : i+1;
            int j_next = j == size-1 ? 0 : j+1;

            if(getIntersectPointOfTwoLineSeg(vertices->points[i], vertices->points[i_next],
                                             vertices->points[j], vertices->points[j_next],
                                             intersect_point)){
                IntersectingPoint ip;
                ip.edge_id1 = i;
                ip.edge_id2 = j;
                ip.point = intersect_point;
                intersect_points.push_back(ip);
            }
        }
    }
}

// ���������߶εĽ���
// ֻ�н�������������߶��ڲ����Ż᷵�غϷ���ֵ
bool SnappingVertices::getIntersectPointOfTwoLineSeg(pcl::PointXYZ &line1_begin, pcl::PointXYZ &line1_end,
                                   pcl::PointXYZ &line2_begin, pcl::PointXYZ &line2_end,
                                   pcl::PointXYZ &intersect_point)
{
    Eigen::Vector3f p1, p2, p3, p4, p12, p34, p31;
    p1 << line1_begin.x, line1_begin.y, line1_begin.z;
    p2 << line1_end.x, line1_end.y, line1_end.z;
    p3 << line2_begin.x, line2_begin.y, line2_begin.z;
    p4 << line2_end.x, line2_end.y, line2_end.z;
    p12 = p2-p1;
    p34 = p4-p3;
    p31 = p1-p3;

    Eigen::Matrix2f A;
    Eigen::Vector2f x, b;
    A << p12.dot(p12), -1*p12.dot(p34),
         -1*p12.dot(p34), p34.dot(p34);
    b << -1*p31.dot(p12), p31.dot(p34);
    x = A.colPivHouseholderQr().solve(b);
    float alpha1, alpha2;
    alpha1 = x[0];
    alpha2 = x[1];

    Eigen::Vector3f p_l1, p_l2;
    p_l1 = p1 + alpha1*p12;
    p_l2 = p3 + alpha2*p34;

    float dist_p2p = (p_l1-p_l2).norm();

    // alpha1��0��1֮����ζ������ֱ��2����ĵ���ֱ��1���ڲ���ͬ���alpha2
    // ����ľ�����С��ζ�������߶�ȷʵ���ཻ��
    if(alpha1 > 0 && alpha1 < 1 &&
       alpha2 > 0 && alpha2 < 1 &&
       dist_p2p < 0.0005){
        intersect_point.x = p_l1[0];
        intersect_point.y = p_l1[1];
        intersect_point.z = p_l1[2];
        return true;
    }else{
        intersect_point.x = intersect_point.y = intersect_point.z = FLT_MAX;
        return false;
    }
}

// ������ܳ�����
float SnappingVertices::getCircumferenceOfPoly(pcl::PointCloud<pcl::PointXYZ>::Ptr &border)
{
	float circumference = 0;
	for(int i = 0; i < border->size(); ++i){
		int next = i == border->size()-1 ? 0 : i+1;
		circumference += this->distP2P(border->points[i], border->points[next]);
	}
	return circumference;
}

// ���ö���������뾶
void SnappingVertices::setSearchRadiusForEachVertex(vector<InitialPoly> &initial_polys)
{
    POLYGON *pPoly;
    Vertex *pv;
    for(int i = 0; i < polygons.size(); ++i){
        pPoly = &polygons[i];
        for(int j = 0; j < pPoly->getSize(); ++j){
            pv = j == 0 ? pPoly->start_point : pv->next_point;
            pv->radius = this->ratio_of_scale * initial_polys[i].scale;
        }
    }
}

// һ��ִ�к���
// ���������Լ��������������ǰ����
//��һ����
//setParams
//�ڶ�����
// ���µ����ݽṹ��������
//this->m_sv.setPolygons(this->initial_polys);
void SnappingVertices::perform(vector<InitialPoly> &initial_polys)
{
    // ���¶���������뾶
    setSearchRadiusForEachVertex(initial_polys);
    // Ϊ����ε�ÿ���������������ϵ
    setNeighborsForEachPoly();
    // �������ƥ���ϵת��Ϊ˫���ƥ���ϵ
    setMatchingRelations();
    // ��ȡ����"��-��"ƥ����ͨ��ϵ�Ķ����
    getMatchingVerticesClusters(clusters);
    // ����ƥ���ϵ(����ӵ�-��ƥ�䣬�����ܻ���ٵ�-��ƥ��)
    refineMachingRelations();
    // �ϲ�����"��-��"ƥ���ϵ�Ĵ�
    mergeMatchingVerticesClusters(clusters);
    // �����-��ƥ��
    dealWithVertexEdgeMatchingRelation();
    // ���������˻����
    dealWithPolysDegeneration(polygons);
}

// �ھ��нǶԱ߹�ϵ���߶��ϲ��붥��
void SnappingVertices::insertVertexOnEdgeWithAngleToEdgeRelation()
{
	Vertex *pv;
	for(int i = 0; i < polygons.size(); ++i){
		for(int j = 0; j < polygons[i].getSize(); ++j){
			pv = j == 0 ? polygons[i].start_point : pv->next_point;
			// �����������������Ƿ���ڴ�����pv���߶Σ�pv�����߶��ڲ�
			// ����У����ڶ�Ӧ�߶��ڲ�������pv��ͬһ��λ�õĶ���
			checkLineSegThatPassThroughVertex(i, pv);
		}
	}
}

void SnappingVertices::checkLineSegThatPassThroughVertex(int poly_id, Vertex *pv)
{
	Vertex *edge_begin;
	for(int i = 0; i < polygons.size(); ++i){
		if(i == poly_id) continue;
		for(int j = 0; j < polygons[i].getSize(); ++j){
			edge_begin = j == 0 ? polygons[i].start_point : edge_begin->next_point;

			// ���pv�Ƿ����߶�(edge_begin, edge_end)���ڲ�
			if(isVertexOnEdge(pv, edge_begin)){
				Vertex * v_new = new Vertex();
				v_new->point = pv->point;
				v_new->point.data[3] = edge_begin->point.data[3];
				v_new->radius = edge_begin->radius;
				polygons[i].insertVertex(edge_begin, v_new);
			}
		}
	}
}

bool SnappingVertices::isVertexOnEdge(Vertex *pv, Vertex *edge_begin)
{
	Eigen::Vector3f p_base, line_dir;
	p_base << edge_begin->point.x, edge_begin->point.y, edge_begin->point.z;
    line_dir << edge_begin->next_point->point.x - edge_begin->point.x,
				edge_begin->next_point->point.y - edge_begin->point.y,
                edge_begin->next_point->point.z - edge_begin->point.z;
    line_dir.normalize();

    Eigen::Vector3f v_p, delta;
	v_p << pv->point.x, pv->point.y, pv->point.z;
    delta = p_base - v_p;
    float lambda = -1.0 * line_dir.dot(delta);

    Eigen::Vector3f p_proj;
    p_proj = p_base + lambda * line_dir;

	float dist = (v_p - p_proj).norm();

	float epsilon = 0.0001;

	if(dist > epsilon) return false;

	float dist1 = this->distP2P(pv->point, edge_begin->point);
	float dist2 = this->distP2P(pv->point, edge_begin->next_point->point);

	if(dist1 <= epsilon*10) return false;				// ���ڶ˵���
	if(dist2 <= epsilon*10) return false;	// ���ڶ˵���

	float dist3 = this->distP2P(edge_begin->point, edge_begin->next_point->point);

	if(dist1+dist2 - dist3 > epsilon*9) return false;	// �����߶��ⲿ
	else return true;						// �����߶��ڲ�
}

// ��鵱ǰģ���Ƿ�Ϸ�
// �����⵽�Ƿ�������򷵻����ⶥ��
bool SnappingVertices::is_model_valid(Vertex *&prob_v)
{
    // ���ȶԶ����ִ�����ǻ�
    EarClip earClip;

    for(int i = 0; i < polygons.size(); ++i){
        earClip.setInputPoly(&(polygons[i]));
        if(!earClip.triangulatePoly()){
            cerr << "poly_id = " << i << ", triangulation failed!" << endl;
        }
    }

    for(int i = 0; i < polygons.size(); ++i){
        POLYGON *pPoly = &polygons[i];

        Vertex* pv;
        for(int j = 0; j < pPoly->getSize(); ++j){
            pv = j == 0 ? pPoly->start_point : pv->next_point;

            // pv �Ƿ�������������εı��ϣ������ߵ������յ㣩
            if(is_vertex_on_edge(i, pv)) continue;
            // pv �Ƿ���������������ڲ������������ڱ߽��ϣ�
            if(is_vertex_in_poly(i, pv)){
                prob_v = pv;
                return false;
            }
            // [pv pv->next_point]�Ƿ񴩹���������ε��ڲ�
            if(is_edge_pass_through_poly(i, pv, pv->next_point)){
                prob_v = pv;
                return false;
            }
        }
    }
    return true;
}

// ���Ϊpoly_id�Ķ���εĶ���v�Ƿ�������������εı���
bool SnappingVertices::is_vertex_on_edge(int poly_id, Vertex* v)
{
    float epsilon = 0.0001;
    for(int i = 0; i < polygons.size(); ++i){
        if(i == poly_id) continue;
        POLYGON *pPoly = &polygons[i];

        Vertex* pv;
        for(int j = 0; j < pPoly->getSize(); ++j){
            pv = j == 0 ? pPoly->start_point : pv->next_point;

            // ��-��
            if(distP2P(v->point, pv->point) <= epsilon)
                return true;

            // ��-��
            if(distP2E(v->point, pv->point, pv->next_point->point) <= epsilon)
                return true;
        }
    }
    return false;
}

// ���Ϊpoly_id�Ķ���εĶ���v�Ƿ�������������ε��ڲ�
bool SnappingVertices::is_vertex_in_poly(int poly_id, Vertex* v)
{
    float epsilon = 0.1;
    for(int i = 0; i < polygons.size(); ++i){
        if(i == poly_id) continue;
        POLYGON *pPoly = &polygons[i];

        // ���ȼ������ƽ��ķ��ž���
        float dist = v->point.x*pPoly->coeff[0] +
                     v->point.y*pPoly->coeff[1] +
                     v->point.z*pPoly->coeff[2] +
                                pPoly->coeff[3];
        if(abs(dist) > epsilon) continue;

        // ����㵽ƽ���ͶӰ��
        pcl::PointXYZ proj_p;
        proj_p.x = v->point.x - dist * pPoly->coeff[0];
        proj_p.y = v->point.y - dist * pPoly->coeff[1];
        proj_p.z = v->point.z - dist * pPoly->coeff[2];

        // �ж�ͶӰ���Ƿ��ڶ�����ڲ�
        for(int j = 0; j < pPoly->triangles.size(); ++j){
            if(is_vertex_in_triangle(v->point, pPoly->triangles[j])){
                return true;
            }
        }
    }
    return false;
}

// ĳ���Ƿ����������ڲ�
bool SnappingVertices::is_vertex_in_triangle(pcl::PointXYZ &point, Triangle &tri)
{
    Eigen::Vector3f e01, e02, e0, e1, e2;
    e01 << tri.v[1]->point.x - tri.v[0]->point.x,
           tri.v[1]->point.y - tri.v[0]->point.y,
           tri.v[1]->point.z - tri.v[0]->point.z;
    e02 << tri.v[2]->point.x - tri.v[0]->point.x,
           tri.v[2]->point.y - tri.v[0]->point.y,
           tri.v[2]->point.z - tri.v[0]->point.z;
    float S = (e01.cross(e02)).norm() * 0.5;
    S = abs(S);

    e0 << tri.v[0]->point.x - point.x,
          tri.v[0]->point.y - point.y,
          tri.v[0]->point.z - point.z;
    e1 << tri.v[1]->point.x - point.x,
          tri.v[1]->point.y - point.y,
          tri.v[1]->point.z - point.z;
    e2 << tri.v[2]->point.x - point.x,
          tri.v[2]->point.y - point.y,
          tri.v[2]->point.z - point.z;
    float S1 = e0.cross(e1).norm() * 0.5;
    float S2 = e0.cross(e2).norm() * 0.5;
    float S3 = e1.cross(e2).norm() * 0.5;
    S1 = abs(S1);
    S2 = abs(S2);
    S3 = abs(S3);

    return abs(S1+S2+S3-S) <= 0.00001;
}

// �߶�[e1, e2]�Ƿ񴩹���������ε��ڲ�
bool SnappingVertices::is_edge_pass_through_poly(int poly_id, Vertex* e1, Vertex* e2)
{
    //float epsilon = 0.0001;
    for(int i = 0; i < polygons.size(); ++i){
        if(i == poly_id) continue;
        POLYGON *pPoly = &polygons[i];

        // e1��pPoly֧��ƽ��ķ��ž���
        float d1 = e1->point.x * pPoly->coeff[0] +
                   e1->point.y * pPoly->coeff[1] +
                   e1->point.z * pPoly->coeff[2] +
                                 pPoly->coeff[3];
        float d2 = e2->point.x * pPoly->coeff[0] +
                   e2->point.y * pPoly->coeff[1] +
                   e2->point.z * pPoly->coeff[2] +
                                 pPoly->coeff[3];
		if(d1*d2 >= 0) return false;	// λ��ƽ���ͬһ���򣬲������ཻ
        if(d1*d2 >= -0.00001) return false;	// ��һ������λ��֧��ƽ����(���丽��)���಻�����ཻ

        // �����߶�������֧��ƽ��Ľ���
        double A, B, lambda;
        A = pPoly->coeff[0] * (e2->point.x - e1->point.x) +
            pPoly->coeff[1] * (e2->point.y - e1->point.y) +
            pPoly->coeff[2] * (e2->point.z - e1->point.z);
        B = pPoly->coeff[0] * e1->point.x +
            pPoly->coeff[1] * e1->point.y +
            pPoly->coeff[2] * e1->point.z +
            pPoly->coeff[3];
        lambda = B/A*-1;
        pcl::PointXYZ p_intersect;
        p_intersect.x = e1->point.x + lambda * (e2->point.x - e1->point.x);
        p_intersect.y = e1->point.y + lambda * (e2->point.y - e1->point.y);
        p_intersect.z = e1->point.z + lambda * (e2->point.z - e1->point.z);

        // �жϽ����Ƿ��ڶ�����ڲ�
        for(int j = 0; j < pPoly->triangles.size(); ++j){
            if(is_vertex_in_triangle(p_intersect, pPoly->triangles[j])){
                return true;
            }
        }
    }

    return false;
}
