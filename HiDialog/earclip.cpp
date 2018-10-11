#include "earclip.h"

EarClip::EarClip()
{
    this->m_poly = NULL;
    PI = 3.14159265358979;
}

// ��������Ķ����
void EarClip::setInputPoly(POLYGON *pPoly)
{
    this->vertices.clear();
    this->m_poly = pPoly;
    Vertex *pv;
    for(int i = 0; i < pPoly->getSize(); ++i){
        pv = i == 0 ? pPoly->start_point : pv->next_point;
        this->vertices.push_back(pv);
    }
    if(vertices.size() != pPoly->getSize()){
        cerr << "in EarClip::setInputPoly(POLYGON *pPoly) vertices.size() != pPoly->getSize()" << endl;
        cerr << "vertices.size() = " << vertices.size() << endl;
        cerr << "pPoly->getSize() = " << pPoly->getSize() << endl;
    }
}

// �Զ���ν������ǻ�
// ǰ�᣺ƽ��ķ�����ָ�������ⲿ
//		ƽ��߽����Ҫ�������ַ��򣺴�Ĵָָ��ƽ�����ࣨ����line(p0,p1)��ߵĵ��ڶ�����ڲ����ұߵĵ��ڶ�����ⲿ��
bool EarClip::triangulatePoly()
{
    if(!m_poly){
        cerr << "in EarClip::triangulatePoly(), m_poly == NULL" << endl;
        return false;
    }

    // ����ζ�����������
    if(m_poly->getSize() < 3){
        cerr << "in EarClip::triangulatePoly(), m_poly->getSize() = " << m_poly->getSize() << endl;
        return false;
    }

    // �������ε�����������
    m_poly->triangles.clear();
    int poly_size = m_poly->getSize();
    // ���������� = ����ζ������� - 2
	m_poly->triangles.reserve(poly_size-2);

    // ����������������ֻ�������㣬��ֱ�ӹ��������Σ�Ȼ���˳�
    if(poly_size == 3){
        Triangle triangle;
        triangle.v[0] = vertices[0];
        triangle.v[1] = vertices[1];
        triangle.v[2] = vertices[2];

        m_poly->triangles.push_back(triangle);
        return true;
    }

    // ���ڶ�����������3��һ�����
    // ���ȱ������εĶ�������
    // indices: ��ǰ�Ķ����б�
    std::vector<int> indices;
    indices.resize(poly_size);
    int i__ = 0;
    for(auto iter = indices.begin(); iter != indices.end(); ++iter){*iter = i__++;}

    // Ȼ����ÿ�������ںϷ��Լ�����Ƿ���Ҫ���м�飨ֻ���ڽ�ΪPI�Ķ���Ų����д����飩
    // �����жϸõ��Ƿ��ڱ������������ڲ�
    std::vector<bool> is_checked;
    is_checked.resize(poly_size);
    for(auto iter = is_checked.begin(); iter != is_checked.end(); ++iter){*iter = true;}    // Ĭ��ÿ���㶼��Ҫ���
    // �洢ÿ��������ڽ�
    std::vector<double> inner_angles;
    inner_angles.resize(poly_size);
    for(auto iter = inner_angles.begin(); iter != inner_angles.end(); ++iter){*iter = -1;}
    // ��Ǹö����Ƿ��Ѿ���������ֻ��û�������Ķ���ſ��Լ������ڽ�
    std::vector<bool> is_cut;
    is_cut.resize(poly_size);
    for(auto iter = is_cut.begin(); iter != is_cut.end(); ++iter){*iter = false;}   // һ��ʼ��û�б�����

    // ��ÿ�ε����У��������ȴ���Ƕ������ڽ�
    // ����ڽǳ�����PI����ֱ�Ӻ���(��Ϊ�����ܱ������Ӷ�����һ���µ�������)
    // ����ڽ�ΪPI���򹹽��˻������Σ������ƽ�Ƕ���Ϊ�������Ķ���
    // ����ڽ�С��PI�������һ��������ڲ��Ƿ����������㣬���û���������㣬��ɹ���������
    // ÿ����һ���µ������Σ������¼����ܵ�Ӱ��Ķ�����ڽǣ��ٽ���Щ�ڽǴӴ�С����
    int vertices_size = poly_size;   // vertices_size: ԭʼ���������
    Eigen::Vector3f normal;   // normal: ����εķ�����
    normal << m_poly->coeff[0],
              m_poly->coeff[1],
              m_poly->coeff[2];
    double eps = 0.000001;

    int while_count = 0;  // while_count: ͳ��whileѭ���Ĵ������Ӷ��������ѭ�������
    while(true){
        if(while_count ++ >= 100000){
            cerr << "���ܳ�������ѭ��" << endl;
            return false;
        }
        // �����һ�ν�ʣ���������㣬��ô�е�һ������֮��ֻʣ���������㣬��ʱ���е������ξ�����ȡ�����˳��ú���
        if(indices.size() < 3){
            return true;
        }

        // ���㶥���ڽ�
        // i: ��ǰ���������
        // prev: ǰ�����������
        // next: ��̶��������
        for(int i = 0; i < vertices_size; ++i){
            if(is_cut[i]){  // ��������ѱ��е��������Ӧ���ڽǸ�������ֵ
                inner_angles[i] = -1;
            }else{
                // �ҵ�i��indices�е�λ��index
                int index = -1;
                for(index = 0; index < indices.size(); ++index){
                    if(indices[index] == i){
                        break;
                    }
                }
                if(index == -1){std::cerr << "index = -1" << endl; return false;}
                // �ҵ�i��ǰ������
                int prev, next;
                prev = indices[getPrev(indices, index)];
                next = indices[getNext(indices, index)];

                inner_angles[i] = getInnerAngle(vertices, normal, i, prev, next);
            }
        }

        // Ѱ���ڽ�ΪPI�Ķ���
        bool is_find_PI = false;
        for(int i = 0; i < vertices_size; ++i){
            if(abs(inner_angles[i]-PI) < eps){  // �ҵ�ƽ�Ƕ���
                // �ҵ�i��indices�е�λ��
                int index = -1;
                for(index = 0; index < indices.size(); ++index){
                    if(indices[index] == i){
                        break;
                    }
                }
                if(indices[index]-i != 0) cerr << "indices[index]-i != 0\n" << "i = " << i << "\nindices[index]=" << indices[index] << endl;
                // �ҵ�i��ǰ������
                int prev, next;
                prev = indices[getPrev(indices, index)];
                next = indices[getNext(indices, index)];

                // ����������
                Triangle triangle;
//                triangle.a = i;
//                triangle.b = prev;
//                triangle.c = next;
                triangle.v[0] = vertices[i];
                triangle.v[1] = vertices[prev];
                triangle.v[2] = vertices[next];
                m_poly->triangles.push_back(triangle);
                del_vertex(indices, index); // ���ö���Ӷ���������ɾ��
                is_cut[i] = true;           // �е��ö���
                is_checked[i] = false;      // ��Ϊ��ƽ�ǣ����������õ㱻����Ȩ��
                is_find_PI = true;          // ��ʶ
                break;
            }
        }

        /*/ test code2:
            cout << "is_find_PI: " << is_find_PI << endl;*/

        if(is_find_PI){
            continue;
        }

        // �������Ƕ���Ѱ�����ĺϷ��ڽǣ���Ȼ���ܳ���PI��
        double max_angle = -1;// ���Ϸ��Ƕ�
        int max_i = -1;       // �����Ƕȶ�Ӧ�Ķ�������(���ԭʼ�Ķ�������)
        int max_prev, max_next;
        for(int i = 0; i < vertices_size; ++i){
            // �����ų������������Ȼ�����Ѿ���ɾ���Ķ��㣬����Щ�������Ѳ�������indices��
            // ��PI�Ķ��㲻����
            if(inner_angles[i] > PI) continue;
            // �Ѿ���ɾ���Ķ��㲻����
            if(is_cut[i]) continue;

            // Ѱ�Ҹö����ǰ����̽��
            int index = -1;
            for(index = 0; index < indices.size(); ++index){
                if(indices[index] == i){
                    break;
                }
            }
            if(indices[index]-i != 0) cerr << "indices[index]-i != 0\n" << "i = " << i << "\nindices[index]=" << indices[index] << endl;

            int prev, next;
            prev = indices[getPrev(indices, index)];
            next = indices[getNext(indices, index)];

            // ���ܵ������Σ�<i,prev,next>
            // ���������ڲ��Ƿ������������
            bool is_other_point_in_triangle = false;
            for(int j = 0; j < vertices_size; ++j){
                if(j == i || j == prev || j == next || !is_checked[j])
                    continue;

                if(isPointInTriangle(j, i, prev, next)){
                    is_other_point_in_triangle = true;
                    break;
                }
            }
            // ���������ڲ�û���������������Ƕ�С�ڵ�ǰ�ڽǣ���������Ƕ�
            if(!is_other_point_in_triangle && max_angle < inner_angles[i]){
                max_angle = inner_angles[i];
                max_i = i;
                max_prev = prev;
                max_next = next;
            }

        }
        if(max_angle == -1){
            cerr << "max_angle == -1 in trianglatePlane" << endl;
            return false;
        }

        /*/ test code3:
            cout << "max_angle = " << max_angle << endl;
            cout << "max_i = " << max_i << "; max_prev = " << max_prev << "; max_next = " << max_next << endl;
            */

        // ����������
        Triangle triangle;
//        triangle.a = max_i;
//        triangle.b = max_prev;
//        triangle.c = max_next;
        triangle.v[0] = vertices[max_i];
        triangle.v[1] = vertices[max_prev];
        triangle.v[2] = vertices[max_next];
        m_poly->triangles.push_back(triangle);

        is_cut[max_i] = true;   // ��Ǹö����ѱ��е�

        // ��indices��ɾ���ö���
        // �ҵ�max_i��Ӧ��indices�е�����.
        int i_ = -1;
        for(i_ = 0; i_ < indices.size(); ++i_){
            if(indices[i_] == max_i){
                break;
            }
        }
        if(i_ == indices.size()){
            cerr << "i_ == -1" << endl;
            return false;
        }
        /*/ test code4:
            cout << "indices.size = " << indices.size() << endl;
            cout << "i_ = " << i_ << endl;
            cout << "indices[i_] = " << indices[i_] << endl;
            cout << "Sleep(10000)" << endl;
            Sleep(10000);*/
        del_vertex(indices, i_);
        /*cout << "\nindices.size = " << indices.size() << endl;
            cout << "Sleep(10000)" << endl;
            Sleep(10000);
            cout << "nextnextnext" << endl;*/
    }
}

// ��ȡ����i��ǰ�����
int EarClip::getPrev(std::vector<int> &indices, int i)
{
    if(i < 0 || i >= indices.size())
    {
        std::cerr << i << " exceeds the scope of [0, " << indices.size()-1 << "] in getPrev" << endl;
        return -1;
    }

    return i == 0 ? indices.size()-1 : i-1;
}

// ��ȡ����i�ĺ�̽��
int EarClip::getNext(std::vector<int> &indices, int i)
{
    if(i < 0 || i >= indices.size())
    {
        std::cerr << i << " exceeds the scope of [0, " << indices.size()-1 << "] in getNext" << endl;
        return -1;
    }

    return i == indices.size()-1 ? 0 : i+1;
}

// ɾ��vector�еĵ�i��Ԫ��
void EarClip::del_vertex(std::vector<int> &indices, int i)
{
    if(i < 0 || i >= indices.size()){
        std::cerr << i << " exceeds the scope of [0, " << indices.size()-1 << "] in del_vertex" << endl;
        return;
    }

    int j = 0;
    for(auto iter = indices.begin(); iter != indices.end(); ++iter){
        if(j == i){
            indices.erase(iter);
            return;
        }else{
            ++j;
        }
    }
}

// ĳ���Ƿ����������ڲ�
// Ŀ�궥�������: p_index
// �����ζ�������: tri_v1, tri_v2, tri_v3
bool EarClip::isPointInTriangle(int p_index, int tri_v1, int tri_v2, int tri_v3)
{
    // ���ȹ�������
    Eigen::Vector3d v_12, v_13, v_p1, v_p2, v_p3;
    v_12 << vertices[tri_v2]->point.x - vertices[tri_v1]->point.x,
            vertices[tri_v2]->point.y - vertices[tri_v1]->point.y,
            vertices[tri_v2]->point.z - vertices[tri_v1]->point.z;
    v_13 << vertices[tri_v3]->point.x - vertices[tri_v1]->point.x,
            vertices[tri_v3]->point.y - vertices[tri_v1]->point.y,
            vertices[tri_v3]->point.z - vertices[tri_v1]->point.z;

    v_p1 << vertices[tri_v1]->point.x - vertices[p_index]->point.x,
            vertices[tri_v1]->point.y - vertices[p_index]->point.y,
            vertices[tri_v1]->point.z - vertices[p_index]->point.z;
    v_p2 << vertices[tri_v2]->point.x - vertices[p_index]->point.x,
            vertices[tri_v2]->point.y - vertices[p_index]->point.y,
            vertices[tri_v2]->point.z - vertices[p_index]->point.z;
    v_p3 << vertices[tri_v3]->point.x - vertices[p_index]->point.x,
            vertices[tri_v3]->point.y - vertices[p_index]->point.y,
            vertices[tri_v3]->point.z - vertices[p_index]->point.z;

    double eps = 0.000001;
    double S = v_12.cross(v_13).norm() *0.5;
    double S1 = v_p1.cross(v_p2).norm() *0.5;
    double S2 = v_p1.cross(v_p3).norm() *0.5;
    double S3 = v_p2.cross(v_p3).norm() *0.5;
    return fabs(S1+S2+S3-S) <= eps;
}

// ��ȡ����ζ�����ڽ�
// vertices: �������У���ʱ������
// normal: ����η�������ָ�������ⲿ
// index: Ŀ�궥������
// prev: index��ǰ��
// next: index�ĺ��
double EarClip::getInnerAngle(vector<Vertex *> &vertices, Eigen::Vector3f &normal, int index, int prev, int next)
{
    // �����ų���������
    // ������T��<index, prev, next>
    // T����С�߳�����С����ֵ����˱���ų�����Ϊ0�����
    float T_min_side_length = 0.001;    // 1����
    float d1 = distP2P(vertices[index]->point, vertices[prev]->point);
    float d2 = distP2P(vertices[index]->point, vertices[next]->point);
    float d3 = distP2P(vertices[prev]->point, vertices[next]->point);
    float d_min = d1;
    if(d_min > d2) d_min = d2;
    if(d_min > d3) d_min = d3;
    if(d_min < T_min_side_length){
        std::cerr << "����������ľ��������Ϊ" << d_min << "m�� ������ֵT_min_side_length:" << d_min << "m" << endl;
        return -1;
    }

    // ��������������㹲�ߵ��������
    Eigen::Vector3d v1, v2, n;
    v1 << vertices[prev]->point.x - vertices[index]->point.x,
          vertices[prev]->point.y - vertices[index]->point.y,
          vertices[prev]->point.z - vertices[index]->point.z;
    if(v1.norm() > 1) v1.normalize();   // ֻ���������ȹ������ſ��ǽ����һ��
    v2 << vertices[next]->point.x - vertices[index]->point.x,
          vertices[next]->point.y - vertices[index]->point.y,
          vertices[next]->point.z - vertices[index]->point.z;
    if(v2.norm() > 1) v2.normalize();
    n << normal[0], normal[1], normal[2];

    double eps = 0.000001;
    // ��˵ķ���Ϊ0�������㹲�ߣ�v1������v2��180�㣬Ҳ���ܳ�0�㡣
    Eigen::Vector3d v1_cross_v2 = v1.cross(v2);
    if(v1_cross_v2.norm() <= eps){
        v1.normalize();
        v2.normalize();
        double cos_angle = v1.dot(v2);
        if(cos_angle < 0) {
            return PI;
        } else{
            /*std::cerr << "�����˼����죬 ����ֵ��" << cos_angle << endl;
            std::cerr << "current point: " << index << vertices->points[index] << endl;
            std::cerr << "prev point: " << prev << vertices->points[prev] << endl;
            std::cerr << "next point: " << next << vertices->points[next] << endl;*/

            return 0;
        }
    }

    v1_cross_v2.normalize();
    // ����뷨���������ڽǣ�PI
    if(v1_cross_v2.dot(n) < 0){
        return acos(v1.dot(v2));
    }else{
        // ����뷨����ͬ���ڽǣ�PI
        return 2.0*PI - acos(v1.dot(v2));
    }
}

// ��-����뺯��
float EarClip::distP2P(pcl::PointXYZ &p1, pcl::PointXYZ &p2)
{
    float dist = (p1.x-p2.x)*(p1.x-p2.x) +
                 (p1.y-p2.y)*(p1.y-p2.y) +
                 (p1.z-p2.z)*(p1.z-p2.z);
    return pow(dist, 0.5f);
}





































