#include "osnap.h"

OSnap::OSnap()
{
    m_poly = NULL;
	this->delta = 0.01;
    this->max_k_count = 50;
	f1_weight = f2_weight = f3_1_weight = f3_2_weight = f8_weight = f4_weight = f5_weight = f6_weight = f7_weight = 1;
}

// �������ݽṹ
void OSnap::reset()
{
	this->m_polygons = NULL;
    this->m_poly = NULL;

    this->normal_vertices.clear();
    this->p2e_vertices.clear();
    this->p2p_vertices.clear();
	normal_vertices_indices.clear();
	p2p_vertices_indices.clear();
	p2e_vertices_indices.clear();

    m_osnap_poly.c << 0, 0, 0;
    m_osnap_poly.c_ << 0, 0, 0;
    m_osnap_poly.b0 << 0, 0, 0;
    m_osnap_poly.b1 << 0, 0, 0;
    m_osnap_poly.b2 << 0, 0, 0;
    m_osnap_poly.coeff << 0, 0, 0, 0;
    m_osnap_poly.coeff_origin << 0, 0, 0, 0;
    m_osnap_poly.vertices.clear();
    m_osnap_poly.vertices_origin.clear();
    m_osnap_poly.coors.clear();

	J.resize(0, 0);
	beta.resize(0);

    //this->delta = 0.01;
}

// ��������Ķ���ζ���m_poly����Ŀ�����ζ���m_osnap_poly
void OSnap::setOSnapPoly()
{
	int vertices_size = m_poly->getSize();
	this->m_osnap_poly.vertices.resize(vertices_size);
	this->m_osnap_poly.vertices_origin.resize(vertices_size);

    // ����ε�ԭʼ����
    Vertex *pv;
    for(int i = 0; i < m_poly->getSize(); ++i){
        pv = i == 0 ? m_poly->start_point : pv->next_point;
		m_osnap_poly.vertices_origin[i] << pv->point.x, pv->point.y, pv->point.z;
		m_osnap_poly.vertices[i] = m_osnap_poly.vertices_origin[i];
    }
    // ����η���ϵ��
    m_osnap_poly.coeff_origin << m_poly->coeff[0],
                                 m_poly->coeff[1],
                                 m_poly->coeff[2],
                                 m_poly->coeff[3];
    m_osnap_poly.coeff = m_osnap_poly.coeff_origin;
    // �����ƶ�����ϵ
    setBases_init(m_osnap_poly);
    // ���ö���ζ������ƶ�����ϵ�еĶ�ά����
    set2DCoordinates(m_osnap_poly);
}

// ���ö��㹹���ʼ���ƶ�����ϵ
void OSnap::setBases_init(OSnapPoly &poly)
{
	Eigen::Vector3d v_centroid;
	v_centroid.setZero();	// ����ÿ��Ԫ��Ϊ0
    for(int i = 0; i < poly.vertices.size(); ++i)
    {
        v_centroid += poly.vertices[i];
    }

    // �ƶ�����ϵ��ԭ��=���㼯�ϵ�����
	v_centroid /= poly.vertices.size();
    poly.b0 = v_centroid;

    Eigen::Vector3d v, e1, e2;
    // �ƶ�����ϵ��һ����e1ָ�����ε�ĳ������
    for(int i = 0; i < poly.vertices.size(); ++i)
    {
        Eigen::Vector3d v_p;
		v_p = poly.vertices[i];
        v = v_p - poly.b0;
        if(v.norm() > 0.01)
        {
            e1 = 1.0 / v.norm() * v;
            break;
        }
    }

    // ����ƽ��ķ�����
    Eigen::Vector3d normal;
    normal << poly.coeff[0], poly.coeff[1], poly.coeff[2];

    e2 = e1.cross(normal);
    e2 = 1.0 / e2.norm() * e2;

    poly.b1 = poly.b0 + e1;
    poly.b2 = poly.b0 + e2;
}

// ���ö���ζ������ƶ�����ϵ�еĶ�ά����
void OSnap::set2DCoordinates(OSnapPoly &poly)
{
    // �������ԭ�еĶ�ά����
    poly.coors.clear();

    Eigen::Matrix<double, 3, 2> A;
    Eigen::Vector3d e1, e2;
    e1 = poly.b1 - poly.b0;
    e2 = poly.b2 - poly.b0;
    A << e1[0], e2[0],
         e1[1], e2[1],
         e1[2], e2[2];
    Eigen::Vector2d coor;
    Coor_sigma coor_sigma;
    for(int i = 0; i < poly.vertices.size(); ++i)
    {
        Eigen::Vector3d p;
		p = poly.vertices[i];
        coor = (A.transpose() * A).inverse() * A.transpose() * (p - poly.b0);
        coor_sigma.px = coor[0];
        coor_sigma.py = coor[1];
        poly.coors.push_back(coor_sigma);
    }
}

// ����ƥ���ϵ���ඥ��
void OSnap::categoriseVertices()
{
    if(!m_poly){
        cerr << "error in OSnap::categoriseVertices(): m_poly == NULL" << endl;
		return;
	}

    Vertex *pv;
    for(int i = 0; i < m_poly->getSize(); ++i){
        pv = i == 0 ? m_poly->start_point : pv->next_point;
        if(pv->neighbor_poly_id == -1){
            // û��ƥ���ϵ
            this->normal_vertices.push_back(pv);
			normal_vertices_indices.push_back(i);
        }else if(pv->is_vertex_neighbor){
            // ��-��ƥ��
            this->p2p_vertices.push_back(pv);
			p2p_vertices_indices.push_back(i);
        }else{
            // ��-��ƥ��
            this->p2e_vertices.push_back(pv);
			p2e_vertices_indices.push_back(i);
        }		
    }
}

// ���������˶�
// ���ñ���
void OSnap::setHelicalVars( Eigen::Vector3d &c,
                     Eigen::Vector3d &c_,
                     Eigen::Matrix3d &R,
                     Eigen::Vector3d &pointOfA,
                     Eigen::Vector3d &a,
                     Eigen::Vector3d &a_,
                     double &pitch,
                     double &alpha )
{
    pitch = c.dot(c_) / c.dot(c);
    a = (1.0/c.norm()) * c;
    a_ = c_ - pitch * c;
    alpha = atan(c.norm());
    pointOfA = a.cross(a_);

    double b0 = cos(alpha*0.5);
    double b1 = a[0] * sin(alpha*0.5);
    double b2 = a[1] * sin(alpha*0.5);
    double b3 = a[2] * sin(alpha*0.5);

    double m00 = b0 * b0 + b1 * b1 + b2 * b2 + b3 * b3;
    double m11 = b0 * b0 + b1 * b1 - b2 * b2 - b3 * b3;
    double m22 = b0 * b0 - b1 * b1 + b2 * b2 - b3 * b3;
    double m33 = b0 * b0 - b1 * b1 - b2 * b2 + b3 * b3;

    R << m11, 2.0*(b1*b2 + b0*b3), 2.0*(b1*b3 - b0*b2),
         2.0*(b1*b2 - b0*b3), m22, 2.0*(b2*b3 + b0*b1),
         2.0*(b1*b3 + b0*b2), 2.0*(b2*b3 - b0*b1), m33;
    R = 1.0/m00 * R;
}

// ִ���ƶ�
void OSnap::uniformHelicalMove( Eigen::Vector3d &c,
                         Eigen::Vector3d &c_,
                         Eigen::Vector3d &x,
                         Eigen::Vector3d &x_,
                         Eigen::Matrix3d &R,
                         Eigen::Vector3d &pointOfA,
                         Eigen::Vector3d &a,
                         double &pitch,
                         double &alpha )
{
    // ������ٶȲ���������
    if(c.norm() > 0.000000000001)
        x_ = R * (x - pointOfA) + pitch * alpha * a + pointOfA;
    else
        x_ = x + c_;
}

// ���¾��������˶�����ƶ�����ϵ�Ļ�
void OSnap::updateBasesAfterHelicalMove(OSnapPoly &poly)
{
    Eigen::Matrix3d R;
    Eigen::Vector3d pointOfA;
    Eigen::Vector3d a;
    Eigen::Vector3d a_;
    double pitch;
    double alpha;
    setHelicalVars( poly.c, poly.c_, R, pointOfA, a, a_, pitch, alpha);

    Eigen::Vector3d x_;
    uniformHelicalMove( poly.c, poly.c_, poly.b0, x_, R, pointOfA, a, pitch, alpha);
    poly.b0 << x_[0], x_[1], x_[2];
    uniformHelicalMove( poly.c, poly.c_, poly.b1, x_, R, pointOfA, a, pitch, alpha);
    poly.b1 << x_[0], x_[1], x_[2];
    uniformHelicalMove( poly.c, poly.c_, poly.b2, x_, R, pointOfA, a, pitch, alpha);
    poly.b2 << x_[0], x_[1], x_[2];
}

// ���¾��������˶��󶥵����ά����
// (ǰ�᣺���ƶ�����ϵ�Ļ���+����ά���ꡯ�ѱ����¹���)
void OSnap::updateVerticesAfterHelicalMove(OSnapPoly &poly)
{
    Eigen::Vector3d p;
    for(int i = 0; i < poly.vertices.size(); ++i)
    {
        p = poly.b0 + poly.coors[i].px * (poly.b1 - poly.b0)
                    + poly.coors[i].py * (poly.b2 - poly.b0);
        poly.vertices[i] = p;
    }
}

// ���ݱ�������beta����ÿ���������ƶ�����ϵ�еĶ�ά����
void OSnap::updateCoors()
{
	int start_index = 6;
	int v_index;
	int beta_index;
	// ���¾���p2pƥ���ϵ�Ķ������ƶ�����ϵ�еĶ�ά����
	for(int i = 0; i < this->p2p_vertices_indices.size(); ++i){
		v_index = p2p_vertices_indices[i];
		beta_index = start_index + 2*i;
		m_osnap_poly.coors[v_index].px += beta[beta_index];
		m_osnap_poly.coors[v_index].py += beta[beta_index+1];
	}

	start_index += p2p_vertices.size() * 2;
	// ���¾���p2eƥ���ϵ�Ķ������ƶ�����ϵ�еĶ�ά����
	for(int i = 0; i < this->p2e_vertices_indices.size(); ++i){
		v_index = p2e_vertices_indices[i];
		beta_index = start_index + 2*i;
		m_osnap_poly.coors[v_index].px += beta[beta_index];
		m_osnap_poly.coors[v_index].py += beta[beta_index+1];
	}

	/*for(int i = 0; i < this->m_osnap_poly.coors.size(); ++i){
		int j = 2*i;
		this->m_osnap_poly.coors[i].px += beta[start_index + j];
		this->m_osnap_poly.coors[i].py += beta[start_index + j + 1];
	}*/
}

// ���º󶥵��λ����ԭ��λ�õľ���
double OSnap::f1(int vertex_index)
{
	//                                ��ǰ�����λ��                        ԭʼ�����λ��
	Eigen::Vector3d v_diff = m_osnap_poly.vertices[vertex_index] - m_osnap_poly.vertices_origin[vertex_index];
    return f1_weight * v_diff.norm();
}

// ���º󶥵�λ����Ŀ�궥��λ�õľ���
double OSnap::f2(int vertex_index_in_p2p_vertices)
{
	// ���º�Ķ���
	Eigen::Vector3d v_update, v_target, v_diff;
	int v_index = p2p_vertices_indices[vertex_index_in_p2p_vertices];
	v_update = m_osnap_poly.vertices[v_index];

	Vertex *pv = this->p2p_vertices[vertex_index_in_p2p_vertices]->neighbor_vertex;
	if(!pv){
		cerr << "OSnap::f2, pv == NULL!" << endl;
		return 0;
	}
	// Ŀ�궥��
	v_target << pv->point.x, pv->point.y, pv->point.z;

	v_diff = v_update - v_target;
    return f2_weight * v_diff.norm();
}

// dist[���º󶥵㣬Ŀ�궥���ǰ���ڵ�] - dist[Ŀ�궥�㣬Ŀ�궥���ǰ���ڵ�]
double OSnap::f3_1(int vertex_index_in_p2p_vertices)
{
	Eigen::Vector3d v_update, v_target, v_target_prev;
	int v_index = p2p_vertices_indices[vertex_index_in_p2p_vertices];
	v_update = m_osnap_poly.vertices[v_index];

	Vertex *pv = p2p_vertices[vertex_index_in_p2p_vertices]->neighbor_vertex;
	if(!pv){
		cerr << "OSnap::f3_1, pv == NULL!" << endl;
		return 0;
	}

	v_target << pv->point.x, pv->point.y, pv->point.z;
	v_target_prev << pv->prev_point->point.x,
		             pv->prev_point->point.y,
		             pv->prev_point->point.z;
	double dist1 = (v_update - v_target_prev).norm();
	double dist2 = (v_target - v_target_prev).norm();
	return f3_1_weight * abs(dist1-dist2);
}

// dist[���º󶥵㣬Ŀ�궥��ĺ�̽ڵ�] - dist[Ŀ�궥�㣬Ŀ�궥��ĺ�̽ڵ�]
double OSnap::f3_2(int vertex_index_in_p2p_vertices)
{
	Eigen::Vector3d v_update, v_target, v_target_next;
	int v_index = p2p_vertices_indices[vertex_index_in_p2p_vertices];
	v_update = m_osnap_poly.vertices[v_index];

	Vertex *pv = p2p_vertices[vertex_index_in_p2p_vertices]->neighbor_vertex;
	if(!pv){
		cerr << "OSnap::f3_2, pv == NULL!" << endl;
		return 0;
	}

	v_target << pv->point.x, pv->point.y, pv->point.z;
	v_target_next << pv->next_point->point.x,
		             pv->next_point->point.y,
		             pv->next_point->point.z;
	double dist1 = (v_update - v_target_next).norm();
	double dist2 = (v_target - v_target_next).norm();
	return f3_2_weight * abs(dist1-dist2);
}

// f5�Ķ�ż�汾�����º󶥵�λ��������ƽ�潻�ߵľ���
double OSnap::f8(int vertex_index_in_p2p_vertices)
{
	// ���º�Ķ���
	Eigen::Vector3d v_update;
	int v_index = p2p_vertices_indices[vertex_index_in_p2p_vertices];
	v_update = m_osnap_poly.vertices[v_index];

	// Ŀ������
	POLYGON *target_poly;

	int target_poly_id = p2p_vertices[vertex_index_in_p2p_vertices]->neighbor_poly_id;
	if(target_poly_id == -1){
		cerr << "in f8, target_poly_id == -1" << endl;
	}
	target_poly = &((*m_polygons)[target_poly_id]);

	// ��ȡ��������εĽ���
	Eigen::Vector3d p_base, line_dir;
	if(!getIntersectLine(m_poly->coeff, target_poly->coeff, p_base, line_dir)){
		cerr << "OSnap::f8, �����ƽ��!" << endl;
		return FLT_MAX;
	}

	Eigen::Vector3d v_p, delta;
	v_p = v_update;
	delta = p_base - v_p;
	double lambda = -1 * line_dir.dot(delta);

	Eigen::Vector3d p_proj;
	p_proj = p_base + lambda * line_dir;

    return f8_weight * (v_p - p_proj).norm();
}

// ���º󶥵�λ�õ�ƥ��ߵľ���
double OSnap::f4(int vertex_index_in_p2e_vertices)
{
	Eigen::Vector3d v_update, e_begin, e_end;

	int v_index = p2e_vertices_indices[vertex_index_in_p2e_vertices];
	v_update = m_osnap_poly.vertices[v_index];

	Vertex *pv = p2e_vertices[vertex_index_in_p2e_vertices]->neighbor_edge;
	if(!pv){
		cerr << "OSnap::f4, pv == NULL!" << endl;
		return 0;
	}

	e_begin << pv->point.x,
		       pv->point.y,
			   pv->point.z;
	e_end << pv->next_point->point.x,
		     pv->next_point->point.y,
			 pv->next_point->point.z;

	// ����v_update��line[e_begin, e_end]�ľ���
	Eigen::Vector3d p_base, line_dir;
	p_base = e_begin;
	line_dir = e_end-e_begin;
	line_dir.normalize();

	Eigen::Vector3d v_p, delta;
	v_p = v_update;
	delta = p_base - v_p;
	double lambda = -1 * line_dir.dot(delta);

	Eigen::Vector3d p_proj;
	p_proj = p_base + lambda * line_dir;

    return f4_weight * (v_p - p_proj).norm();
}

// ������������εĽ���
bool OSnap::getIntersectLine(Eigen::Vector4f &coeff1, Eigen::Vector4f &coeff2,
                             Eigen::Vector3d &p_base, Eigen::Vector3d &line_dir)
{
	 Eigen::Vector3d n1, n2;
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
    double min = coeff1[0];
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

// ���º󶥵�λ��������ƽ�潻�ߵľ���
double OSnap::f5(int vertex_index_in_p2e_vertices)
{
	// ���º�Ķ���
	Eigen::Vector3d v_update;
	int v_index = p2e_vertices_indices[vertex_index_in_p2e_vertices];
	v_update = m_osnap_poly.vertices[v_index];

	// Ŀ������
	POLYGON *target_poly;
	Vertex *pv = p2e_vertices[vertex_index_in_p2e_vertices]->neighbor_edge;
	if(!pv){
		cerr << "OSnap::f5, pv == NULL!" << endl;
		return 0;
	}

	int target_poly_id = floor(pv->point.data[3]+0.5f);
	target_poly = &((*m_polygons)[target_poly_id]);

	// ��ȡ��������εĽ���
	Eigen::Vector3d p_base, line_dir;
	if(!getIntersectLine(m_poly->coeff, target_poly->coeff, p_base, line_dir)){
		cerr << "OSnap::f5, �����ƽ��!" << endl;
		return FLT_MAX;
	}

	Eigen::Vector3d v_p, delta;
	v_p = v_update;
	delta = p_base - v_p;
	double lambda = -1 * line_dir.dot(delta);

	Eigen::Vector3d p_proj;
	p_proj = p_base + lambda * line_dir;

    return f5_weight * (v_p - p_proj).norm();
}

// ���º󶥵㵽ƥ��������˵����֮�� - ƥ��߳���
double OSnap::f6(int vertex_index_in_p2e_vertices)
{
	Eigen::Vector3d v_update, e_begin, e_end;
	// ���º�Ķ���
	int v_index = p2e_vertices_indices[vertex_index_in_p2e_vertices];
	v_update = m_osnap_poly.vertices[v_index];

	// ƥ���
	Vertex *pv = p2e_vertices[vertex_index_in_p2e_vertices]->neighbor_edge;
	if(!pv){
		cerr << "OSnap::f4, pv == NULL!" << endl;
		return 0;
	}

	// ƥ��ߵ������˵�
	e_begin << pv->point.x,
		       pv->point.y,
			   pv->point.z;
	e_end << pv->next_point->point.x,
		     pv->next_point->point.y,
			 pv->next_point->point.z;

	double dist1 = (v_update - e_begin).norm();
	double dist2 = (v_update - e_end).norm();
	double dist3 = (e_end - e_begin).norm();

	return f6_weight * abs(dist1 + dist2 - dist3);
}

// ���º󶥵㵽ƥ���ϵ����֧��ƽ��ľ���
double OSnap::f7(int vertex_index, Vertex *pv)
{
    //������ζ���
    Eigen::Vector4d homogeneous_vertex;
    homogeneous_vertex << m_osnap_poly.vertices[vertex_index][0],
                          m_osnap_poly.vertices[vertex_index][1],
                          m_osnap_poly.vertices[vertex_index][2],
                          1;

    // ��ȡƥ���ϵ��������ε�����
    int poly_index;
    if(pv->neighbor_poly_id == -1){
        cerr << "in OSnap::f7, pv->neighbor_poly_id == -1!" << endl;
        return 0;
    }

    poly_index = pv->neighbor_poly_id;

    // ���췽��ϵ��
    Eigen::Vector4d coeff;
    coeff << (*m_polygons)[poly_index].coeff[0],
             (*m_polygons)[poly_index].coeff[1],
             (*m_polygons)[poly_index].coeff[2],
             (*m_polygons)[poly_index].coeff[3];

    double dist = homogeneous_vertex.dot( coeff );
    return f7_weight * abs(dist);
}

// ��ʼ����������beta,��ÿ���Ż��Ŀ�ʼʹ��һ��
void OSnap::initBeta()
{
	int size = this->m_osnap_poly.vertices.size();
	if(size == 0){
		cerr << "in OSnap::initBeta(), m_osnap_poly.vertices->size() == 0" << endl;
		return;
	}

	// ǰ�������������ٶȣ����ٶ�
	// �����2*n��������n������ƥ���ϵ�Ķ��㣬����ÿ����������ھֲ�����ϵ��΢Сƫ��(delta_x, delta_y)
	size = 6 + (p2p_vertices.size() + p2e_vertices.size())*2;

	beta.resize(size);
	beta.setZero();	// ���������������е�Ԫ�ؾ���ʼ��Ϊ0
	for(int i = 0; i < beta.size(); ++i){
		beta[i] = 0.0001;
	}
}

// �����ƶ�����ϵ
void OSnap::updateMovingCoor()
{
	Eigen::Matrix3d R;
	Eigen::Vector3d pointOfA;
	Eigen::Vector3d a;
	Eigen::Vector3d a_;
	double pitch;
	double alpha;

	// ���ȴ�beta����ȡ���ٶ�c�����ٶ�c_
	/*m_osnap_poly.c << beta[0], beta[1], beta[2];
	m_osnap_poly.c_ << beta[3], beta[4], beta[5];*/
	m_osnap_poly.c[0] = beta[0];
	m_osnap_poly.c[1] = beta[1];
	m_osnap_poly.c[2] = beta[2];
	m_osnap_poly.c_[0] = beta[3];
	m_osnap_poly.c_[1] = beta[4];
	m_osnap_poly.c_[2] = beta[5];

	// ����c��c_�������������˶�����ر���
	setHelicalVars(m_osnap_poly.c, m_osnap_poly.c_, R, pointOfA, a, a_, pitch, alpha);

	// ���¾��������˶�����ƶ�����ϵ�Ļ�
	Eigen::Vector3d x_;
    uniformHelicalMove( m_osnap_poly.c, m_osnap_poly.c_, m_osnap_poly.b0, x_, R, pointOfA, a, pitch, alpha);
    m_osnap_poly.b0 = x_;
    uniformHelicalMove( m_osnap_poly.c, m_osnap_poly.c_, m_osnap_poly.b1, x_, R, pointOfA, a, pitch, alpha);
    m_osnap_poly.b1 = x_;
    uniformHelicalMove( m_osnap_poly.c, m_osnap_poly.c_, m_osnap_poly.b2, x_, R, pointOfA, a, pitch, alpha);
    m_osnap_poly.b2 = x_;	
}

// ���¶���λ��
void OSnap::updateVertices()
{
	// ���¶����ά����
	updateCoors();
	// ���¶���λ��
	updateVerticesAfterHelicalMove(this->m_osnap_poly);
}

// ����Jacob����
void OSnap::cosntructJacobMat()
{
	/*
	* û��ƥ���ϵ�Ķ����Ӧ��Ŀ�꺯��Ϊ��f1
    * ��-��ƥ��Ķ����Ӧ��Ŀ�꺯��Ϊ��f1, f2, f3_1, f3_2, f7, f8
    * ��-��ƥ��Ķ����Ӧ��Ŀ�꺯��Ϊ��f1, f4, f5, f6, f7
	*/
	int rows = normal_vertices.size() +
               p2p_vertices.size() * 6 +
               p2e_vertices.size() * 5;
	int cols = 6 + (p2p_vertices.size() + p2e_vertices.size()) * 2;

	J.resize(rows, cols);
	J.setZero();	// ����Ԫ������Ϊ0

	// ���ȴ���û��ƥ���ϵ�Ķ���
	// ��ƫ�����Ĳ���
	double delta = this->delta;

	int v_index;

	/*
	* û��ƥ���ϵ�Ķ���,�����ƶ�����ϵ�Ķ�ά���걣�ֲ��䣬�����ƶ�����ϵ�ı仯��ı����ǵ�λ�ã����������ԭ��λ�õľ��벻��̫Զ
	* Ŀ�꺯����f1
	* beta��ǰ��������
	*/
	for(int i = 0; i < normal_vertices.size(); ++i){		
		v_index = normal_vertices_indices[i];

		pd_f1_normal_vertices(v_index, J, i, delta);
	}	// û��ƥ���ϵ�Ķ���

	/*
	* ��-��ƥ���ϵ�Ķ���,�ƶ�����ϵ�п��ܱ䶯���䱾��������ƶ�����ϵ�Ķ�ά����Ҳ������һЩ��΢�ı䶯
    * Ŀ�꺯��: f1, f2, f3_1, f3_2, f7, f8������Ŀ�꺯��
	* beta: beta[0-5], beta[j],beta[j+1] ���˸�����
	*/
	int start_row_index = normal_vertices.size();	// ������
	int start_index = 6;	// ������
	for(int i = 0; i < p2p_vertices.size(); ++i)
	{
		v_index = p2p_vertices_indices[i];

        pd_f1_matching_vertices(v_index, J, start_row_index + 6*i,    start_index + 2*i, delta);
        pd_f2  (i, J,                       start_row_index + 6*i + 1, start_index + 2*i, delta);
        pd_f3_1(i, J,                       start_row_index + 6*i + 2, start_index + 2*i, delta);
        pd_f3_2(i, J,                       start_row_index + 6*i + 3, start_index + 2*i, delta);
        pd_f7(v_index, p2p_vertices[i], J,  start_row_index + 6*i + 4, start_index + 2*i, delta);
		pd_f8(i, J,                         start_row_index + 6*i + 5, start_index + 2*i, delta);
	}	// ��-��ƥ��

	/*
	* ��-��ƥ���ϵ�Ķ���,�ƶ�����ϵ�п��ܱ䶯���䱾��������ƶ�����ϵ�Ķ�ά����Ҳ������һЩ��΢�ı䶯
    * Ŀ�꺯��: f1, f4, f5, f6, f7 �����Ŀ�꺯��
	* beta: beta[0-5], beta[j],beta[j+1] ���˸�����
	*/
    start_row_index = normal_vertices.size() + 6*p2p_vertices.size();	// ������
	start_index = 6 + 2*p2p_vertices.size();	// ������
	for(int i = 0; i < p2e_vertices.size(); ++i){
		v_index = p2e_vertices_indices[i];

        pd_f1_matching_vertices(v_index, J, start_row_index + 5*i, start_index + 2*i, delta);
        pd_f4(i, J,                         start_row_index + 5*i + 1, start_index + 2*i, delta);
        pd_f5(i, J,                         start_row_index + 5*i + 2, start_index + 2*i, delta);
        pd_f6(i, J,                         start_row_index + 5*i + 3, start_index + 2*i, delta);
        pd_f7(v_index, p2e_vertices[i], J,  start_row_index + 5*i + 4, start_index + 2*i, delta);
	}	// ��-��ƥ��
}

// ִ�е�����˹ţ�ٵ������Ӷ�����beta
void OSnap::performGN()
{
	Eigen::MatrixXd A;
	Eigen::VectorXd b, x;
	int n = 6 + 2*(p2p_vertices.size() + p2e_vertices.size());
	A.resize(n, n);
	A.setZero();

	A = J.transpose()*J;

	b.resize(n);
	b.setZero();
	x.resize(n);
	x.setZero();

	Eigen::VectorXd r;
	fill_r(r);
    double sum_r = 0;
    for(int i = 0; i < r.size(); ++i){sum_r += r[i];}
    cout << "sum(r1+...+rn) = " << sum_r << endl;
	//cout << "r = \n" << r << endl;
	
	b = J.transpose() * r;

	//x = A.fullPivHouseholderQr().solve(b);
	//x = A.householderQr().solve(b);
	//conjugateGradient(A, x, b);

	// ����beta��ֵ
    Eigen::VectorXd beta_bck /*����beta�ĳ�ֵ*/, beta_opt/*����beta������ֵ*/;
	beta_bck.resize(beta.size());
	beta_bck = beta;
	beta_opt.resize(beta.size());

	int k_count = this->max_k_count;

    int k_count_valid = -1;  // �������ŵĵ�������

	for(int i = 1; i <= k_count; ++i){
        this->conjugateGradient_with_max_k(A, x, b, i); // �����ݶȵ���i��
        beta = beta-x;  // ����beta
        fill_r(r);      // ����Ŀ�꺯��

		double sum_r_tmp = 0;
		for(int j = 0; j < r.size(); ++j){sum_r_tmp += r[j];}

		if(sum_r_tmp < sum_r){	// sum_r ���浱ǰ��С��Ŀ�꺯��ֵ
			sum_r = sum_r_tmp;
			k_count_valid = i;
			beta_opt = beta;
		}

		// ����beta;
		beta = beta_bck;
	}

	cout << "k_count_valid = " << k_count_valid << endl;
	cout << "sum_r = " << sum_r << endl;

	// ��beta����Ϊ����ֵ
	if(k_count_valid > 0)
		beta = beta_opt;
	// ����Ŀ�꺯��
    fill_r(r);

	sum_r = 0;
	for(int i = 0; i < r.size(); ++i){sum_r += r[i];}
    cout << "sum_r = " << sum_r << endl;

	//beta = beta - x;

	//beta = beta - (J.transpose()*J).inverse()*J.transpose()*r;
}

void OSnap::fill_r(Eigen::VectorXd &r)
{
	// ����r
	int m = normal_vertices.size() + 6*p2p_vertices.size() + 5*p2e_vertices.size();
	r.resize(m);
	r.setZero();

	// ���������˶�����ر���
	Eigen::Vector3d c;
	Eigen::Vector3d c_;
	Eigen::Matrix3d R;
	Eigen::Vector3d pointOfA;
	Eigen::Vector3d a;
	Eigen::Vector3d a_;
	double pitch;
	double alpha;

	// �ƶ�����ϵ�Ļ�
	// ԭ�㣺b0
	// ��������������b1-b0, b2-b0
	Eigen::Vector3d b0, b1, b2;

	// ����beta���ý��ٶ������ٶ�
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	// ����c��c_���������˶�����
	this->setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	// ������ϵ�Ļ�ִ�����������˶�
	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	int v_index;

	// û��ƥ���ϵ�Ķ���
	for(int i = 0; i < normal_vertices.size(); ++i){
		v_index = normal_vertices_indices[i];
		m_osnap_poly.vertices[v_index] = b0 + m_osnap_poly.coors[v_index].px * (b1 - b0)
                                            + m_osnap_poly.coors[v_index].py * (b2 - b0);
		r[i] = f1(v_index);
	}

	// ��-��
	int start_row_index = normal_vertices.size();	// ������
	int start_index = 6;	// ������
	double px, py;
	for(int i = 0; i < p2p_vertices.size(); ++i)
	{
		v_index = p2p_vertices_indices[i];

		px = m_osnap_poly.coors[v_index].px + beta[start_index + 2*i];
		py = m_osnap_poly.coors[v_index].py + beta[start_index + 2*i + 1];

		m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

        r[start_row_index + 6*i] = f1(v_index);
        r[start_row_index + 6*i + 1] = f2(i);
        r[start_row_index + 6*i + 2] = f3_1(i);
        r[start_row_index + 6*i + 3] = f3_2(i);
        r[start_row_index + 6*i + 4] = f7(v_index, p2p_vertices[i]);
		r[start_row_index + 6*i + 5] = f8(i);
	}

	// ��-��
    start_row_index = normal_vertices.size() + 6*p2p_vertices.size();	// ������
	start_index = 6 + 2*p2p_vertices.size();	// ������
	for(int i = 0; i < p2e_vertices.size(); ++i){
		v_index = p2e_vertices_indices[i];

		px = m_osnap_poly.coors[v_index].px + beta[start_index + 2*i];
		py = m_osnap_poly.coors[v_index].py + beta[start_index + 2*i + 1];

		m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

        r[start_row_index + 5*i] = f1(v_index);
        r[start_row_index + 5*i + 1] = f4(i);
        r[start_row_index + 5*i + 2] = f5(i);
        r[start_row_index + 5*i + 3] = f6(i);
        r[start_row_index + 5*i + 4] = f7(v_index, p2e_vertices[i]);
	}	// ��-��ƥ��
}

// �����ݶȽⷽ����
void OSnap::conjugateGradient(Eigen::MatrixXd &A, Eigen::VectorXd &x, Eigen::VectorXd &b)
{
	//cout << "A.diagonal() = " << A.diagonal() << endl;
	cout << "A.inverse() = \n" << A.inverse() << endl;
	cout << "A.inverse()*A = \n" << A.inverse()*A << endl;

	int n = b.size();
	x.resize(n);
	x.setZero();	// x�ĳ�ֵ��Ϊ0

	Eigen::VectorXd r, r_, p;
	r.resize(n);
	p.resize(n);
	r_.resize(n);
	
	r = b - A*x;
	p = r;

	int k = 0;
	double alpha, beta;
	do{
		if(k >= 10000){
			cout << "k == 10000, in conjugateGradient()" << endl;
			break;
		}

		alpha = r.dot(r) / (p.transpose()*A*p);
		x = x + alpha * p;
		r_ = r - alpha * (A * p);

        /*if(r.norm() <= this->max_r_norm)	// ��ֵ�������õ�ԽСԽ�ã���ֵԽС����������Խ�࣬�������������
			break;*/

		beta = r_.dot(r_) / r.dot(r);
		p = r_ + beta*p;
		r = r_;
		++k;
	}while(true);

	cout << "k = " << k << endl;
}

void OSnap::conjugateGradient_with_max_k(Eigen::MatrixXd &A, Eigen::VectorXd &x, Eigen::VectorXd &b, int k_count)
{
	int n = b.size();
	x.resize(n);
	x.setZero();	// x�ĳ�ֵ��Ϊ0

	Eigen::VectorXd r, r_, p;
	r.resize(n);
	p.resize(n);
	r_.resize(n);
	
	r = b - A*x;
	p = r;

	double alpha, beta;

	for(int k = 0; k < k_count; ++k){
		alpha = r.dot(r) / (p.transpose()*A*p);
		x = x + alpha * p;
		r_ = r - alpha * (A * p);

		beta = r_.dot(r_) / r.dot(r);
		p = r_ + beta*p;
		r = r_;
	}
}

// ���յ��Ż�ִ�к���
void OSnap::optimise()
{

}

// Ŀ�꺯����ƫ����
// ���û��ƥ���ϵ�Ķ��㣺normal_vertices
void OSnap::pd_f1_normal_vertices(int v_index, Eigen::MatrixXd &J, int row, double delta)
{
	// �������betaǰ����Ԫ�ظ��µĵ�ǰf1ֵ
	// ���ȼ���û�и���ʱ��f1ֵ
	// ���������˶�����ر���
	Eigen::Vector3d c;
	Eigen::Vector3d c_;
	Eigen::Matrix3d R;
	Eigen::Vector3d pointOfA;
	Eigen::Vector3d a;
	Eigen::Vector3d a_;
	double pitch;
	double alpha;

	// �ƶ�����ϵ�Ļ�
	// ԭ�㣺b0
	// ��������������b1-b0, b2-b0
	Eigen::Vector3d b0, b1, b2;

	// ����beta���ý��ٶ������ٶ�
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	// ����c��c_���������˶�����
	this->setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	// ������ϵ�Ļ�ִ�����������˶�
	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	// ����m_osnap_poly.vertices[v_index]
	m_osnap_poly.vertices[v_index] = b0 + m_osnap_poly.coors[v_index].px * (b1 - b0)
                                        + m_osnap_poly.coors[v_index].py * (b2 - b0);

	// ����f1(beta)
	double f1_val = this->f1(v_index);	// û�иı��κ�beta������f1ֵ
	double f1_delta_val;				// �ı�һ��beta������f1ֵ
	double pd;							// ƫ����

	// �������f1(beta)���betaǰ����������ƫ����
	// beta[0]///////////////////////////////////
	c << beta[0] + delta, beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	m_osnap_poly.vertices[v_index] = b0 + m_osnap_poly.coors[v_index].px * (b1 - b0)
                                        + m_osnap_poly.coors[v_index].py * (b2 - b0);

	f1_delta_val = f1(v_index);
	pd = (f1_delta_val - f1_val) / delta;
	J(row, 0) = pd;

	// beta[1]///////////////////////////////////
	c << beta[0], beta[1] + delta, beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	m_osnap_poly.vertices[v_index] = b0 + m_osnap_poly.coors[v_index].px * (b1 - b0)
                                        + m_osnap_poly.coors[v_index].py * (b2 - b0);

	f1_delta_val = f1(v_index);
	pd = (f1_delta_val - f1_val) / delta;
	J(row, 1) = pd;

	// beta[2]///////////////////////////////////
	c << beta[0], beta[1], beta[2] + delta;
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	m_osnap_poly.vertices[v_index] = b0 + m_osnap_poly.coors[v_index].px * (b1 - b0)
                                        + m_osnap_poly.coors[v_index].py * (b2 - b0);

	f1_delta_val = f1(v_index);
	pd = (f1_delta_val - f1_val) / delta;
	J(row, 2) = pd;

	// beta[3]///////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3] + delta, beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	m_osnap_poly.vertices[v_index] = b0 + m_osnap_poly.coors[v_index].px * (b1 - b0)
                                        + m_osnap_poly.coors[v_index].py * (b2 - b0);

	f1_delta_val = f1(v_index);
	pd = (f1_delta_val - f1_val) / delta;
	J(row, 3) = pd;

	// beta[4]///////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4] + delta, beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	m_osnap_poly.vertices[v_index] = b0 + m_osnap_poly.coors[v_index].px * (b1 - b0)
                                        + m_osnap_poly.coors[v_index].py * (b2 - b0);

	f1_delta_val = f1(v_index);
	pd = (f1_delta_val - f1_val) / delta;
	J(row, 4) = pd;

	// beta[5]///////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5] + delta;

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	m_osnap_poly.vertices[v_index] = b0 + m_osnap_poly.coors[v_index].px * (b1 - b0)
                                        + m_osnap_poly.coors[v_index].py * (b2 - b0);

	f1_delta_val = f1(v_index);
	pd = (f1_delta_val - f1_val) / delta;
	J(row, 5) = pd;
}

// �����ƥ���ϵ�Ķ���: p2p_vertices �� p2e_vertices������Ҫ��������ϵ������Ҫ���¶����ά����
void OSnap::pd_f1_matching_vertices(int v_index, Eigen::MatrixXd &J, int row, int beta_index, double delta)
{
	// �������betaǰ����Ԫ�ظ��µĵ�ǰf1ֵ�Լ�beta[beta_index],beta[beta_index+1]����Ԫ�ظ��º��f1ֵ
	// ���ȼ���û�и���ʱ��f1ֵ
	// ���������˶�����ر���
	Eigen::Vector3d c;
	Eigen::Vector3d c_;
	Eigen::Matrix3d R;
	Eigen::Vector3d pointOfA;
	Eigen::Vector3d a;
	Eigen::Vector3d a_;
	double pitch;
	double alpha;

	// �ƶ�����ϵ�Ļ�
	// ԭ�㣺b0
	// ��������������b1-b0, b2-b0
	Eigen::Vector3d b0, b1, b2;

	// ����beta���ý��ٶ������ٶ�
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	// ����c��c_���������˶�����
	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	// ������ϵ�Ļ�ִ�����������˶�
	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	// ����beta[beta_index],beta[beta_index+1]���ø��º�Ķ�ά����
	double px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	double py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	// ����m_osnap_poly.vertices[v_index]
	// b0, b1, b2, px, py ���ѱ�����
	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	// ����f1(beta)
	double f1_val = this->f1(v_index);	// û�иı��κ�beta������f1ֵ
	double f1_delta_val;				// �ı�һ��beta������f1ֵ
	double pd;							// ƫ����

	// beta[0]//////////////////////////////////////////////////////////////////
	c << beta[0] + delta, beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f1_delta_val = f1(v_index);
	pd = (f1_delta_val - f1_val) / delta;
	J(row, 0) = pd;

	// beta[1]//////////////////////////////////////////////////////////////////
	c << beta[0], beta[1] + delta, beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f1_delta_val = f1(v_index);
	pd = (f1_delta_val - f1_val) / delta;
	J(row, 1) = pd;

	// beta[2]//////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2] + delta;
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f1_delta_val = f1(v_index);
	pd = (f1_delta_val - f1_val) / delta;
	J(row, 2) = pd;

	// beta[3]//////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3] + delta, beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f1_delta_val = f1(v_index);
	pd = (f1_delta_val - f1_val) / delta;
	J(row, 3) = pd;

	// beta[4]//////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4] + delta, beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f1_delta_val = f1(v_index);
	pd = (f1_delta_val - f1_val) / delta;
	J(row, 4) = pd;

	// beta[5]//////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5] + delta;

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f1_delta_val = f1(v_index);
	pd = (f1_delta_val - f1_val) / delta;
	J(row, 5) = pd;

	// beta[beta_index]//////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index] + delta;
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f1_delta_val = f1(v_index);
	pd = (f1_delta_val - f1_val) / delta;
	J(row, beta_index) = pd;

	// beta[beta_index + 1]//////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1] + delta;

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f1_delta_val = f1(v_index);
	pd = (f1_delta_val - f1_val) / delta;
	J(row, beta_index + 1) = pd;
}

// ��Ե�-��
// f2��ƫ����
void OSnap::pd_f2(int vertex_index_in_p2p_vertices, Eigen::MatrixXd &J, int row, int beta_index, double delta)
{
	// ������vertices�е�����v_index
	int v_index = p2p_vertices_indices[vertex_index_in_p2p_vertices];

	// ���ȼ���û�и���ʱ��f2ֵ
	// ���������˶�����ر���
	Eigen::Vector3d c;
	Eigen::Vector3d c_;
	Eigen::Matrix3d R;
	Eigen::Vector3d pointOfA;
	Eigen::Vector3d a;
	Eigen::Vector3d a_;
	double pitch;
	double alpha;

	// �ƶ�����ϵ�Ļ�
	// ԭ�㣺b0
	// ��������������b1-b0, b2-b0
	Eigen::Vector3d b0, b1, b2;

	// ����beta���ý��ٶ������ٶ�
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	// ����c��c_���������˶�����
	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	// ������ϵ�Ļ�ִ�����������˶�
	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	// ����beta[beta_index],beta[beta_index+1]���ø��º�Ķ�ά����
	double px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	double py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	// ����m_osnap_poly.vertices[v_index]
	// b0, b1, b2, px, py ���ѱ�����
	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	// ����f2(beta)
	double f2_val = f2(vertex_index_in_p2p_vertices);	// û�иı��κ�beta������f2ֵ
	double f2_delta_val;								// �ı�һ��beta������f2ֵ
	double pd;											// ƫ����

	// beta[0]///////////////////////////////////////////////////////////////////////
	c << beta[0] + delta, beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f2_delta_val = f2(vertex_index_in_p2p_vertices);
	pd = (f2_delta_val - f2_val) / delta;
	J(row, 0) = pd;

	// beta[1]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1] + delta, beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f2_delta_val = f2(vertex_index_in_p2p_vertices);
	pd = (f2_delta_val - f2_val) / delta;
	J(row, 1) = pd;

	// beta[2]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2] + delta;
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f2_delta_val = f2(vertex_index_in_p2p_vertices);
	pd = (f2_delta_val - f2_val) / delta;
	J(row, 2) = pd;

	// beta[3]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3] + delta, beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f2_delta_val = f2(vertex_index_in_p2p_vertices);
	pd = (f2_delta_val - f2_val) / delta;
	J(row, 3) = pd;

	// beta[4]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4] + delta, beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f2_delta_val = f2(vertex_index_in_p2p_vertices);
	pd = (f2_delta_val - f2_val) / delta;
	J(row, 4) = pd;

	// beta[5]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5] + delta;

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f2_delta_val = f2(vertex_index_in_p2p_vertices);
	pd = (f2_delta_val - f2_val) / delta;
	J(row, 5) = pd;

	// beta[beta_index]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index] + delta;
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f2_delta_val = f2(vertex_index_in_p2p_vertices);
	pd = (f2_delta_val - f2_val) / delta;
	J(row, beta_index) = pd;

	// beta[beta_index+1]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1] + delta;

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f2_delta_val = f2(vertex_index_in_p2p_vertices);
	pd = (f2_delta_val - f2_val) / delta;
	J(row, beta_index+1) = pd;
}

// ��Ե�-��
// f3_1��ƫ����
void OSnap::pd_f3_1(int vertex_index_in_p2p_vertices, Eigen::MatrixXd &J, int row, int beta_index, double delta)
{
	// ������vertices�е�����v_index
	int v_index = p2p_vertices_indices[vertex_index_in_p2p_vertices];

	// ���ȼ���û�и���ʱ��f3_1ֵ
	// ���������˶�����ر���
	Eigen::Vector3d c;
	Eigen::Vector3d c_;
	Eigen::Matrix3d R;
	Eigen::Vector3d pointOfA;
	Eigen::Vector3d a;
	Eigen::Vector3d a_;
	double pitch;
	double alpha;

	// �ƶ�����ϵ�Ļ�
	// ԭ�㣺b0
	// ��������������b1-b0, b2-b0
	Eigen::Vector3d b0, b1, b2;

	// ����beta���ý��ٶ������ٶ�
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	// ����c��c_���������˶�����
	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	// ������ϵ�Ļ�ִ�����������˶�
	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	// ����beta[beta_index],beta[beta_index+1]���ø��º�Ķ�ά����
	double px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	double py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	// ����m_osnap_poly.vertices[v_index]
	// b0, b1, b2, px, py ���ѱ�����
	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	// ����f3_1(beta)
	double f3_1_val = f3_1(vertex_index_in_p2p_vertices);	// û�иı��κ�beta������f3_1ֵ
	double f3_1_delta_val;								// �ı�һ��beta������f3_1ֵ
	double pd;											// ƫ����

	// beta[0]///////////////////////////////////////////////////////////////////////
	c << beta[0] + delta, beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f3_1_delta_val = f3_1(vertex_index_in_p2p_vertices);
	pd = (f3_1_delta_val - f3_1_val) / delta;
	J(row, 0) = pd;

	// beta[1]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1] + delta, beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f3_1_delta_val = f3_1(vertex_index_in_p2p_vertices);
	pd = (f3_1_delta_val - f3_1_val) / delta;
	J(row, 1) = pd;

	// beta[2]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2] + delta;
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f3_1_delta_val = f3_1(vertex_index_in_p2p_vertices);
	pd = (f3_1_delta_val - f3_1_val) / delta;
	J(row, 2) = pd;

	// beta[3]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3] + delta, beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f3_1_delta_val = f3_1(vertex_index_in_p2p_vertices);
	pd = (f3_1_delta_val - f3_1_val) / delta;
	J(row, 3) = pd;

	// beta[4]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4] + delta, beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f3_1_delta_val = f3_1(vertex_index_in_p2p_vertices);
	pd = (f3_1_delta_val - f3_1_val) / delta;
	J(row, 4) = pd;

	// beta[5]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5] + delta;

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f3_1_delta_val = f3_1(vertex_index_in_p2p_vertices);
	pd = (f3_1_delta_val - f3_1_val) / delta;
	J(row, 5) = pd;

	// beta[beta_index]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index] + delta;
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f3_1_delta_val = f3_1(vertex_index_in_p2p_vertices);
	pd = (f3_1_delta_val - f3_1_val) / delta;
	J(row, beta_index) = pd;

	// beta[beta_index+1]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1] + delta;

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f3_1_delta_val = f3_1(vertex_index_in_p2p_vertices);
	pd = (f3_1_delta_val - f3_1_val) / delta;
	J(row, beta_index+1) = pd;
}

// ��Ե�-��
// f3_2��ƫ����
void OSnap::pd_f3_2(int vertex_index_in_p2p_vertices, Eigen::MatrixXd &J, int row, int beta_index, double delta)
{
	// ������vertices�е�����v_index
	int v_index = p2p_vertices_indices[vertex_index_in_p2p_vertices];

	// ���ȼ���û�и���ʱ��f3_2ֵ
	// ���������˶�����ر���
	Eigen::Vector3d c;
	Eigen::Vector3d c_;
	Eigen::Matrix3d R;
	Eigen::Vector3d pointOfA;
	Eigen::Vector3d a;
	Eigen::Vector3d a_;
	double pitch;
	double alpha;

	// �ƶ�����ϵ�Ļ�
	// ԭ�㣺b0
	// ��������������b1-b0, b2-b0
	Eigen::Vector3d b0, b1, b2;

	// ����beta���ý��ٶ������ٶ�
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	// ����c��c_���������˶�����
	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	// ������ϵ�Ļ�ִ�����������˶�
	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	// ����beta[beta_index],beta[beta_index+1]���ø��º�Ķ�ά����
	double px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	double py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	// ����m_osnap_poly.vertices[v_index]
	// b0, b1, b2, px, py ���ѱ�����
	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	// ����f3_2(beta)
	double f3_2_val = f3_2(vertex_index_in_p2p_vertices);	// û�иı��κ�beta������f3_2ֵ
	double f3_2_delta_val;								// �ı�һ��beta������f3_2ֵ
	double pd;											// ƫ����

	// beta[0]///////////////////////////////////////////////////////////////////////
	c << beta[0] + delta, beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f3_2_delta_val = f3_2(vertex_index_in_p2p_vertices);
	pd = (f3_2_delta_val - f3_2_val) / delta;
	J(row, 0) = pd;

	// beta[1]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1] + delta, beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f3_2_delta_val = f3_2(vertex_index_in_p2p_vertices);
	pd = (f3_2_delta_val - f3_2_val) / delta;
	J(row, 1) = pd;

	// beta[2]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2] + delta;
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f3_2_delta_val = f3_2(vertex_index_in_p2p_vertices);
	pd = (f3_2_delta_val - f3_2_val) / delta;
	J(row, 2) = pd;

	// beta[3]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3] + delta, beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f3_2_delta_val = f3_2(vertex_index_in_p2p_vertices);
	pd = (f3_2_delta_val - f3_2_val) / delta;
	J(row, 3) = pd;

	// beta[4]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4] + delta, beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f3_2_delta_val = f3_2(vertex_index_in_p2p_vertices);
	pd = (f3_2_delta_val - f3_2_val) / delta;
	J(row, 4) = pd;

	// beta[5]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5] + delta;

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f3_2_delta_val = f3_2(vertex_index_in_p2p_vertices);
	pd = (f3_2_delta_val - f3_2_val) / delta;
	J(row, 5) = pd;

	// beta[beta_index]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index] + delta;
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f3_2_delta_val = f3_2(vertex_index_in_p2p_vertices);
	pd = (f3_2_delta_val - f3_2_val) / delta;
	J(row, beta_index) = pd;

	// beta[beta_index+1]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1] + delta;

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f3_2_delta_val = f3_2(vertex_index_in_p2p_vertices);
	pd = (f3_2_delta_val - f3_2_val) / delta;
	J(row, beta_index+1) = pd;
}

// ��Ե�-��
// f8��ƫ����
void OSnap::pd_f8(int vertex_index_in_p2p_vertices, Eigen::MatrixXd &J, int row, int beta_index, double delta)
{
	// ������vertices�е�����v_index
	int v_index = p2p_vertices_indices[vertex_index_in_p2p_vertices];

	// ���ȼ���û�и���ʱ��f8ֵ
	// ���������˶�����ر���
	Eigen::Vector3d c;
	Eigen::Vector3d c_;
	Eigen::Matrix3d R;
	Eigen::Vector3d pointOfA;
	Eigen::Vector3d a;
	Eigen::Vector3d a_;
	double pitch;
	double alpha;

	// �ƶ�����ϵ�Ļ�
	// ԭ�㣺b0
	// ��������������b1-b0, b2-b0
	Eigen::Vector3d b0, b1, b2;

	// ����beta���ý��ٶ������ٶ�
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	// ����c��c_���������˶�����
	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	// ������ϵ�Ļ�ִ�����������˶�
	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	// ����beta[beta_index],beta[beta_index+1]���ø��º�Ķ�ά����
	double px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	double py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	// ����m_osnap_poly.vertices[v_index]
	// b0, b1, b2, px, py ���ѱ�����
	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	// ����f8(beta)
	double f8_val = f8(vertex_index_in_p2p_vertices);	// û�иı��κ�beta������f8ֵ
	double f8_delta_val;								// �ı�һ��beta������f8ֵ
	double pd;											// ƫ����

	// beta[0]///////////////////////////////////////////////////////////////////////
	c << beta[0] + delta, beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f8_delta_val = f8(vertex_index_in_p2p_vertices);
	pd = (f8_delta_val - f8_val) / delta;
	J(row, 0) = pd;

	// beta[1]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1] + delta, beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f8_delta_val = f8(vertex_index_in_p2p_vertices);
	pd = (f8_delta_val - f8_val) / delta;
	J(row, 1) = pd;

	// beta[2]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2] + delta;
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f8_delta_val = f8(vertex_index_in_p2p_vertices);
	pd = (f8_delta_val - f8_val) / delta;
	J(row, 2) = pd;

	// beta[3]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3] + delta, beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f8_delta_val = f8(vertex_index_in_p2p_vertices);
	pd = (f8_delta_val - f8_val) / delta;
	J(row, 3) = pd;

	// beta[4]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4] + delta, beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f8_delta_val = f8(vertex_index_in_p2p_vertices);
	pd = (f8_delta_val - f8_val) / delta;
	J(row, 4) = pd;

	// beta[5]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5] + delta;

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f8_delta_val = f8(vertex_index_in_p2p_vertices);
	pd = (f8_delta_val - f8_val) / delta;
	J(row, 5) = pd;

	// beta[beta_index]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index] + delta;
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f8_delta_val = f8(vertex_index_in_p2p_vertices);
	pd = (f8_delta_val - f8_val) / delta;
	J(row, beta_index) = pd;

	// beta[beta_index+1]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1] + delta;

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f8_delta_val = f8(vertex_index_in_p2p_vertices);
	pd = (f8_delta_val - f8_val) / delta;
	J(row, beta_index+1) = pd;
}

// ��Ե�-��
// f4��ƫ����
void OSnap::pd_f4(int vertex_index_in_p2e_vertices, Eigen::MatrixXd &J, int row, int beta_index, double delta)
{
	// ������vertices�е�����v_index
	int v_index = p2e_vertices_indices[vertex_index_in_p2e_vertices];

	// ���ȼ���û�и���ʱ��f4ֵ
	// ���������˶�����ر���
	Eigen::Vector3d c;
	Eigen::Vector3d c_;
	Eigen::Matrix3d R;
	Eigen::Vector3d pointOfA;
	Eigen::Vector3d a;
	Eigen::Vector3d a_;
	double pitch;
	double alpha;

	// �ƶ�����ϵ�Ļ�
	// ԭ�㣺b0
	// ��������������b1-b0, b2-b0
	Eigen::Vector3d b0, b1, b2;

	// ����beta���ý��ٶ������ٶ�
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	// ����c��c_���������˶�����
	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	// ������ϵ�Ļ�ִ�����������˶�
	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	// ����beta[beta_index],beta[beta_index+1]���ø��º�Ķ�ά����
	double px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	double py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	// ����m_osnap_poly.vertices[v_index]
	// b0, b1, b2, px, py ���ѱ�����
	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	// ����f4(beta)
	double f4_val = f4(vertex_index_in_p2e_vertices);	// û�иı��κ�beta������f4ֵ
	double f4_delta_val;								// �ı�һ��beta������f4ֵ
	double pd;											// ƫ����

	// beta[0]///////////////////////////////////////////////////////////////////////
	c << beta[0] + delta, beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f4_delta_val = f4(vertex_index_in_p2e_vertices);
	pd = (f4_delta_val - f4_val) / delta;
	J(row, 0) = pd;

	// beta[1]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1] + delta, beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f4_delta_val = f4(vertex_index_in_p2e_vertices);
	pd = (f4_delta_val - f4_val) / delta;
	J(row, 1) = pd;

	// beta[2]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2] + delta;
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f4_delta_val = f4(vertex_index_in_p2e_vertices);
	pd = (f4_delta_val - f4_val) / delta;
	J(row, 2) = pd;

	// beta[3]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3] + delta, beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f4_delta_val = f4(vertex_index_in_p2e_vertices);
	pd = (f4_delta_val - f4_val) / delta;
	J(row, 3) = pd;

	// beta[4]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4] + delta, beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f4_delta_val = f4(vertex_index_in_p2e_vertices);
	pd = (f4_delta_val - f4_val) / delta;
	J(row, 4) = pd;

	// beta[5]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5] + delta;

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f4_delta_val = f4(vertex_index_in_p2e_vertices);
	pd = (f4_delta_val - f4_val) / delta;
	J(row, 5) = pd;

	// beta[beta_index]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index] + delta;
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f4_delta_val = f4(vertex_index_in_p2e_vertices);
	pd = (f4_delta_val - f4_val) / delta;
	J(row, beta_index) = pd;

	// beta[beta_index+1]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1] + delta;

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f4_delta_val = f4(vertex_index_in_p2e_vertices);
	pd = (f4_delta_val - f4_val) / delta;
	J(row, beta_index+1) = pd;
}

// ��Ե�-��
// f5��ƫ����
void OSnap::pd_f5(int vertex_index_in_p2e_vertices, Eigen::MatrixXd &J, int row, int beta_index, double delta)
{
	// ������vertices�е�����v_index
	int v_index = p2e_vertices_indices[vertex_index_in_p2e_vertices];

	// ���ȼ���û�и���ʱ��f5ֵ
	// ���������˶�����ر���
	Eigen::Vector3d c;
	Eigen::Vector3d c_;
	Eigen::Matrix3d R;
	Eigen::Vector3d pointOfA;
	Eigen::Vector3d a;
	Eigen::Vector3d a_;
	double pitch;
	double alpha;

	// �ƶ�����ϵ�Ļ�
	// ԭ�㣺b0
	// ��������������b1-b0, b2-b0
	Eigen::Vector3d b0, b1, b2;

	// ����beta���ý��ٶ������ٶ�
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	// ����c��c_���������˶�����
	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	// ������ϵ�Ļ�ִ�����������˶�
	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	// ����beta[beta_index],beta[beta_index+1]���ø��º�Ķ�ά����
	double px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	double py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	// ����m_osnap_poly.vertices[v_index]
	// b0, b1, b2, px, py ���ѱ�����
	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	// ����f5(beta)
	double f5_val = f5(vertex_index_in_p2e_vertices);	// û�иı��κ�beta������f5ֵ
	double f5_delta_val;								// �ı�һ��beta������f5ֵ
	double pd;											// ƫ����

	// beta[0]///////////////////////////////////////////////////////////////////////
	c << beta[0] + delta, beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f5_delta_val = f5(vertex_index_in_p2e_vertices);
	pd = (f5_delta_val - f5_val) / delta;
	J(row, 0) = pd;

	// beta[1]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1] + delta, beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f5_delta_val = f5(vertex_index_in_p2e_vertices);
	pd = (f5_delta_val - f5_val) / delta;
	J(row, 1) = pd;

	// beta[2]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2] + delta;
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f5_delta_val = f5(vertex_index_in_p2e_vertices);
	pd = (f5_delta_val - f5_val) / delta;
	J(row, 2) = pd;

	// beta[3]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3] + delta, beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f5_delta_val = f5(vertex_index_in_p2e_vertices);
	pd = (f5_delta_val - f5_val) / delta;
	J(row, 3) = pd;

	// beta[4]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4] + delta, beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f5_delta_val = f5(vertex_index_in_p2e_vertices);
	pd = (f5_delta_val - f5_val) / delta;
	J(row, 4) = pd;

	// beta[5]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5] + delta;

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f5_delta_val = f5(vertex_index_in_p2e_vertices);
	pd = (f5_delta_val - f5_val) / delta;
	J(row, 5) = pd;

	// beta[beta_index]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index] + delta;
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f5_delta_val = f5(vertex_index_in_p2e_vertices);
	pd = (f5_delta_val - f5_val) / delta;
	J(row, beta_index) = pd;

	// beta[beta_index+1]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1] + delta;

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f5_delta_val = f5(vertex_index_in_p2e_vertices);
	pd = (f5_delta_val - f5_val) / delta;
	J(row, beta_index+1) = pd;
}

// ��Ե�-��
// f6��ƫ����
void OSnap::pd_f6(int vertex_index_in_p2e_vertices, Eigen::MatrixXd &J, int row, int beta_index, double delta)
{
	// ������vertices�е�����v_index
	int v_index = p2e_vertices_indices[vertex_index_in_p2e_vertices];

	// ���ȼ���û�и���ʱ��f6ֵ
	// ���������˶�����ر���
	Eigen::Vector3d c;
	Eigen::Vector3d c_;
	Eigen::Matrix3d R;
	Eigen::Vector3d pointOfA;
	Eigen::Vector3d a;
	Eigen::Vector3d a_;
	double pitch;
	double alpha;

	// �ƶ�����ϵ�Ļ�
	// ԭ�㣺b0
	// ��������������b1-b0, b2-b0
	Eigen::Vector3d b0, b1, b2;

	// ����beta���ý��ٶ������ٶ�
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	// ����c��c_���������˶�����
	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	// ������ϵ�Ļ�ִ�����������˶�
	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	// ����beta[beta_index],beta[beta_index+1]���ø��º�Ķ�ά����
	double px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	double py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	// ����m_osnap_poly.vertices[v_index]
	// b0, b1, b2, px, py ���ѱ�����
	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	// ����f6(beta)
	double f6_val = f6(vertex_index_in_p2e_vertices);	// û�иı��κ�beta������f6ֵ
	double f6_delta_val;								// �ı�һ��beta������f6ֵ
	double pd;											// ƫ����

	// beta[0]///////////////////////////////////////////////////////////////////////
	c << beta[0] + delta, beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f6_delta_val = f6(vertex_index_in_p2e_vertices);
	pd = (f6_delta_val - f6_val) / delta;
	J(row, 0) = pd;

	// beta[1]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1] + delta, beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f6_delta_val = f6(vertex_index_in_p2e_vertices);
	pd = (f6_delta_val - f6_val) / delta;
	J(row, 1) = pd;

	// beta[2]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2] + delta;
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f6_delta_val = f6(vertex_index_in_p2e_vertices);
	pd = (f6_delta_val - f6_val) / delta;
	J(row, 2) = pd;

	// beta[3]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3] + delta, beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f6_delta_val = f6(vertex_index_in_p2e_vertices);
	pd = (f6_delta_val - f6_val) / delta;
	J(row, 3) = pd;

	// beta[4]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4] + delta, beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f6_delta_val = f6(vertex_index_in_p2e_vertices);
	pd = (f6_delta_val - f6_val) / delta;
	J(row, 4) = pd;

	// beta[5]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5] + delta;

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f6_delta_val = f6(vertex_index_in_p2e_vertices);
	pd = (f6_delta_val - f6_val) / delta;
	J(row, 5) = pd;

	// beta[beta_index]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index] + delta;
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f6_delta_val = f6(vertex_index_in_p2e_vertices);
	pd = (f6_delta_val - f6_val) / delta;
	J(row, beta_index) = pd;

	// beta[beta_index+1]///////////////////////////////////////////////////////////////////////
	c << beta[0], beta[1], beta[2];
	c_ << beta[3], beta[4], beta[5];

	setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

	uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
	uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

	px = m_osnap_poly.coors[v_index].px + beta[beta_index];
	py = m_osnap_poly.coors[v_index].py + beta[beta_index+1] + delta;

	m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

	f6_delta_val = f6(vertex_index_in_p2e_vertices);
	pd = (f6_delta_val - f6_val) / delta;
	J(row, beta_index+1) = pd;
}

// �����ƥ���ϵ�Ķ���
void OSnap::pd_f7(int v_index, Vertex *pv, Eigen::MatrixXd &J, int row, int beta_index, double delta)
{
    // ���ȼ���û�и���ʱ��f7ֵ
    // ���������˶�����ر���
    Eigen::Vector3d c;
    Eigen::Vector3d c_;
    Eigen::Matrix3d R;
    Eigen::Vector3d pointOfA;
    Eigen::Vector3d a;
    Eigen::Vector3d a_;
    double pitch;
    double alpha;

    // �ƶ�����ϵ�Ļ�
    // ԭ�㣺b0
    // ��������������b1-b0, b2-b0
    Eigen::Vector3d b0, b1, b2;

    // ����beta���ý��ٶ������ٶ�
    c << beta[0], beta[1], beta[2];
    c_ << beta[3], beta[4], beta[5];

    // ����c��c_���������˶�����
    setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

    // ������ϵ�Ļ�ִ�����������˶�
    uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

    // ����beta[beta_index],beta[beta_index+1]���ø��º�Ķ�ά����
    double px = m_osnap_poly.coors[v_index].px + beta[beta_index];
    double py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

    // ����m_osnap_poly.vertices[v_index]
    // b0, b1, b2, px, py ���ѱ�����
    m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

    // ����f7(beta)
    double f7_val = f7(v_index, pv);	// û�иı��κ�beta������f7ֵ
    double f7_delta_val;								// �ı�һ��beta������f7ֵ
    double pd;											// ƫ����

    // beta[0]///////////////////////////////////////////////////////////////////////
    c << beta[0] + delta, beta[1], beta[2];
    c_ << beta[3], beta[4], beta[5];

    setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

    uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

    px = m_osnap_poly.coors[v_index].px + beta[beta_index];
    py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

    m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

    f7_delta_val = f7(v_index, pv);
    pd = (f7_delta_val - f7_val) / delta;
    J(row, 0) = pd;

    // beta[1]///////////////////////////////////////////////////////////////////////
    c << beta[0], beta[1] + delta, beta[2];
    c_ << beta[3], beta[4], beta[5];

    setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

    uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

    px = m_osnap_poly.coors[v_index].px + beta[beta_index];
    py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

    m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

    f7_delta_val = f7(v_index, pv);
    pd = (f7_delta_val - f7_val) / delta;
    J(row, 1) = pd;

    // beta[2]///////////////////////////////////////////////////////////////////////
    c << beta[0], beta[1], beta[2] + delta;
    c_ << beta[3], beta[4], beta[5];

    setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

    uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

    px = m_osnap_poly.coors[v_index].px + beta[beta_index];
    py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

    m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

    f7_delta_val = f7(v_index, pv);
    pd = (f7_delta_val - f7_val) / delta;
    J(row, 2) = pd;

    // beta[3]///////////////////////////////////////////////////////////////////////
    c << beta[0], beta[1], beta[2];
    c_ << beta[3] + delta, beta[4], beta[5];

    setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

    uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

    px = m_osnap_poly.coors[v_index].px + beta[beta_index];
    py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

    m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

    f7_delta_val = f7(v_index, pv);
    pd = (f7_delta_val - f7_val) / delta;
    J(row, 3) = pd;

    // beta[4]///////////////////////////////////////////////////////////////////////
    c << beta[0], beta[1], beta[2];
    c_ << beta[3], beta[4] + delta, beta[5];

    setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

    uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

    px = m_osnap_poly.coors[v_index].px + beta[beta_index];
    py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

    m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

    f7_delta_val = f7(v_index, pv);
    pd = (f7_delta_val - f7_val) / delta;
    J(row, 4) = pd;

    // beta[5]///////////////////////////////////////////////////////////////////////
    c << beta[0], beta[1], beta[2];
    c_ << beta[3], beta[4], beta[5] + delta;

    setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

    uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

    px = m_osnap_poly.coors[v_index].px + beta[beta_index];
    py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

    m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

    f7_delta_val = f7(v_index, pv);
    pd = (f7_delta_val - f7_val) / delta;
    J(row, 5) = pd;

    // beta[beta_index]///////////////////////////////////////////////////////////////////////
    c << beta[0], beta[1], beta[2];
    c_ << beta[3], beta[4], beta[5];

    setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

    uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

    px = m_osnap_poly.coors[v_index].px + beta[beta_index] + delta;
    py = m_osnap_poly.coors[v_index].py + beta[beta_index+1];

    m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

    f7_delta_val = f7(v_index, pv);
    pd = (f7_delta_val - f7_val) / delta;
    J(row, beta_index) = pd;

    // beta[beta_index+1]///////////////////////////////////////////////////////////////////////
    c << beta[0], beta[1], beta[2];
    c_ << beta[3], beta[4], beta[5];

    setHelicalVars(c, c_, R, pointOfA, a, a_, pitch, alpha);

    uniformHelicalMove(c, c_, m_osnap_poly.b0, b0, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b1, b1, R, pointOfA, a, pitch, alpha);
    uniformHelicalMove(c, c_, m_osnap_poly.b2, b2, R, pointOfA, a, pitch, alpha);

    px = m_osnap_poly.coors[v_index].px + beta[beta_index];
    py = m_osnap_poly.coors[v_index].py + beta[beta_index+1] + delta;

    m_osnap_poly.vertices[v_index] = b0 + px * (b1 - b0) + py * (b2 - b0);

    f7_delta_val = f7(v_index, pv);
    pd = (f7_delta_val - f7_val) / delta;
    J(row, beta_index+1) = pd;
}









