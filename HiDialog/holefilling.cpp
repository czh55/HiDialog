#include "holefilling.h"

HoleFilling::HoleFilling()
{
	sv_polygons = NULL;
	m_vertices_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    m_cloud_for_indentify_poly_id.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

// �������ݽṹ
void HoleFilling::clear()
{
	sv_polygons = NULL;
	m_vertices_cloud->clear();
	for(int i = 0; i < m_vertices.size(); ++i){
		if(!m_vertices[i]){
			delete m_vertices[i];
		}
	}
	m_vertices.clear();
	for(int i = 0; i < m_polygons.size(); ++i){
		if(!m_polygons[i]){
			delete m_polygons[i];
		}
	}
	m_polygons.clear();
}

// step1: �Զ�������ݽ��и�ʽת��
void HoleFilling::setInputPolygons(vector<POLYGON, Eigen::aligned_allocator<POLYGON>> *p_polygons)
{
	if(!p_polygons){
		cerr << "in HoleFilling::setInputPolygons: p_polygons == NULL" << endl;
		return;
	}
	clear();	// �����������ݽṹ
	this->sv_polygons = p_polygons;

	// step1: ����û�����ඥ���ȫ�ֶ�����Ƽ��ռ����ݽṹ
	pcl::PointCloud<pcl::PointXYZ>::Ptr vertices (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	vector<int> indices;
	vector<float> dists;
	float epsilon = 0.0001;

	int vertices_size = 0;
	for(int i = 0; i < sv_polygons->size(); ++i){
		vertices_size += (*sv_polygons)[i].getSize();
	}
	vertices->reserve(vertices_size * 2);

	Vertex *pv;
	for(int i = 0; i < sv_polygons->size(); ++i){
		for(int j = 0; j < (*sv_polygons)[i].getSize(); ++j){
			pv = j == 0 ? (*sv_polygons)[i].start_point : pv->next_point;
			vertices->push_back(pv->point);
		}
	}

	kdtree.setInputCloud(vertices);
	vector<bool> is_processed(vertices_size, false);	// �����Ƿ��ѱ������
	vector<bool> is_del(vertices_size, false);			// �����Ƿ��豻ɾ��

	for(int i = 0; i < vertices->size(); ++i){
		if(is_processed[i]) continue;
		is_processed[i] = true;

		kdtree.radiusSearch(vertices->points[i], epsilon, indices, dists);
		if(indices.size() <= 1){
			continue;
		}else{
			for(int j = 0; j < indices.size(); ++j){	// indices�еĵ�һ��Ԫ��δ�ؾ���i
				if(indices[j] != i){
					is_processed[indices[j]] = true;
					is_del[indices[j]] = true;
				}
			}
		}
	}

	m_vertices_cloud->reserve(vertices_size * 2);
	for(int i = 0; i < vertices->size(); ++i){
		if(!is_del[i]) m_vertices_cloud->push_back(vertices->points[i]);
	}
	m_vertices_kdtree.setInputCloud(m_vertices_cloud);

	int duplicate_vertices_size = vertices->size() - m_vertices_cloud->size();
	cout << "�غϵ��������" << duplicate_vertices_size << endl;

	// step2: ���춥�㼯��
	m_vertices.reserve(vertices_size * 2);
	for(int i = 0; i < m_vertices_cloud->size(); ++i){
		Vertex2 *v = new Vertex2();
		v->pos << m_vertices_cloud->points[i].x,
			      m_vertices_cloud->points[i].y,
				  m_vertices_cloud->points[i].z;
		v->coeff << 0, 0, 0, 0;
		v->proj_pos << 0, 0, 0;
		v->state_angle = 0;
		m_vertices.push_back(v);
	}

	// step3: �������μ���
	for(int i = 0; i < sv_polygons->size(); ++i){
		POLYGON2 *poly = new POLYGON2();
		for(int j = 0; j < (*sv_polygons)[i].getSize(); ++j){
			pv = j == 0 ? (*sv_polygons)[i].start_point : pv->next_point;

			m_vertices_kdtree.nearestKSearch(pv->point, 1, indices, dists);
			if(dists[0] > epsilon){
				cerr << "in HoleFilling::setInputPolygons: dists[0] > epsilon" << endl;
				cerr << "dists[0] = " << dists[0] << "; epsilon = " << epsilon << endl;
				return;
			}
			int v_index = indices[0];

			if(j == 0){
				poly->insertFirstVertex2(m_vertices[v_index]);
			}else if(j == (*sv_polygons)[i].getSize()-1){
				poly->insertLastVertex2(m_vertices[v_index]);
			}else{
				poly->insertMiddleVertex2(m_vertices[v_index]);
			}
		}
        poly->coeff << (*sv_polygons)[i].coeff[0],
                       (*sv_polygons)[i].coeff[1],
                       (*sv_polygons)[i].coeff[2],
                       (*sv_polygons)[i].coeff[3];
		m_polygons.push_back(poly);
	}
}

// step2: ����������Ƭ����������m_cloud_for_indentify_poly_id
void HoleFilling::set_cloud_for_indentify_poly_id()
{
    m_cloud_for_indentify_poly_id->clear();
    int reserved_size = 0;
    for(int i = 0; i < m_polygons.size(); ++i){
        reserved_size += m_polygons[i]->getSize();
    }
    m_cloud_for_indentify_poly_id->reserve(reserved_size * 5);

    pcl::PointXYZ p;
    //int poly_index;
    //Vertex2*pv;
    POLYGON2 *pPoly;
    Eigen::Vector3d vec;
    for(int i = 0; i < this->m_polygons.size(); ++i){
        pPoly = m_polygons[i];
        for(int j = 0; j < pPoly->triangles.size(); ++j){
            vec << 0, 0, 0;
            vec += pPoly->triangles[j].v[0]->pos;
            vec += pPoly->triangles[j].v[1]->pos;
            vec += pPoly->triangles[j].v[2]->pos;
            vec = 1.0/3.0*vec;
            p.x = vec[0];
            p.y = vec[1];
            p.z = vec[2];
            p.data[3] = i;
            m_cloud_for_indentify_poly_id->push_back(p);
        }
    }

    m_cloud_for_indentify_poly_id_kdtree.setInputCloud(m_cloud_for_indentify_poly_id);
}

// ���¶�������(�������Ӧ�Ķ������ͬ�����£�
void HoleFilling::updateVertices()
{
	if(m_vertices.size() == 0){
		return;
	}

	vector<bool> is_del(m_vertices.size(), false);

	int del_count = 0;

	for(int i = 0; i < m_vertices.size(); ++i){
		if(m_vertices[i]->relevant_polys.size() == 0){
			is_del[i] = true;
			++del_count;
			delete m_vertices[i];
			m_vertices[i] = NULL;
		}
	}

	if(del_count == 0) return;	// û�ж�����Ҫ��ɾ��

	cout << "����ɾ��ǰ:" << endl;
	cout << "m_vertices.size = " << m_vertices.size() << endl;
	cout << "m_vertices_cloud.size = " << m_vertices_cloud->size() << endl;

	vector<int> off_set(m_vertices.size(), 0);
	del_count = 0;
	for(int i = 0; i < is_del.size(); ++i){
		if(is_del[i]){
			off_set[i] = -1;
			++del_count;
		}else{
			off_set[i] = del_count;
		}
	}

	for(int i = 0; i < m_vertices.size(); ++i){
		if(off_set[i] != -1){
			m_vertices[i - off_set[i]] = m_vertices[i];
			m_vertices_cloud->points[i - off_set[i]] = m_vertices_cloud->points[i];
		}
	}
	for(int i = 0; i < del_count; ++i){
		m_vertices.pop_back();
		m_vertices_cloud->points.pop_back();
	}
	cout << "����ɾ����:" << endl;
	cout << "del_count = " << del_count << endl;
	cout << "m_vertices.size = " << m_vertices.size() << endl;
	cout << "m_vertices_cloud.size = " << m_vertices_cloud->size() << endl;
}

// ��ȡһ�����㵥��ƥ���ϵ������
int HoleFilling::getSingleRelationCount(Vertex2* v)
{
    // ����ƥ���ϵ����v������ĳ�������P����ǰ��Ϊprev, ���Ϊnext
    // ��prev�����ڱ�Ķ����P1,prev��ǰ��Ϊv����v��prev֮��Ϊ˫�����������Ϊ�������
    // ��next�����ڱ�Ķ����P2,next�ĺ��Ϊv����v��next֮��Ϊ˫�����������Ϊ�������
    // û�е��������ϵ�Ķ��㣬�����û�������Ķ���Σ�����Ϊ�������㣻����Ϊģ���ڲ��Ķ���
    // ֻ���������������ϵ�Ķ�����ģ�͵ı߽���׶�����ı߽��
    // ���������ϵ��������ϵ�Ķ����Ǿ��нǶԽǹ�ϵ�Ķ���

    int single_relation_count = 0;
    Vertex2 *prev, *next;

    for(int i = 0; i < v->relevant_polys.size(); ++i){
        // �ȴ���ǰ���ڵ�
        prev = (Vertex2 *)v->relevant_polys[i].prev_point;
        bool is_v_prev_of_prev_in_another_poly = false;
        for(int j = 0; j < prev->relevant_polys.size(); ++j){
            if((Vertex2*)prev->relevant_polys[j].prev_point == v){
                is_v_prev_of_prev_in_another_poly = true;
                break;
            }
        }
        if(!is_v_prev_of_prev_in_another_poly){
            ++single_relation_count;
        }

        // �ڴ����̽ڵ�
        next = (Vertex2*)v->relevant_polys[i].next_point;
        bool is_v_next_of_next_in_another_poly = false;
        for(int j = 0; j < next->relevant_polys.size(); ++j){
            if((Vertex2*)next->relevant_polys[j].next_point == v){
                is_v_next_of_next_in_another_poly = true;
                break;
            }
        }
        if(!is_v_next_of_next_in_another_poly){
            ++single_relation_count;
        }
    }

    return single_relation_count;
}

// ���ڶ���ε����ǻ��ʷֽ������������Ƭ���ĵ���
void HoleFilling::updateTriCentroids()
{
	m_cloud_for_indentify_poly_id->clear();

	for(int i = 0; i < m_polygons.size(); ++i){
		POLYGON2* pPoly = m_polygons[i];
		Eigen::Vector3d centroid;
		pcl::PointXYZ p;
		for(int j = 0; j < pPoly->triangles.size(); ++j){
			centroid.setZero();
			centroid += pPoly->triangles[j].v[0]->pos;
			centroid += pPoly->triangles[j].v[1]->pos;
			centroid += pPoly->triangles[j].v[2]->pos;
			centroid /= 3;
			p.x = centroid[0];
			p.y = centroid[1];
			p.z = centroid[2];
			p.data[3] = i;
			m_cloud_for_indentify_poly_id->push_back(p);
		}
	}

	this->m_cloud_for_indentify_poly_id_kdtree.setInputCloud(this->m_cloud_for_indentify_poly_id);
}

// ���¶������
void HoleFilling::updateVerCloud()
{
	/*m_vertices_cloud->clear();	
	pcl::PointXYZ p;
	for(int i = 0; i < m_vertices.size(); ++i){
		p.x = m_vertices[i]->pos[0];
		p.y = m_vertices[i]->pos[1];
		p.z = m_vertices[i]->pos[2];
		m_vertices_cloud->push_back(p);
	}*/
	this->updateVertices();	// ��û�й�������εĶ���ɾ��
	m_vertices_kdtree.setInputCloud(m_vertices_cloud);
}