#include "SimplifyVerticesSize.h"


SimplifyVerticesSize::SimplifyVerticesSize(void)
{
}


SimplifyVerticesSize::~SimplifyVerticesSize(void)
{
}

// �������ζ��㷨����(Ĭ�϶��������Ѿ��������ַ���)
// ������ǰ�󶥵��Լ���ǰ����ȷ����ǰ����ķ�����
void SimplifyVerticesSize::computeVerticesNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &contour, 
	                        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &vertices_normals,
							Eigen::Vector3f &plane_normal)
{
	if(vertices_normals.size() > 0) vertices_normals.clear();
    vertices_normals.reserve(contour->size());
	int prev, next;
	Eigen::Vector3f v_prev_p, v_p_next, v1, v2, v;
	float ratio1, ratio2, sum_dist, dist1, dist2;
	for(int i = 0; i < contour->size(); ++i)
	{
		prev = i == 0 ? contour->size()-1 : i-1;
		next = i == contour->size()-1 ? 0 : i+1;
		v_prev_p << contour->points[i].x - contour->points[prev].x,
					contour->points[i].y - contour->points[prev].y,
					contour->points[i].z - contour->points[prev].z;
		v_p_next << contour->points[next].x - contour->points[i].x,
					contour->points[next].y - contour->points[i].y,
					contour->points[next].z - contour->points[i].z;
		v_prev_p.normalize(); v_p_next.normalize();
		v1 = v_prev_p.cross(plane_normal);
		v2 = v_p_next.cross(plane_normal);
		v1.normalize(); v2.normalize();
		dist1 = distP2P(contour->points[i], contour->points[prev]);
		dist2 = distP2P(contour->points[i], contour->points[next]);
		sum_dist = dist1 + dist2;
		ratio1 = 1-dist1/sum_dist;
		ratio2 = 1-dist2/sum_dist;
		v = ratio1*v1+ratio2*v2;
        v.normalize();
		vertices_normals.push_back(v);
	}
}
// �������������ʼ���߶μ���
// ���ݶ���ķ�������ʹ��������������ȡֱ��
// �����ӵ��ѡȡԭ�����ӵ������ڵ�ķ�����������нϺõ�һ����(�������������)
// ��ʱ���µı��������ֱ�ߵĲ��������ֱ�ߵķ�����
// ���������㵽���ֱ�ߵľ��룬��ķ����������ֱ�ߵķ������ļн�
void SimplifyVerticesSize::constructInitLineSegs( pcl::PointCloud<pcl::PointXYZ>::Ptr &contour,// ����ε�������
	                        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &vertices_normals,// ���㷨����
							Eigen::Vector3f &poly_normal,	// ����η�����
							std::vector<std::vector<int>> &line_segs )	// ��Ϊ������У���1���߶ε�������յ㣻��2��������
{
	if(line_segs.size() > 0) line_segs.clear();

	std::vector<bool> is_processed(contour->size(), false);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr seg_inliers (new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setModelType (pcl::SACMODEL_LINE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (FLT_MAX);

	pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	int seed_index = -1;
	while(true)
	{
		line_cloud->clear();
		getSeedIndex(contour, seed_index, is_processed, vertices_normals);
		if(seed_index == -1) break;

		int prev = seed_index == 0 ? contour->size()-1 : seed_index-1;
		int next = seed_index == contour->size()-1 ? 0 : seed_index+1;
		is_processed[prev] = is_processed[next] = is_processed[seed_index] = true;
		std::vector<int> indices;
		indices.push_back(prev); indices.push_back(seed_index); indices.push_back(next);

		// �������ֱ�ߵĲ���
		for(int i = 0; i < indices.size(); ++i) line_cloud->push_back(contour->points[indices[i]]);
		seg.setInputCloud(line_cloud);
		seg.segment(*seg_inliers, *coefficients);
		Eigen::Vector3f p_base, line_dir, line_normal, /*poly_normal,*/ p_normal;
		p_base << coefficients->values[0], coefficients->values[1], coefficients->values[2];
		line_dir << coefficients->values[3], coefficients->values[4], coefficients->values[5];
		/*poly_normal << poly_set[poly_index].coeff[0], poly_set[poly_index].coeff[1], poly_set[poly_index].coeff[2];*/
		line_normal/*���ֱ�ߵķ���������ָ�������ⲿ*/ = line_dir.cross(poly_normal); line_normal.normalize();
		p_normal = vertices_normals[seed_index];
		if(p_normal.dot(line_normal) < 0) line_normal *= -1.0f;

		// �����ӵ��ǰ��ֱ�ʼ����
		bool is_prev_stopped = false;
		bool is_next_stopped = false;

		while(true)
		{
			// update prev & next
			if(!is_prev_stopped) prev = prev == 0 ? contour->size()-1 : prev-1;
			if(!is_next_stopped) next = next == contour->size()-1 ? 0 : next+1;
			// �����Ѿ���������ĵ���ֹͣ����
			if(is_processed[prev]) is_prev_stopped = true;
			if(is_processed[next]) is_next_stopped = true;

			if(!is_prev_stopped)
			{
				float dist = distP2Line(contour->points[prev], p_base, line_dir);
				p_normal = vertices_normals[prev];/*<< normal_cloud->points[prev].normal_x, normal_cloud->points[prev].normal_y, normal_cloud->points[prev].normal_z;*/
                float angle = acos(line_normal.dot(p_normal));
				if(angle <= T_angle_betPointAndLine && dist <= T_dist_betPointAndLine)
				{
					is_processed[prev] = true;
					indices.push_back(prev);
					line_cloud->push_back(contour->points[prev]);

					// update line params
					seg.setInputCloud(line_cloud);
					seg.segment(*seg_inliers, *coefficients);
					p_base << coefficients->values[0], coefficients->values[1], coefficients->values[2];
					line_dir << coefficients->values[3], coefficients->values[4], coefficients->values[5];
					line_normal = line_dir.cross(poly_normal); line_normal.normalize();
					p_normal = vertices_normals[prev];
					if(p_normal.dot(line_normal) < 0) line_normal *= -1.0f;
				}else{ // ���������������ĵ㣬��ֹͣ����
					is_prev_stopped = true;
				}
			}

			if(!is_next_stopped)
			{
				float dist = distP2Line(contour->points[next], p_base, line_dir);
				p_normal = vertices_normals[next]; /*<< normal_cloud->points[next].normal_x, normal_cloud->points[next].normal_y, normal_cloud->points[next].normal_z;*/
				float angle = acos(line_normal.dot(p_normal));
				if(angle <= T_angle_betPointAndLine && dist <= T_dist_betPointAndLine)
				{
					is_processed[next] = true;
					indices.push_back(next);
					line_cloud->push_back(contour->points[next]);

					// update line params
					seg.setInputCloud(line_cloud);
					seg.segment(*seg_inliers, *coefficients);
					p_base << coefficients->values[0], coefficients->values[1], coefficients->values[2];
					line_dir << coefficients->values[3], coefficients->values[4], coefficients->values[5];
					line_normal = line_dir.cross(poly_normal); line_normal.normalize();
					p_normal = vertices_normals[next];
					if(p_normal.dot(line_normal) < 0) line_normal *= -1.0f;
				}else{ // ���������������ĵ㣬��ֹͣ����
					is_next_stopped = true;
				}
			}

			if(is_prev_stopped && is_next_stopped)
			{	
				line_segs.push_back(indices);
				break;
			}
		}
	}
}

void SimplifyVerticesSize::getSeedIndex( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
				   int &seed_index,
				   std::vector<bool> &is_processed,
				   std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &vertices_normals )
{
	seed_index = -1;
	Eigen::Vector3f v_prev, v_i, v_next;
	float angle1, angle2;
	for(int i = 0; i < cloud->size(); ++i)
	{
		if(is_processed[i]) continue;	// ����õ��ѱ��������������Ϊ���ӵ�

		int prev = i == 0 ? cloud->size()-1 : i-1;
		int next = i == cloud->size()-1 ? 0 : i+1;

		if(is_processed[prev] || is_processed[next]) continue;	// ����õ�֮ǰ��֮��ĵ㱻���������õ㲻����Ϊ���ӵ�
		
		v_prev = vertices_normals[prev];
		v_i = vertices_normals[i];
		v_next = vertices_normals[next];

		angle1 = acos(v_i.dot(v_prev));
		angle2 = acos(v_i.dot(v_next));

		if(angle1 <= T_angle_betPointAndPoint && angle2 <= T_angle_betPointAndPoint)	// ���ӷ������ķ����Ͻ����ж�
		{
			seed_index = i;
			break;
		}
	}
}

float SimplifyVerticesSize::distP2Line( pcl::PointXYZ &point,
				  Eigen::Vector3f &p_base,		// ֱ��ģ�͵Ļ���
				  Eigen::Vector3f &line_dir )	// ֱ��ģ�͵ķ�������
{
	Eigen::Vector3f p, delta;
	p << point.x, point.y, point.z;
	delta = p_base - p;

	float lambda = -1.0 * line_dir.dot(delta);

	Eigen::Vector3f p_proj;
	p_proj = p_base + lambda * line_dir;

	return (p_proj-p).norm();
}

float SimplifyVerticesSize::distP2P(pcl::PointXYZ &p1, pcl::PointXYZ &p2)
{
	return pow((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z), 0.5f);
}

// �����Ż�����εĶ�������
void SimplifyVerticesSize::refineLineSegs( pcl::PointCloud<pcl::PointXYZ>::Ptr &contour,			// ԭʼ����
					 pcl::PointCloud<pcl::PointXYZ>::Ptr &final_vertices,	// ���յĶ��㼯�ϣ�x,y,z)
					 std::vector<std::vector<int>> &retrieved_line_segs, // ������������ȡ���߶�����
					 std::vector<int> &final_line_segs )	 // ���յ��߶�����(index)
{
	// final_line_segs�������յ��߶����У��������߶ε������contour�е�����)
	// �߶�L=(pi,pj)�ı�ʾ��ֻ�����߶ε����pi(��final_line_segs��)
	if(final_line_segs.size()>0) final_line_segs.clear();
	if(final_vertices==NULL){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		final_vertices = cloud;
	} else { final_vertices->clear(); }

	// ���������retrieved_line_segs.size()==0
	if(retrieved_line_segs.size()==0)
	{
		*final_vertices = *contour;
		return;
	}

	// step1:
	// ���߶ζ����������(�����������������ӵ�����ͬʱ�����ģ����retrieved_line_segs��ĳ���߶��ڶ�������в���һ���������ģ�
	// һ�����������һ���򵥵����򼴿ɣ����治�ܰ�����������е������յ㣩
	// ��������Ƕ���ε������յ��������ͬһ���߶���
	for(int i = 0; i < retrieved_line_segs.size(); ++i)
	{
		std::sort(retrieved_line_segs[i].begin(), retrieved_line_segs[i].end());
		/* ����0��һ���ᱻ������ȥ��
		 * ����������ʱ��ÿ�ν����������Ԫ���ڵĶ��㣬������ɵ��߶������ڲ�һ���������ġ�
		 * �������0û�б����������������߶��ڣ���ÿ���߶��ڲ��ĵ��������Ȼ�������
		 * �������0�����������������߶��ڣ���0֮ǰ�ĵ�Ҳ��һЩ��������������������к�0��Ȼ�����ڵ�һ�������ϡ�
		 *	   �������Ķ�������֮����һ���ϲ㣬���Ǽ�һ�����Ǽ�һ����1��ܶ�������������һ���߶�Ӧ���ƶ���ǰ������
		*/
        int i_size = retrieved_line_segs[i].size();
        if(retrieved_line_segs[i][0] == 0 && retrieved_line_segs[i][i_size-1] != i_size-1)
		{
			for(int j = 0; j < retrieved_line_segs[i].size()-1; ++j)
			{
				int next = j+1;
				if(next != retrieved_line_segs[i][j+1])
				{
					std::vector<int> tmp;
					for(int k = next; k < retrieved_line_segs[i].size(); ++k)
						tmp.push_back(retrieved_line_segs[i][k]);
					for(int k = 0; k < next; ++k)
						tmp.push_back(retrieved_line_segs[i][k]);
					for(int k = 0; k < tmp.size(); ++k)
						retrieved_line_segs[i][k] = tmp[k];
					break;
				}
			}
		}
	}

	// step2:
	// ��ȡ�߶ζ����Լ������߶�֮��Ĺ�����,���չ��ɳ�ʼ���߶����У�������final_line_segs����
	for(int i = 0; i < retrieved_line_segs.size(); ++i)
	{	// i: ��ǰ�߶���retrieved_line_segs�е�����
		int begin = retrieved_line_segs[i][0];	// ��һ���߶ε����
		int end = retrieved_line_segs[i][retrieved_line_segs[i].size()-1];	// ��һ���߶ε��յ�
		// ��һ���߶ε����
		int begin_next_line = i < retrieved_line_segs.size()-1 ? retrieved_line_segs[i+1][0] : retrieved_line_segs[0][0];
		final_line_segs.push_back(begin);
		final_line_segs.push_back(end);
		
		// end->end's next(end��ʾ��i���߶�֮������������)
		end = end == contour->size()-1 ? 0 : end+1;
		while(end != begin_next_line)
		{
			final_line_segs.push_back(end);
			end = end == contour->size()-1 ? 0 : end+1;
		}
	}

	assert(final_line_segs.size() >= 3);
	if(final_line_segs.size() < 3){
		*final_vertices = *contour;
		return;
	}

	// step3:
	// ���ĳ���߶εĳ���<<�߶ε�ƽ�����ȣ���ɽ������߶�ɾ��
	// ÿ�ν�ȥ��һ����С�߶Σ������������ȥ��������ʱ��ֹͣ
	std::vector<float> line_lens;
	line_lens.reserve(final_line_segs.size());
	float total_len = 0;
	for(int begin = 0; begin < final_line_segs.size(); ++begin)
	{
		int end = begin == final_line_segs.size()-1 ? 0 : begin+1;

		int begin_index/*w.r.t. contour*/ = final_line_segs[begin];
		int end_index/*w.r.t. contour*/ = final_line_segs[end];
		float dist = distP2P(contour->points[begin_index], contour->points[end_index]);
		line_lens.push_back(dist);
		total_len += dist;
	}
	float mean_len = total_len / float(final_line_segs.size());

	std::vector<bool> is_del(final_line_segs.size(), false);

	std::vector<int> indices;
	vectorSort(line_lens, indices);

	int residual_points = final_line_segs.size();
	int cur_index = indices[0];

    // float T_lenRatio = 0.25;
	int count_del = 0;
	std::vector<int> offset(indices.size(), 0);
	for(int i = 0; i < indices.size(); ++i)
	{
		cur_index = indices[i];
		if(residual_points <= 3) break;
		if(line_lens[cur_index]/mean_len < T_lenRatio){
			is_del[cur_index] = true;
			--residual_points;
			++count_del;
			offset[i]=-1;	// -1������������Ӧ���߶λᱻɾ��
		}else{
			break;
		}
	}

	// ����offset
	int off_set = 0;
	for(int i = 0; i < offset.size(); ++i)
	{
		if(!is_del[i]){
			offset[i] = off_set;
		}else{
			++off_set;
		}
	}

	for(int i = 0; i < indices.size(); ++i)
	{
		if(offset[i] > 0 && !is_del[i]){
			final_line_segs[i - offset[i]] = final_line_segs[i];
		}
	}
	for(int i = 0; i < count_del; ++i)
		final_line_segs.pop_back();
	

	// method 2
	//// �����߶εĳ���
	//std::vector<float> line_lens, line_lens_for_sort;
	//float total_len = 0;
	//for(int begin = 0; begin < final_line_segs.size(); ++begin)
	//{
	//	int end = begin == final_line_segs.size()-1 ? 0 : begin+1;

	//	int begin_index/*w.r.t. contour*/ = final_line_segs[begin];
	//	int end_index/*w.r.t. contour*/ = final_line_segs[end];
	//	float dist = distP2P(contour->points[begin_index], contour->points[end_index]);
	//	line_lens.push_back(dist);
	//	line_lens_for_sort.push_back(dist);
	//	total_len += dist;
	//}
	//float mean_len = total_len / float(final_line_segs.size());
	//std::sort(line_lens_for_sort.begin(), line_lens_for_sort.end());
	//float median_len = line_lens[line_lens.size()/2];

	//// ȷ����С�߶εĳ���������߶ξ�ֵ����󳤶ȱ���
	//float T_lenRatio = 0.25;
	//std::vector<bool> is_del(final_line_segs.size(), false);// ������Ƕ�Ӧ���߶��Ƿ���Ҫ��ɾ��
	//std::vector<int> off_set(final_line_segs.size(), 0);	// �������ÿ��������ǰƽ�Ƶ�λ������
	//int count_del = 0;	// ���������Ҫɾ���������߶�
	//for(int i = 0; i < final_line_segs.size(); ++i){
	//	if( line_lens[i]/mean_len < T_lenRatio ||
	//		line_lens[i]/median_len < T_lenRatio ){
	//		is_del[i] = true;
	//		++count_del;
	//		off_set[i] = -1;
	//		continue;
	//	}
	//	off_set[i] = count_del;
	//}

	//// ִ��ɾ��
	//for(int i = 0; i < final_line_segs.size(); ++i)
	//{
	//	if(off_set[i] > 0)
	//	{
	//		final_line_segs[i - off_set[i]] = final_line_segs[i];	// �ƶ�����ǰ���е�
	//	}
	//}
	//for(int i = 0; i < count_del; ++i)
	//	final_line_segs.pop_back();

	//// ��Ҫ���������������������εĶ�����������3������������������ʾ��
	//if(final_line_segs.size() < 3) {
	//	std::cerr << "������������3����" << std::endl;
	//	// ���Զ�����л���
	//	*final_vertices = *contour;
	//	return;
	//}

	// method1:
	//// step3:
	//// �������ȳ�����ֵ���߶�
	//// step3(a):
	//// �����߶εĳ���
	//std::vector<float> line_lens;
	//for(int begin = 0; begin < final_line_segs.size(); ++begin)
	//{
	//	int end = begin == final_line_segs.size()-1 ? 0 : begin+1;

	//	int begin_index/*w.r.t. contour*/ = final_line_segs[begin];
	//	int end_index/*w.r.t. contour*/ = final_line_segs[end];
	//	float dist = distP2P(contour->points[begin_index], contour->points[end_index]);
	//	line_lens.push_back(dist);
	//}

	//// step3(b):
	//// �������������߶εĳ��Ⱦ�С����ֵ�����⼸���߶εĳ���֮���ִ�����ֵ����ϲ��⼸���߶�
	//std::vector<int> line_segs(final_line_segs.size(), 0); // ���浱ǰ���߶�
	//for(int i = 0; i < final_line_segs.size(); ++i) line_segs[i] = final_line_segs[i];
	//
	//std::vector<bool> is_len_valid(line_segs.size(), false);
	//for(int i = 0; i < line_segs.size(); ++i){
	//	if(line_lens[i] >= T_dist_refine_lines) is_len_valid[i] = true;
	//}

	//// �߶�Li,Li+1,...,Li+N-1������N���߶Σ���len(Lj)<T_dist_refine_lines, j = i,i+1,...,i+N-1
	//// ��Len(Li)+Len(Li+1)+Len(Li+j)>T_dist_refine_lines
	//// ��Li...Lj�ɱ�����һ���߶�
	//std::vector<std::vector<int>> continuous_lines;
	//int first_valid_line_index = -1;
	//for(int i = 0; i < line_lens.size(); ++i)
	//{// ��ȡ��һ�����ȺϷ����߶ε�����
	//	if(is_len_valid[i])
	//	{
	//		first_valid_line_index = i;
	//		break;
	//	}
	//}
	//if(first_valid_line_index == line_lens.size()-1) {std::cerr << "error: the line segment length threshold is set to be too large!" << std::endl;}
	//int cur_index = first_valid_line_index + 1;
	//while(true)
	//{
	//	if(cur_index == first_valid_line_index){break;}
	//	if(is_len_valid[cur_index])	// ���߶γ��ȺϷ��������ȥ
	//	{
	//		cur_index = cur_index == line_lens.size()-1 ? 0 : cur_index+1;
	//		continue;
	//	}
	//	std::vector<int> v;	// ���߶γ��Ȳ��Ϸ��򱣴���λ��
	//	while(!is_len_valid[cur_index])
	//	{
	//		v.push_back(cur_index);
	//		cur_index = cur_index == line_lens.size()-1 ? 0 : cur_index+1;
	//	}
	//	continuous_lines.push_back(v);
	//}

	//// ���ڳ���С����ֵ�������߶Σ�����䳤��֮���Ƿ������ֵ�����������ֵ��ϲ���Щ�߶Σ�������Щ�߶�ɾȥ
	//std::vector<int> condition(line_segs.size(),0);	// 0: ������ 1��ɾ���� 2���ϲ�����������Щ�����ı����Ϊ2�ĵ㣩

	//for(int i = 0; i < continuous_lines.size(); ++i)
	//{
	//	if(continuous_lines[i].size() == 1) {condition[continuous_lines[i][0]] = 1; continue;}
	//	float sum_len = 0;
	//	for(int j = 0; j < continuous_lines[i].size(); ++j)
	//	{
	//		sum_len += line_lens[continuous_lines[i][j]];
	//	}
	//	if(sum_len <= T_dist_refine_lines)
	//	{
	//		for(int j = 0; j < continuous_lines[i].size(); ++j)
	//			condition[continuous_lines[i][j]] = 1;
	//	}else{
	//		/*for(int j = 0; j < continuous_lines[i].size(); ++j)
	//			condition[continuous_lines[i][j]] = 2;*/
	//		for(int j = 1; j < continuous_lines[i].size(); ++j)
	//			condition[continuous_lines[i][j]] = 1;
	//	}
	//}
	//// ���������߶�����
	//final_line_segs.clear();
	//for(int i = 0; i < line_segs.size(); ++i){
	//	if(condition[i] == 0){
	//		final_line_segs.push_back(line_segs[i]);
	//		final_vertices->push_back(contour->points[line_segs[i]]);	// ������һ���ô����߶β�����ܳ����������������ཻ�Ŀ����ԣ��ҽ�ʡ�˼�����
	//	}
	//}

	//// ��Ҫ���������������������εĶ�����������3������������������ʾ��
	//if(final_vertices->size() < 3) std::cerr << "������������3����" << std::endl;

	// step4:
	// ��������߶ν���ƽ�У���ϲ��������߶�
	std::vector<int> line_segs;
	line_segs.clear();
	for(int i = 0; i < final_line_segs.size(); ++i) line_segs.push_back(final_line_segs[i]);

	residual_points = line_segs.size();// ���浱ǰ����ζ��������
	final_line_segs.clear();
	std::vector<int> condition;
	condition.clear();
	for(int i = 0; i < line_segs.size(); ++i) condition.push_back(0);	// ע��0��Ĭ��ֵ��Ĭ�ϱ���

	for(int i = 0; i < line_segs.size(); ++i)
	{
		Eigen::Vector3f line_i_begin_point, line_i_end_point, prev_line_end_point;
		line_i_begin_point << contour->points[line_segs[i]].x, contour->points[line_segs[i]].y, contour->points[line_segs[i]].z;
		int prev = i == 0 ? line_segs.size()-1 : i-1;
		int next = i == line_segs.size()-1 ? 0 : i+1;
		line_i_end_point << contour->points[line_segs[next]].x, contour->points[line_segs[next]].y, contour->points[line_segs[next]].z;
		prev_line_end_point << contour->points[line_segs[prev]].x, contour->points[line_segs[prev]].y, contour->points[line_segs[prev]].z;

		Eigen::Vector3f line1_dir, line2_dir;
		line1_dir = line_i_begin_point - prev_line_end_point;
		line2_dir = line_i_end_point - line_i_begin_point;
		line1_dir.normalize();
		line2_dir.normalize();
		float cos_val = line1_dir.dot(line2_dir);
		if(cos_val >= cos(T_angle_betLines) && residual_points > 3)// Ҫ��֤���㹻�ĵ�ɱ�ɾ��
		{	
			condition[i] = 1;
			--residual_points;
		}
	}
	final_vertices->clear();final_line_segs.clear();
	for(int i = 0; i < line_segs.size(); ++i){
		if(condition[i]==0){
			final_line_segs.push_back(line_segs[i]);
			final_vertices->push_back(contour->points[line_segs[i]]);
		}
	}
}

bool compair(std::pair<float, int> &pa1, std::pair<float, int> &pa2)
{
	return pa1.first < pa2.first;
}
// ����v�����ֵ���䣬indices������С�����ֵ��Ӧ������
void SimplifyVerticesSize::vectorSort(std::vector<float> &v, std::vector<int> &indices)
{
	std::vector<std::pair<float, int>> pv;
	pv.reserve(v.size());
	std::pair<float, int> pa;
	for(int i = 0; i < v.size(); ++i)
	{
		pa.first = v[i];
		pa.second = i;
		pv.push_back(pa);
	}
	std::sort(pv.begin(), pv.end(), compair);
	indices.clear();
	indices.reserve(v.size());
	for(int i = 0; i < v.size(); ++i)
	{
		indices.push_back(pv[i].second);
	}
}

//void SimplifyVerticesSize::simplifyPolyVertices(MyPoly &poly)
//{
//	// step1: ȥ��ͬһ��λ���ϵ������
//	cleanPolyVer(poly.vertices_cloud);
//	// step2: ������������ʹ��������ַ���
//	validateVerticesSeq(poly.vertices_cloud, poly.poly_normal);
//	// step3�����㶥��ķ�����
//	computeVerticesNormals(poly.vertices_cloud, poly.vertices_normals, poly.poly_normal);
//	// step4: �����ʼ���߶�����
//	constructInitLineSegs(poly.vertices_cloud, poly.vertices_normals, poly.poly_normal, poly.line_segs);
//	// step5: �����߶�����
//	std::vector<int> final_line_segs;
//	refineLineSegs(poly.vertices_cloud, poly.final_vertices/*���ǲ�����*/, poly.line_segs, final_line_segs);
//}
