#ifndef OSNAP_H
#define OSNAP_H

#include "snappingvertices.h"

struct Coor_sigma	// �ƶ�����ϵ�Ķ�ά����
{
    double px;
    double py;
};

struct OSnapPoly
{
    Eigen::Vector3d c;		// ���ٶ�����
    Eigen::Vector3d c_;		// ƽ���ٶ�����
    Eigen::Vector3d b0;		// �ƶ�����ϵ��ԭ��
    Eigen::Vector3d b1;		// �ƶ�����ϵ��e1�Ķ˵�
    Eigen::Vector3d b2;		// �ƶ�����ϵ��e2�Ķ˵�
    Eigen::Vector4d coeff;			// ��ǰƽ�淽�̵�ϵ��
    Eigen::Vector4d coeff_origin;	// ԭʼƽ�淽�̵�ϵ��
	vector<Eigen::Vector3d> vertices;			// ƽ�����εĵ�ǰ����
    vector<Eigen::Vector3d> vertices_origin;	// ƽ�����ε�ԭʼ����
    std::vector<Coor_sigma> coors;							// ��ǰƽ��������ÿ���������ƶ�����ϵ�Ķ�ά����
};

class OSnap
{
public:
    OSnap();

	vector<POLYGON, Eigen::aligned_allocator<POLYGON>>* m_polygons;	// ����Ķ���ζ��󼯺�
    POLYGON *m_poly;    // ����Ķ���ζ���
    OSnapPoly m_osnap_poly;     // Ŀ�����ζ���

    vector<Vertex*> normal_vertices;    // û��ƥ���ϵ�Ķ��㼯��
    vector<Vertex*> p2p_vertices;       // ���е�-��ƥ���ϵ�Ķ��㼯��
    vector<Vertex*> p2e_vertices;       // ���е�-��ƥ���ϵ�Ķ��㼯��

	// ��ԭʼ�����е�����
	vector<int> normal_vertices_indices;
	vector<int> p2p_vertices_indices;
	vector<int> p2e_vertices_indices;

    Eigen::MatrixXd J;					// Jacob����
    Eigen::VectorXd beta;				// ��������

    double delta;   // ������ֵ������ƫ���Ĳ���
    //double max_r_norm;  // �����ݶȽⷽ��,����r�������
	int max_k_count;	// �����ݶȽⷽ�̵�����������

    // �������ݽṹ
    void reset();

    // ��������Ķ���ζ���m_poly����Ŀ�����ζ���m_osnap_poly
    void setOSnapPoly();
    // ���ö��㹹���ʼ���ƶ�����ϵ
    void setBases_init(OSnapPoly &poly);
    // ���ö���ζ������ƶ�����ϵ�еĶ�ά����
    void set2DCoordinates(OSnapPoly &poly);

    // ����ƥ���ϵ���ඥ��
    void categoriseVertices();
	
    // ���������˶�
    // ���ñ���
    void setHelicalVars( Eigen::Vector3d &c,
                         Eigen::Vector3d &c_,
                         Eigen::Matrix3d &R,
                         Eigen::Vector3d &pointOfA,
                         Eigen::Vector3d &a,
                         Eigen::Vector3d &a_,
                         double &pitch,
                         double &alpha );
    // ִ���ƶ�
    void uniformHelicalMove( Eigen::Vector3d &c,
                             Eigen::Vector3d &c_,
                             Eigen::Vector3d &x,
                             Eigen::Vector3d &x_,
                             Eigen::Matrix3d &R,
                             Eigen::Vector3d &pointOfA,
                             Eigen::Vector3d &a,
                             double &pitch,
                             double &alpha );
    // ���¾��������˶�����ƶ�����ϵ�Ļ�
    void updateBasesAfterHelicalMove(OSnapPoly &poly);
    // ���¾��������˶��󶥵����ά����
    // (ǰ�᣺���ƶ�����ϵ�Ļ���+����ά���ꡯ�ѱ����¹���)
    void updateVerticesAfterHelicalMove(OSnapPoly &poly);

	// ���ݱ�������beta����ÿ���������ƶ�����ϵ�еĶ�ά����
	void updateCoors();

	// ������������εĽ���
	bool getIntersectLine(Eigen::Vector4f &coeff1, Eigen::Vector4f &coeff2,
                          Eigen::Vector3d &p_base, Eigen::Vector3d &line_dir);

    // Ŀ�꺯��
	// ���ֶ��㹲��
    // ���º󶥵��λ����ԭ��λ�õľ���
	double f1(int vertex_index);

	// ��-��
	// ���º󶥵�λ����Ŀ�궥��λ�õľ���
	double f2(int vertex_index_in_p2p_vertices);
	// dist[���º󶥵㣬Ŀ�궥���ǰ���ڵ�] - dist[Ŀ�궥�㣬Ŀ�궥���ǰ���ڵ�]
	double f3_1(int vertex_index_in_p2p_vertices);
	// dist[���º󶥵㣬Ŀ�궥��ĺ�̽ڵ�] - dist[Ŀ�궥�㣬Ŀ�궥��ĺ�̽ڵ�]
	double f3_2(int vertex_index_in_p2p_vertices);
	// f5�Ķ�ż�汾�����º󶥵�λ��������ƽ�潻�ߵľ���
	double f8(int vertex_index_in_p2p_vertices);

	// ��-��
	// ���º󶥵�λ�õ�ƥ��ߵľ���
	double f4(int vertex_index_in_p2e_vertices);
	// ���º󶥵�λ��������ƽ�潻�ߵľ���
	double f5(int vertex_index_in_p2e_vertices);
	// ���º󶥵㵽ƥ��������˵����֮�� - ƥ��߳���
	double f6(int vertex_index_in_p2e_vertices);

	// ��ƥ���ϵ�Ķ���
    // ���º󶥵㵽ƥ���ϵ����֧��ƽ��ľ���
    double f7(int vertex_index, Vertex *pv);

	// Ŀ�꺯����Ȩֵ
	double f1_weight;
	double f2_weight;
	double f3_1_weight;
	double f3_2_weight;
	double f8_weight;
	double f4_weight;
	double f5_weight;
	double f6_weight;
	double f7_weight;

	// Ŀ�꺯����ƫ����
	// ���û��ƥ���ϵ�Ķ��㣺normal_vertices
	void pd_f1_normal_vertices(int v_index, Eigen::MatrixXd &J, int row, double delta);

	// �����ƥ���ϵ�Ķ���: p2p_vertices �� p2e_vertices
	void pd_f1_matching_vertices(int v_index, Eigen::MatrixXd &J, int row, int beta_index, double delta);

	// ��Ե�Ե�
	// f2��ƫ����
	void pd_f2(int vertex_index_in_p2p_vertices, Eigen::MatrixXd &J, int row, int beta_index, double delta);
	// f3_1��ƫ����
	void pd_f3_1(int vertex_index_in_p2p_vertices, Eigen::MatrixXd &J, int row, int beta_index, double delta);
	// f3_2��ƫ����
	void pd_f3_2(int vertex_index_in_p2p_vertices, Eigen::MatrixXd &J, int row, int beta_index, double delta);
	// f8��ƫ����
	void pd_f8(int vertex_index_in_p2p_vertices, Eigen::MatrixXd &J, int row, int beta_index, double delta);

	// ��Ե�Ա�
	// f4��ƫ����
	void pd_f4(int vertex_index_in_p2e_vertices, Eigen::MatrixXd &J, int row, int beta_index, double delta);
	// f5��ƫ����
	void pd_f5(int vertex_index_in_p2e_vertices, Eigen::MatrixXd &J, int row, int beta_index, double delta);
	// f6��ƫ����
	void pd_f6(int vertex_index_in_p2e_vertices, Eigen::MatrixXd &J, int row, int beta_index, double delta);

    // �����ƥ���ϵ�Ķ���
    // f7��ƫ����
    void pd_f7(int v_index, Vertex *pv, Eigen::MatrixXd &J, int row, int beta_index, double delta);

	// �Ż�	
	// ��ʼ����������beta,��ÿ���Ż��Ŀ�ʼʹ��һ��
	void initBeta();
	// ����Jacob����
	void cosntructJacobMat();
	// ִ�е�����˹ţ�ٵ������Ӷ�����beta
	void fill_r(Eigen::VectorXd &r);
	void conjugateGradient(Eigen::MatrixXd &A, Eigen::VectorXd &x, Eigen::VectorXd &b);
	void conjugateGradient_with_max_k(Eigen::MatrixXd &A, Eigen::VectorXd &x, Eigen::VectorXd &b, int k_count);
	void performGN();
	// �����ƶ�����ϵ
	void updateMovingCoor();
	// ���¶���λ��
	void updateVertices();

	// ���յ��Ż�ִ�к���
	void optimise();
};

#endif // OSNAP_H
