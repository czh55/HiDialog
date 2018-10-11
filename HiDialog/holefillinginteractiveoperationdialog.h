#ifndef HOLEFILLINGINTERACTIVEOPERATIONDIALOG_H
#define HOLEFILLINGINTERACTIVEOPERATIONDIALOG_H

#include <QDialog>
#include "holefilling.h"

struct Vertex_info
{
    int poly_id;    // ����α��
    Vertex2* v;     // ����λ��
};

namespace Ui {
class HoleFillingInteractiveOperationDialog;
}

class HoleFillingInteractiveOperationDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit HoleFillingInteractiveOperationDialog(QWidget *parent = 0);
    ~HoleFillingInteractiveOperationDialog();

    void * m_pclviewer;
    char * m_CMD;           // �����л�����

    int selected_poly_id;   // ѡ�еĶ����id
    Vertex2 * m_vertex1;    // ѡ�еĶ���1
    Vertex2 * m_vertex2;    // ѡ�еĶ���2

    vector<Vertex2*> line_segs;  // ����1�붥��2���ɵ��߶�����

    void updateInfo();

    ////////////////////////////////////////////////////////
    // �������ǻ��㼯
    vector<Vertex2*> m_points_set_for_tri;  // �������ģʽ(1,2,3)ѡ���һ���㼯�������˽������ǻ���Ҫ���ĵ����߶εķ�Χ
    // ѡ��׶��е�һ�����㣬�Ӷ�ʶ����׶��߽�
    Vertex_info m_vi_one_vertex;
    Eigen::Vector3d m_camera_pos;
    // ģʽ1��������һ���������ɿ׶��߽�㼯
    bool genPointsSetWithOneVertex();
	// Input: v: ��ǰ���㣻poly��v�����������
	// Output: next_v: ��һ�����㣻next_poly��next_v�����������
	// Ӧ�ø÷�����ǰ������Ҫ��֤Ŀ�����ε�v��ǰ���ڵ�����ڱ߽��ϣ�
    bool findNextPoint(Vertex2* v, POLYGON2* poly, Vertex2* &next_v, POLYGON2* &next_poly);
	// ����ƽ���ͶӰ
	void getProjPointOnPlane(Eigen::Vector3d &v_pos, Eigen::Vector4d &coeff, Eigen::Vector3d &proj_pos);
	// e2˳ʱ����ת��e1ת���ĽǶ�
	double getRotationAngle(Eigen::Vector3d &e1, Eigen::Vector3d &e2, Eigen::Vector3d &normal);
    // ģʽ2�� ����һ�������һ���յ����ɿ׶��߽�㼯
    Vertex_info m_v_begin_mode2;
    Vertex_info m_v_end_mode2;
    bool genPointsSetWithTwoVertices();
    // ģʽ3�������ĸ��������ɿ׶��߽�㼯�����������������������ε����
    Vertex_info m_v1_mode3;
    Vertex_info m_v2_mode3;
    Vertex_info m_v3_mode3;
    Vertex_info m_v4_mode3;
    bool genPointsSetWithFourVertices();

	// ���ǻ�
	// ��ȡ�㵽�߶εľ���
	double getDistP2LS(Vertex2* l_begin, Vertex2* l_end, Vertex2* v);
	// ��ȡָ���߶ε�ĳ���㼯��ÿ������ľ��룬�����մ�С�����˳���������
	void getDistInfo(Vertex2* l_begin, Vertex2 *l_end, vector<Vertex2*> &points, vector<int> &indices, vector<float> &dists);
	// �����κϷ��Լ��
	bool isValidTriangle(Vertex2* l_begin, Vertex2* l_end, Vertex2* v, 
	                     vector<Vertex2*> &m_points_set_for_tri, int poly_start_index,
	                     Eigen::Vector3d &normal);
	// �ж϶����Ƿ����߶���
	bool isVertexOnLingSeg(Vertex2* l_begin, Vertex2* l_end, Vertex2* v);
	// �ж϶����Ƿ����������ڲ�
	bool isVertexInTriangle(Vertex2* l_begin, Vertex2* l_end, Vertex2* v, Vertex2* test_point);
	// �����߶��Ƿ�ƽ��
	bool isTwoLineSegsParal(Vertex2* l1_begin, Vertex2* l1_end, Vertex2* l2_begin, Vertex2* l2_end);
    // �����߶��Ƿ��ཻ(��λ���������ཻ)
	bool isTwoLineSegsIntersect(Vertex2* l1_begin, Vertex2* l1_end, Vertex2* l2_begin, Vertex2* l2_end);
	// �����������㹹���µ������ζ����
	void genNewTriPly(Vertex2* v0, Vertex2* v1, Vertex2* v2, Eigen::Vector3d &normal);

    // ˮƽ���򣬴�ֱ��������ģ�ͱ߽�
    // ͶӰ���߶������Ƿ�Ϸ�
    // ��������߶�����: problem_i, problem_j
    bool isProjLineSegsValid(vector<Vertex2*> &line_segs, int &problem_i, int &problem_j);
    // �����߶εķ��������Ƿ�ƽ��(i,j��Ϊ�߶ε����)
    bool isBothLineSegsParal(vector<Vertex2*> &line_segs, int i, int j);
    // �����߶��Ƿ񲿷��ص�
    bool isBothLineSegsOverlap(vector<Vertex2*> &line_segs, int i, int j);
    // �߶�i���߶�i+1�Ƿ���кϷ���λ�ù�ϵ
    bool isAdjacentLineSegsHasValidPosRelation(vector<Vertex2*> &line_segs, int i);
	//���ñ�����ʱĬ��ģ���ڲ��Ѿ������ڿ׶���
	// input��v��ģ�ͱ߽��ϵĶ���
	// output: next_v, ģ�ͱ߽��ϵĶ���
	bool findNextPointVertexOnModelBorder(Vertex2* v, Vertex2* &next_v);
	// �ж϶���v�Ƿ���е�������ӹ�ϵ
	bool isVertexHasSingleConnRelation(Vertex2*v);
    // ִ�к���
    void performExtention(int extend_type);
    // ģ�ͷ�յ����ݽṹ
    vector<vector<Vertex2*>> m_four_vertices;
    Vertex2* m_bottom_left_vertex;
    Vertex2* m_bottom_right_vertex;
    double m_min_y;
    double m_min_x;
    double m_max_x;
    double m_min_z;
    int plane_type; // 0:���棻1���������;2:�Ҳ����棻3���������

	// ��ͼ���º���
	void updateViewer();// ȫ�ָ���

private slots:
    void on_pushButton_select_poly_clicked();

    void on_pushButton_simplify_cancel_poly_selection_clicked();

    void on_pushButton_select_p1_clicked();

    void on_pushButton_select_p2_clicked();

    void on_pushButton_gener_line_seg_clicked();

    void on_pushButton_del_lineseg_clicked();

    void on_pushButton_switch_ling_seg_clicked();

    void on_pushButton_simplify_line_segs_clicked();

    void on_pushButton_find_prev_point_clicked();

    void on_pushButton_find_next_point_clicked();

    void on_pushButton_search_angle2angle_vertices_clicked();

    void on_pushButton_select_one_vertex_clicked();

    void on_pushButton_get_camera_pos_clicked();

    void on_pushButton_gen_points_set_for_tri_clicked();

    void on_pushButton_test_findNextPoint_clicked();

    void on_pushButton_select_two_vertices_clicked();

    void on_pushButton_sel_v1_for_mode2_clicked();

    void on_pushButton_sel_v2_for_mode2_clicked();

    void on_pushButton_select_four_vertices_clicked();

    void on_pushButton_sel_v1_for_mode3_clicked();

    void on_pushButton_sel_v2_for_mode3_clicked();

    void on_pushButton_sel_v3_for_mode3_clicked();

    void on_pushButton_sel_v4_for_mode3_clicked();

    void on_pushButton_test_clicked();

    void on_pushButton_set_camera_pos_clicked();

    void on_pushButton_get_poly_num_clicked();

    void on_pushButton_get_ver_num_clicked();

    void on_pushButton_cancel_poly_wireframe_display_clicked();

    void on_pushButton_display_poly_wireframe_clicked();

    void on_pushButton_gen_proj_line_segs_clicked();

    void on_pushButton_extend_to_bottom_clicked();

    void on_pushButton_extend_to_left_clicked();

    void on_pushButton_extend_to_right_clicked();

    void on_pushButton_extend_to_backward_clicked();

    //void on_pushButton_close_the_model_clicked();

    //void on_pushButton_gen_the_two_border_vertices_clicked();

    //void on_pushButton_gen_bottom_plane_clicked();

    //void on_pushButton_gen_left_plane_clicked();

    //void on_pushButton_gen_right_plane_clicked();

    //void on_pushButton_gen_backward_plane_clicked();

    void on_pushButton_clicked();

    void on_pushButton_del_poly_clicked();

    void on_pushButton_CDT_Hole_Filling_clicked();

    void on_pushButton_dist_V2V_clicked();

    void on_pushButton_test_ply_op_clicked();

    void on_pushButton_insert_vertices_clicked();

    void on_pushButton_gen_two_key_vertices_clicked();

    void on_pushButton_gen_bottom_plane_clicked();

    void on_pushButton__insert_interior_vertices_clicked();

    void on_pushButton_display_ply_normal_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_3_clicked();

private:
    Ui::HoleFillingInteractiveOperationDialog *ui;    
};

#endif // HOLEFILLINGINTERACTIVEOPERATIONDIALOG_H
