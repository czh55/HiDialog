#include "setparams_mergingvertices_dialog.h"
#include "ui_setparams_mergingvertices_dialog.h"
#include <QMessageBox>
#include <QStringList>

SetParams_MergingVertices_Dialog::SetParams_MergingVertices_Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SetParams_MergingVertices_Dialog)
{
    ui->setupUi(this);
    model = new QStandardItemModel();
    initTable();
}

SetParams_MergingVertices_Dialog::~SetParams_MergingVertices_Dialog()
{
    delete ui;
    delete model;
    delete configIniRead;
}

void SetParams_MergingVertices_Dialog::initTable()
{
    // ���ñ��ÿ�еĿ��
    ui->tableView->setColumnWidth(0, 100);
    ui->tableView->setColumnWidth(1, 600);
    ui->tableView->setColumnWidth(2, 50);

    // ���ñ�ͷ
    model->setHorizontalHeaderItem(0, new QStandardItem(QObject::tr("variable name")));
    model->setHorizontalHeaderItem(1, new QStandardItem(QObject::tr("role")));
    model->setHorizontalHeaderItem(2, new QStandardItem(QObject::tr("value")));
    ui->tableView->setModel(model);

    configIniRead = new QSettings("config.ini", QSettings::IniFormat);

    // ���ñ���
    setTableItem(0, "ratio_of_scale", "�߶ȵı������ӣ�������������뾶");
    setTableItem(1, "T_ratio_lineSeg_middle_area", "�߶��в�����ռ���߶γ��ȵı���");
    setTableItem(2, "T_dist_proj_to_lineSegEnd", "ͶӰ�����߶ζ˵��������");
    setTableItem(3, "T_maxAngle_bet_two_polys", "����ƽ�ж���εķ��������н�");
    setTableItem(4, "osnap_delta", "��ֵ����ƫ�����Ĳ���");
    setTableItem(5, "osnap_max_k_count", "�����ݶȵ�����������");
	setTableItem(6, "f1_weight", "���º󶥵��λ����ԭ��λ�õľ���");
	setTableItem(7, "f2_weight", "���º󶥵�λ����Ŀ�궥��λ�õľ���");
	setTableItem(8, "f3_1_weight", "dist[���º󶥵㣬Ŀ�궥���ǰ���ڵ�] - dist[Ŀ�궥�㣬Ŀ�궥���ǰ���ڵ�]");
	setTableItem(9, "f3_2_weight", "dist[���º󶥵㣬Ŀ�궥��ĺ�̽ڵ�] - dist[Ŀ�궥�㣬Ŀ�궥��ĺ�̽ڵ�]");
	setTableItem(10, "f8_weight", "f5�Ķ�ż�汾(��-��)�����º󶥵�λ��������ƽ�潻�ߵľ���");
	setTableItem(11, "f4_weight", "���º󶥵�λ�õ�ƥ��ߵľ���");
	setTableItem(12, "f5_weight", "���º󶥵�λ��������ƽ�潻�ߵľ���");
	setTableItem(13, "f6_weight", "���º󶥵㵽ƥ��������˵����֮�� - ƥ��߳���");
	setTableItem(14, "f7_weight", "���º󶥵㵽ƥ���ϵ����֧��ƽ��ľ���");
}

void SetParams_MergingVertices_Dialog::setTableItem(int row, QString str1, QString str2)
{
    model->setItem(row,0, new QStandardItem(str1));
    model->setItem(row,1, new QStandardItem(str2));
    QString str3 = "merge vertices/";
    str3.append(str1);
    model->setItem(row,2, new QStandardItem(configIniRead->value(str3).toString()));
}

// ���ò���
void SetParams_MergingVertices_Dialog::on_buttonBox_accepted()
{
    QStringList strList;
    strList << "ratio_of_scale"
            << "T_ratio_lineSeg_middle_area"
            << "T_dist_proj_to_lineSegEnd"
            << "T_maxAngle_bet_two_polys"
            << "osnap_delta"
            << "osnap_max_k_count"
			<< "f1_weight"
			<< "f2_weight"
			<< "f3_1_weight"
			<< "f3_2_weight"
			<< "f8_weight"
			<< "f4_weight"
			<< "f5_weight"
			<< "f6_weight"
			<< "f7_weight";
    int size = strList.size();
    for(int i = 0; i < size; ++i)
    {
        QString key, value;
        key = model->data(model->index(i, 0), Qt::DisplayRole).toString();
        value = model->data(model->index(i, 2), Qt::DisplayRole).toString();
        key = "merge vertices/" + key;
        configIniRead->setValue(key, value);
    }

    QMessageBox::about(NULL, "��ʾ", "���������ú�");
}
