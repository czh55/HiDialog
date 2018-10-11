#include "setbgcolordialog.h"
#include <iostream>

SetBGColorDialog::SetBGColorDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SetBGColorDialog)
{
    ui->setupUi(this);
    ui->colorBHorizontalSlider->setRange(0, 255);
    ui->colorGHorizontalSlider->setRange(0, 255);
    ui->colorRHorizontalSlider->setRange(0, 255);
}

SetBGColorDialog::~SetBGColorDialog()
{
    delete ui;
}

void SetBGColorDialog::on_buttonBox_accepted()
{
    R = ui->colorRLcdNumber->value();
    G = ui->colorGLcdNumber->value();
    B = ui->colorBLcdNumber->value();
}

void SetBGColorDialog::on_buttonBox_rejected()
{
    return;
}
