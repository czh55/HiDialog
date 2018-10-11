/********************************************************************************
** Form generated from reading UI file 'setbgcolordialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SETBGCOLORDIALOG_H
#define UI_SETBGCOLORDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSlider>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SetBGColorDialog
{
public:
    QWidget *widget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QSlider *colorRHorizontalSlider;
    QLCDNumber *colorRLcdNumber;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QSlider *colorGHorizontalSlider;
    QLCDNumber *colorGLcdNumber;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_3;
    QSlider *colorBHorizontalSlider;
    QLCDNumber *colorBLcdNumber;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *SetBGColorDialog)
    {
        if (SetBGColorDialog->objectName().isEmpty())
            SetBGColorDialog->setObjectName(QStringLiteral("SetBGColorDialog"));
        SetBGColorDialog->resize(400, 163);
        widget = new QWidget(SetBGColorDialog);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(10, 10, 381, 141));
        verticalLayout = new QVBoxLayout(widget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label = new QLabel(widget);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout->addWidget(label);

        colorRHorizontalSlider = new QSlider(widget);
        colorRHorizontalSlider->setObjectName(QStringLiteral("colorRHorizontalSlider"));
        colorRHorizontalSlider->setOrientation(Qt::Horizontal);

        horizontalLayout->addWidget(colorRHorizontalSlider);

        colorRLcdNumber = new QLCDNumber(widget);
        colorRLcdNumber->setObjectName(QStringLiteral("colorRLcdNumber"));

        horizontalLayout->addWidget(colorRLcdNumber);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_2 = new QLabel(widget);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_2->addWidget(label_2);

        colorGHorizontalSlider = new QSlider(widget);
        colorGHorizontalSlider->setObjectName(QStringLiteral("colorGHorizontalSlider"));
        colorGHorizontalSlider->setOrientation(Qt::Horizontal);

        horizontalLayout_2->addWidget(colorGHorizontalSlider);

        colorGLcdNumber = new QLCDNumber(widget);
        colorGLcdNumber->setObjectName(QStringLiteral("colorGLcdNumber"));

        horizontalLayout_2->addWidget(colorGLcdNumber);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        label_3 = new QLabel(widget);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout_3->addWidget(label_3);

        colorBHorizontalSlider = new QSlider(widget);
        colorBHorizontalSlider->setObjectName(QStringLiteral("colorBHorizontalSlider"));
        colorBHorizontalSlider->setOrientation(Qt::Horizontal);

        horizontalLayout_3->addWidget(colorBHorizontalSlider);

        colorBLcdNumber = new QLCDNumber(widget);
        colorBLcdNumber->setObjectName(QStringLiteral("colorBLcdNumber"));

        horizontalLayout_3->addWidget(colorBLcdNumber);


        verticalLayout->addLayout(horizontalLayout_3);

        buttonBox = new QDialogButtonBox(widget);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        verticalLayout->addWidget(buttonBox);


        retranslateUi(SetBGColorDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), SetBGColorDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), SetBGColorDialog, SLOT(reject()));
        QObject::connect(colorRHorizontalSlider, SIGNAL(valueChanged(int)), colorRLcdNumber, SLOT(display(int)));
        QObject::connect(colorGHorizontalSlider, SIGNAL(valueChanged(int)), colorGLcdNumber, SLOT(display(int)));
        QObject::connect(colorBHorizontalSlider, SIGNAL(valueChanged(int)), colorBLcdNumber, SLOT(display(int)));

        QMetaObject::connectSlotsByName(SetBGColorDialog);
    } // setupUi

    void retranslateUi(QDialog *SetBGColorDialog)
    {
        SetBGColorDialog->setWindowTitle(QApplication::translate("SetBGColorDialog", "Dialog", Q_NULLPTR));
        label->setText(QApplication::translate("SetBGColorDialog", "R:", Q_NULLPTR));
        label_2->setText(QApplication::translate("SetBGColorDialog", "G:", Q_NULLPTR));
        label_3->setText(QApplication::translate("SetBGColorDialog", "B:", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SetBGColorDialog: public Ui_SetBGColorDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SETBGCOLORDIALOG_H
