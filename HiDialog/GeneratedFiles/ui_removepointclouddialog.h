/********************************************************************************
** Form generated from reading UI file 'removepointclouddialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_REMOVEPOINTCLOUDDIALOG_H
#define UI_REMOVEPOINTCLOUDDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RemovePointCloudDialog
{
public:
    QWidget *widget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *lineEdit;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *RemovePointCloudDialog)
    {
        if (RemovePointCloudDialog->objectName().isEmpty())
            RemovePointCloudDialog->setObjectName(QStringLiteral("RemovePointCloudDialog"));
        RemovePointCloudDialog->resize(369, 94);
        widget = new QWidget(RemovePointCloudDialog);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(10, 10, 351, 71));
        verticalLayout = new QVBoxLayout(widget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label = new QLabel(widget);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout->addWidget(label);

        lineEdit = new QLineEdit(widget);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));

        horizontalLayout->addWidget(lineEdit);


        verticalLayout->addLayout(horizontalLayout);

        buttonBox = new QDialogButtonBox(widget);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        verticalLayout->addWidget(buttonBox);


        retranslateUi(RemovePointCloudDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), RemovePointCloudDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), RemovePointCloudDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(RemovePointCloudDialog);
    } // setupUi

    void retranslateUi(QDialog *RemovePointCloudDialog)
    {
        RemovePointCloudDialog->setWindowTitle(QApplication::translate("RemovePointCloudDialog", "Dialog", Q_NULLPTR));
        label->setText(QApplication::translate("RemovePointCloudDialog", "Point Cloud ID:", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class RemovePointCloudDialog: public Ui_RemovePointCloudDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_REMOVEPOINTCLOUDDIALOG_H
