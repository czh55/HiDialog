#ifndef SETBGCOLORDIALOG_H
#define SETBGCOLORDIALOG_H

#include <QDialog>

namespace Ui {
class SetBGColorDialog;
}

class SetBGColorDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SetBGColorDialog(QWidget *parent = 0);
    ~SetBGColorDialog();

    int R;
    int G;
    int B;
private slots:
    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

private:
    Ui::SetBGColorDialog *ui;
};

#endif // SETBGCOLORDIALOG_H
