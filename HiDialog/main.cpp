#include "HiDialog.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	HiDialog w;
	w.show();
	return a.exec();
}
