#include "pclviewer.h"
#include <QApplication>
#include <QMainWindow>
#include <QTextCodec>

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);
  PCLViewer w;
  w.show ();
  w.init();

  return a.exec ();
}
