#include "pclviewer.h"
#include <QApplication>
#include <QMainWindow>
#include <QTextCodec>

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);
  QTextCodec::setCodecForCStrings(QTextCodec::codecForName("GB2312"));
  PCLViewer w;
  w.show ();
  w.init();

  return a.exec ();
}
