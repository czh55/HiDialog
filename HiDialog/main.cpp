//以下这句话解决这个错误：https://stackoverflow.com/questions/18642155/no-override-found-for-vtkpolydatamapper
#define vtkRenderingCore_AUTOINIT 2(vtkRenderingOpenGL2, vtkInteractionStyle)

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
