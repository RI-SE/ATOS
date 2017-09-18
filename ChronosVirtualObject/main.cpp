#include "mainwindow.h"
#include <QApplication>
#include "chronos.h"
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    // Enable the meta data to be passed through signals and slots
    qRegisterMetaType<VOBJ_DATA>();

    return a.exec();
}
