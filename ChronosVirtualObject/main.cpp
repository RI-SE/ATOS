#include "mainwindow.h"
#include <QApplication>
#include "chronos.h"
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    Chronos chronos;
    PacketInterface mPacketInt;

    chronos.startServer(&mPacketInt);


    return a.exec();
}
