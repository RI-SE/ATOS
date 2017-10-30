#include <QCoreApplication>

#include "tcptextsender.h"
int main(int argc, char *argv[])
{
    // fnts [port] [filename]
    QCoreApplication a(argc, argv);

    qDebug() << "argc=" << QString::number(argc);
    if(argc != 3)
    {
        exit(1);
    }

    QString filename = argv[2];
    TCPTextSender *tts = new TCPTextSender(atoi(argv[1]), filename);
    tts->start();


    return a.exec();
}
