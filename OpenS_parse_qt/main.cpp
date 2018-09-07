#include <QCoreApplication>
#include "oscreader.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);


    QStringList args = a.arguments();

    OSCReader reader;
    if (args.size() < 2)
    {
        qDebug() << "No file to read";
        return -1;
    }


    reader.parseOSFile(args[1]);

    return a.exec();
}
