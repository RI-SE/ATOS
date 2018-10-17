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


    // Print arguments
    qDebug() << "Reading file: " << args[1];

    //reader.parseOSFile(args[1]);

    qDebug() << "ReadToDoc:" << reader.readtoDOMdoc(args[1]);
    //reader.printDOMdoc();
    qDebug() << "LoadDoc:" << reader.loadDomdoc();
    qDebug() << "ReadLoadedDoc" << reader.printLoadedDomDoc();

    //return a.exec();
    return 0;
}
