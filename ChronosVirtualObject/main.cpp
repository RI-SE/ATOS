#include "mainwindow.h"
#include <QApplication>
#include <signal.h>

// Create either a terminal application or a gui application
QCoreApplication* createApplication(int &argc, char *argv[])
{
    for (int i = 1; i < argc; ++i)
        if (!qstrcmp(argv[i], "-nogui"))
            return new QCoreApplication(argc, argv);
    return new QApplication(argc, argv);

}

// Handle interrupt
void sigint_handler(int param)
{
    (void)param;
    qDebug() << "\nEnding program.";// Code: "<< QString::number(param);
    exit(1);
}

// Main program
int main(int argc, char *argv[])
{
    //Pointer to application
    QScopedPointer<QCoreApplication> app(createApplication(argc, argv));

    if (qobject_cast<QApplication *>(app.data())) {
       // start GUI version...

        // Enable the meta data to be passed through signals and slots
        // between gui and the virtual object
        qRegisterMetaType<VOBJ_DATA>();

        // Create the gui
        QApplication *a = (QApplication*) app.data();
        MainWindow w;
        w.show();
        return a->exec();
    } else {
        // Add text to the ending of the program
        signal(SIGINT,sigint_handler);

        // Create the console application
        QCoreApplication *a = (QCoreApplication*) app.data();
        // Create a virtual object
        VirtualObject *vobj = new VirtualObject(0);
        // Establish TCP and UDP connection to ports 53240,53241
        vobj->connectToServer(53240,53241);
        // Start the thread with highest priority
        vobj->start(QThread::TimeCriticalPriority);
        return a->exec();
    }

    //return app->exec();
}


