#include "mainwindow.h"
#include <QApplication>
#include <signal.h>

QCoreApplication* createApplication(int &argc, char *argv[]);
void sigint_handler(int);

int main(int argc, char *argv[])
{
    QScopedPointer<QCoreApplication> app(createApplication(argc, argv));

    if (qobject_cast<QApplication *>(app.data())) {
       // start GUI version...
        // Enable the meta data to be passed through signals and slots
        qRegisterMetaType<VOBJ_DATA>();
        QApplication *a = (QApplication*) app.data();
        MainWindow w;
        w.show();
        return a->exec();
    } else {
        QCoreApplication *a = (QCoreApplication*) app.data();
        signal(SIGINT,sigint_handler);
        VirtualObject *vobj = new VirtualObject(0);
        vobj->connectToServer(53240,53241);
        QObject::connect(a,SIGNAL(aboutToQuit()),vobj,SLOT(stopSimulation()));
        QObject::connect(a,SIGNAL(aboutToQuit()),vobj,SLOT(deleteLater()));
        vobj->start(QThread::TimeCriticalPriority);
        return a->exec();
    }

    //return app->exec();
}

QCoreApplication* createApplication(int &argc, char *argv[])
{
    for (int i = 1; i < argc; ++i)
        if (!qstrcmp(argv[i], "-gui"))
            return new QApplication(argc, argv);
    return new QCoreApplication(argc, argv);
}

void sigint_handler(int param)
{
    (void)param;
    qDebug() << "\nEnding program.";// Code: "<< QString::number(param);
    exit(1);
}
