#ifndef VIRTUALOBJECT_H
#define VIRTUALOBJECT_H

#include <QObject>
#include <QDebug>
#include <QDateTime>
#include <QThread>
#include "locpoint.h"
#include "datatypes.h"
#include <string.h>

typedef enum {
    IDLE = 0,
    INIT,
    ARMED,
    RUNNING,
    STOPPED,
    ERROR
} OBJ_STATUS;


/* TODO: Integrate chronos */
class VirtualObject : public QThread
{
    Q_OBJECT
public:
    VirtualObject(int);
    ~VirtualObject();

    void run();
signals:
    //void updated_position(double x, double y, long t,int ID); // To be removed


    void updated_state(int ID, qint32 t, double x, double y);
    void send_monr(chronos_monr monr);

private slots:


private:
    //QDateTime program_time;

    qint64 start_time;
    qint64 clock;
    int id;
    double x = 0;
    double y = 0;

    double mRefLat = 57.71495867;
    double mRefLon = 12.89134921;
    double mRefAlt = 219.0;

    LocPoint getCurrentState();
/*
    int status;



    double mLat;
    double mLon;
    double mAlt;

    double speed;
    double heading;
    */

    void updateTime();

};

#endif // VIRTUALOBJECT_H
