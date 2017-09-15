#ifndef VIRTUALOBJECT_H
#define VIRTUALOBJECT_H

#include <QObject>
#include <QDebug>
#include <QDateTime>
#include <QThread>
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
    VirtualObject();
    ~VirtualObject();

    void run();
signals:
    void updated_position(double x, double y, long t);
    void send_monr(chronos_monr monr);

private:
    //QDateTime program_time;

    qint64 start_time;
    qint64 clock;
/*
    int status;

    double mRefLat;
    double mRefLon;
    double mRefAlt;

    double mLat;
    double mLon;
    double mAlt;

    double speed;
    double heading;
    */

    void updateTime();

};

#endif // VIRTUALOBJECT_H
