#ifndef VIRTUALOBJECT_H
#define VIRTUALOBJECT_H

#include <QObject>
#include <QDebug>
#include <QDateTime>
#include <QThread>
#include "chronos.h"
#include "locpoint.h"
#include "datatypes.h"
#include <string.h>


// Object data of interest
typedef struct {
    int ID;
    qint64 time;
    double x;
    double y;
    double mRefLat;
    double mRefLon;
    double mRefAlt;
} VOBJ_DATA;

Q_DECLARE_METATYPE(VOBJ_DATA)

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
    int connectToServer(int updSocket,int tcpSocket);
    int getID();
signals:
    //void updated_position(double x, double y, long t,int ID); // To be removed


    void updated_state(VOBJ_DATA currentState);
    void send_monr(chronos_monr monr);

private slots:
    void handleOSEM(chronos_osem msg);
    void handleDOPM(chronos_dopm_pt msg);

private:
    //QDateTime program_time;

    Chronos* cClient;

    VOBJ_DATA data;

    qint64 start_time;
    qint64 clock;


    // Don't know what use these will be
    double mRefHeading;

    /*
    int id;
    double x = 0;
    double y = 0;

    double mRefLat = 57.71495867;
    double mRefLon = 12.89134921;
    double mRefAlt = 219.0;
*/
    //LocPoint getCurrentState();
/*
    int status;



    double mLat;
    double mLon;
    double mAlt;

    double speed;
    double heading;
    */

    //void sendCurrentState();

    void updateTime();

};

#endif // VIRTUALOBJECT_H
