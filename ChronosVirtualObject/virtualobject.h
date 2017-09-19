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
#include "utility.h"

#define HEARTBEAT_TIME 100 // maximum time to wait for a heartbeat

// Object data of interest
typedef struct {
    int ID;
    qint64 time;
    double x;
    double y;
    double heading;
    double speed;
    double acc;
    double mRefLat;
    double mRefLon;
    double mRefAlt;
} VOBJ_DATA;

Q_DECLARE_METATYPE(VOBJ_DATA)

typedef enum {
    INIT = 0,
    ARMED,
    RUNNING,
    STOPPED,
    ERROR,
    IDLE
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
    void new_trajectory(QVector<chronos_dopm_pt> traj);
    void send_monr(chronos_monr monr);

private slots:
    void handleOSEM(chronos_osem msg);
    void handleDOPM(QVector<chronos_dopm_pt> msg);
    void handleHEAB(chronos_heab msg);

private:
    //QDateTime program_time;

    qint8 status = IDLE;
    Chronos* cClient;

    VOBJ_DATA data;

    qint64 start_time;
    qint64 clock;

    qint64 heab_recieved_time;

    QVector<chronos_dopm_pt> traj;


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
    chronos_monr getMONR();
    void updateTime();

};

#endif // VIRTUALOBJECT_H
