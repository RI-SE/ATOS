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

#define HEARTBEAT_TIME 200 // maximum time to wait for a heartbeat
#define EARTH_RADIUS 6367000
#define SOCKET_STACK_START 53240

// Object data of interest
typedef struct {
    int ID;
    qint64 time;
    double x;
    double y;
    double z;
    double heading;
    double speed;
    double acc;
    qint8 status;
    bool isMaster;
} VOBJ_DATA;

Q_DECLARE_METATYPE(VOBJ_DATA)

typedef enum {
    INIT = 1,
    ARMED,
    DISARMED,
    RUNNING,
    STOP,
    ABORT,
    ERROR
} OBJ_STATUS;


class VirtualObject : public QThread
{
    Q_OBJECT
public:
    VirtualObject(int);
    VirtualObject(int,double rLat,double rLon,double rAlt);
    ~VirtualObject();

    void run();
    int connectToServer(int updSocket,int tcpSocket);
    int getID();
    static void control_function(double* vel,chronos_dopm_pt ref, VOBJ_DATA data );
signals:
    //void updated_position(double x, double y, long t,int ID); // To be removed


    void updated_state(VOBJ_DATA currentState);
    void new_OSEM(chronos_osem msg);
    void new_trajectory(int ID,QVector<chronos_dopm_pt> traj);
    void send_monr(chronos_monr monr);
    void thread_done(int ID);


private slots:
    void handleOSEM(chronos_osem msg);
    void handleDOPM(QVector<chronos_dopm_pt> msg);
    void handleHEAB(chronos_heab msg);
    void handleOSTM(chronos_ostm msg);
    void handleSTRT(chronos_strt msg);
    void handleSYPM(chronos_sypm msg);
    void handleMTSP(chronos_mtsp msg);

    void stopSimulation();

private:
    //QDateTime program_time;

    bool shutdown = false;
    bool hasOSEM = false;
    bool hasDOPM = false;

    qint8 status = INIT;
    qint8 pendingStatus = INIT;
    Chronos* cClient;

    bool isMaster = true;

    VOBJ_DATA data;

    //qint64 start_time;
    //qint64 clock;

    // The time to adjust the trajectory file with
    qint64 time_adjustment = 0;
    // Needed in order to track when Heartbeat came in
    qint64 heab_recieved_time;

    qint64 sleep_time = 2;

    QVector<chronos_dopm_pt> traj;

    double mRefLat = 57.71495867;
    double mRefLon = 12.89134921;
    double mRefAlt = 219.0;

    // Don't know what use these will be
    double mRefHeading;

    //void sendCurrentState();
    void control_object(int curr_idx_point,qint64 deltaT);
    void update_system( double *vel, qint64 deltaT, chronos_dopm_pt ref);
    chronos_monr getMONR();
    void xyz_to_llh(double *lat,double *lon, double *alt);



};

#endif // VIRTUALOBJECT_H
