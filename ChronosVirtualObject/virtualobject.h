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
#include <random>

#define HEARTBEAT_TIME 1000 // maximum time to wait for a heartbeat
#define EARTH_RADIUS 6367000
#define SOCKET_STACK_START 53240

#define MONR_SEND_TIME_INTERVAL 10 // Time to wait between sending monr

/* 34 years between 1970 and 2004, 8 days for leap year between 1970 and 2004      */
/* Calculation: 34 * 365 * 24 * 3600 * 1000 + 8 * 24 * 3600 * 1000 = 1072915200000 */
#define MS_FROM_1970_TO_2004_NO_LEAP_SECS 1072915200000

/* Difference of leap seconds between UTC and ETSI */
#define DIFF_LEAP_SECONDS_UTC_ETSI 5

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
    qint64 mtsp;
} VOBJ_DATA;

Q_DECLARE_METATYPE(VOBJ_DATA)

typedef enum {
    INIT = 1,
    ARMED,
    DISARMED,
    RUNNING,
    STOP,
    ABORT,
    ERROR,
    RUNNING_STANDBY
} OBJ_STATUS;


class VirtualObject : public QThread
{
    Q_OBJECT
public:
    VirtualObject(int ID, double rLat = 0,double rLon = 0, double rAlt = 0);
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
    void simulation_start(int ID);
    void simulation_stop(int ID);


private slots:
    // Slots for Chronos communication
    void handleOSEM(chronos_osem msg);
    void handleDOPM(QVector<chronos_dopm_pt> msg);
    void handleHEAB(chronos_heab msg);
    void handleOSTM(chronos_ostm msg);
    void handleSTRT(chronos_strt msg);
    void handleSYPM(chronos_sypm msg);
    void handleMTSP(chronos_mtsp msg);

    // Slots for UI communication
    void MONREnabledChanged(int ID, bool status);
    void handleMeasurementNoiseToggle(int ID, bool checked,double stddev);
    void stopSimulation();
    //void handleChangedNoise(double mean, double variance);

private:
    bool shutdown = false;

    // Object data
    VOBJ_DATA data;

    // Chronos client
    Chronos* cClient;

    // Status variables
    qint8 status = INIT;
    qint8 pendingStatus = INIT;



    // Check to see if the messages
    bool hasOSEM = false;
    bool hasDOPM = false;

    // Variables for random normaly distributed noise
    bool isMeasurementNoiseEnabled = false;

    std::default_random_engine generator;
    //std::normal_distribution<double> distribution(0.0);
    std::normal_distribution<double> *distribution;


    bool sendMONREnabled = true;



    //Syncronisation variables
    bool isMaster = true;
    qint64 time_adjustment = 0;
    QVector<quint32> sync_points;


    // Time variables
    quint64 heab_recieved_time; // Needed in order to track when Heartbeat came in
    quint64 sleep_time = 20;     // How long the process should be suspended
    quint64 start_ETSI_time;    // The ETSI time to start at
    // The trajectory to follow
    QVector<chronos_dopm_pt> traj;


    // The Reference coordinates
    double mRefLat = 57.71495867;
    double mRefLon = 12.89134921;
    double mRefAlt = 219.0;

    double mRefHeading; // Don't know what use this will be

    /* METHODS */

    // Update the position of the object based on the reference points
    void control_object(chronos_dopm_pt next,chronos_dopm_pt prev);

    // Depending on time and where current reference value is, find the next reference point
    int findRefPoint(qint64 tRel, uint fromIndex,qint64 refTimeOffset);

    // Construct a MONR message from the VirtualObject data
    chronos_monr getMONR();

};

#endif // VIRTUALOBJECT_H
