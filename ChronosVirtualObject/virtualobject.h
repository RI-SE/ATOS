#ifndef VIRTUALOBJECT_H
#define VIRTUALOBJECT_H

#include <QObject>
#include <QDebug>
#include <QThread>
//#include "chronos.h"
#include "isocom.h"
#include "locpoint.h"
#include "datatypes.h"
#include <string.h>
#include "utility.h"
#include <random>

#define HEARTBEAT_TIME 1000 // maximum time to wait for a heartbeat
#define EARTH_RADIUS 6367000
#define SOCKET_STACK_START 53240

#define MONR_SEND_TIME_INTERVAL 10 // Time to wait between sending monr



// Object data of interest
typedef struct {
    int ID;             // Virtual Object ID
    quint64 time;       // Object perception of the current time
    double x;           // x-coordinate
    double y;           // y-coordinate
    double z;           // z-coordinate
    double heading;
    double speed;
    double acc;
    qint8 status;
    bool isMaster;
} VOBJ_DATA;

typedef struct {
    quint64 clock;                      // Holding the current time value [ms] (UTC)
    quint64 test_start_time;            // Start time [ms] (UTC)
    qint64 elapsed_time;                // Time since start of test
    quint64 position_update_time;       // Time at the latest position update [ms] (UTC)
    quint64 HEAB_rec_time;              // Time at which the latest HEAB message was received [ms] (UTC)
    quint64 monr_send_time;             // Time at the most recent sent MONR message [ms] (UTC)
    quint64 sleep_time;                 // Decides downtime between loops during runtime [ms]
    quint64 runtime_start;              // The time at which the object entered its running state
    quint64 time_since_runtime_start;   // The time since runtime start
} VOBJ_TIME_DATA;

typedef struct {
    quint32 date;                   // GPS date sent from server represented as integer ex. 20180801 for 1st of august 2018
    quint16 gps_week;               // GPS week sent from server
    quint32 gps_qmsOfWeek;          // Quarter ms of the current week sent from server
    quint64 HEAB_server_send_time;  // Time of transmission from server to object
} SRV_TIME_DATA;

Q_DECLARE_METATYPE(VOBJ_DATA)

typedef enum {
    INIT = 1,
    ARMED,
    DISARMED,
    RUNNING,
    POSTRUN,
    REMOTECTRL,
    PRERUN
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
    void getRefLLH(double&,double&,double&);
    bool initObjectState();
    //static void control_function(double* vel,chronos_dopm_pt ref, VOBJ_DATA data );

signals:
    //void updated_position(double x, double y, long t,int ID); // To be removed


    void updated_state(VOBJ_DATA currentState);
    void new_origin(double lat, double lon, double alt);
    void new_trajectory(int ID,QVector<dotm_pt> traj);
    void thread_done(int ID);
    void simulation_start(int ID);
    void simulation_stop(int ID);


    //Not yet connected
    void forward_tcm(int ID, bool active);
    void forward_mtsp(int ID, qint64 sim_time, qint64 deltaT);


private slots:
    // Slots for Chronos communication
    void handleOSEM(osem msg);
    void handleDOPM(QVector<dotm_pt> msg);
    void handleLoadedDOPM(int ID, QVector<dotm_pt> msg);
    void handleHEAB(heab msg);
    void handleOSTM(ostm msg);
    void handleSTRT(strt msg);
    void handleSYPM(chronos_sypm msg);
    void handleMTSP(chronos_mtsp msg);
    void handleTCM(chronos_tcm msg);

    void triggerOccured(int);
    // Slots for UI communication
    void MONREnabledChanged(int ID, bool status);
    void handleMeasurementNoiseToggle(int ID, bool checked,double stddev);
    void handleTrajSimDelayToggle(int ID, bool checked, double delayFactor);
    void stopSimulation();
    //void handleChangedNoise(double mean, double variance);

private:
    bool shutdown = false;

    // Object data
    VOBJ_DATA data;

    VOBJ_TIME_DATA timedata;


    // ISO client
    ISOcom* iClient;

    // Status variables
    qint8 status = INIT;
    qint8 pendingStatus = INIT;


    // Check to see if the messages has been received
    bool hasOSEM = false;
    bool hasDOPM = false;

    // Variables for random normaly distributed noise
    bool isMeasurementNoiseEnabled = false;

    std::default_random_engine generator;
    std::normal_distribution<double> *distribution;

    // Set true to only output the trajectory points
    bool traj_simulation_only = false;
    // Delay simulation factor
    double delay_simulation_factor = 1.0;


    bool sendMONREnabled = true;


    // TAA vairables (can only handle on trigger)
    bool isTCMReceived = false;
    uint16_t TAA_delay = 0;
    uint8_t TAA_ID = 0;
    uint8_t TAA_action = 0;
    uint8_t TAA_trigger_type = 0;

    //Syncronisation variables
    bool isMaster = true;
    bool first_mtsp_received = false;
    uint64_t first_mtsp = 0;
    qint64 time_adjustment = 0;
    QVector<quint32> sync_points;
    QVector<quint32> sync_stop_points;


    // Time variables
    //quint64 last_received_heab_time_from_server; // Needed in order to track when Heartbeat came in
    quint64 sleep_time = 20;     // How long the process should be suspended



    // Trajectory
    int reference_point_index = 0; // Index for the reference point
    QVector<dotm_pt> traj; // The trajectory to follow

    // The Reference coordinates
    double mRefLat = 57.71495867;
    double mRefLon = 12.89134921;
    double mRefAlt = 219.0;

    double mRefHeading; // Don't know what use this will be

    SRV_TIME_DATA srv_timedata;


    /* METHODS */

    // Update the position of the object based on the reference points
    void control_object(chronos_dopm_pt next,chronos_dopm_pt prev);

    // Depending on time and where current reference value is, find the next reference point
    int findRefPoint(qint64 tRel, uint fromIndex,qint64 refTimeOffset);

    // Construct a MONR message from the VirtualObject data
    monr getMONR();

};

#endif // VIRTUALOBJECT_H
