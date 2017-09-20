#include "virtualobject.h"

VirtualObject::VirtualObject(int id)
{
    /*
    mRefLat = 0;
    mRefLon = 0;
    mRefAlt = 0;

    mLat = 0;
    mLon = 0;
    mAlt = 0;

    speed = 0;
    heading = 0;

    status = (int) IDLE;
*/
    //program_time = new QDateTime();
    //program_time = QDateTime::currentDateTime();

    start_time = 0;
    clock = 0;

    data= {
        id, // id
        0,  // time
        0,  // x
        0,  // y
        0,  // z
        0,  // heading
        0,  // speed
        0,  // acceleration
        57.71495867,  // ref lat
        12.89134921,  // ref lon
        219.0   // ref alt
    };

    //this->id = id;

    updateTime();

    cClient = new Chronos();

    // Connect signals to SLOTs


    // Start the chronos object
    //cClient->startServer(53240, 53241);
}

VirtualObject::~VirtualObject() {
    delete cClient;
}


void VirtualObject::run()
{
    qDebug() << "Virtual Object Started";

    qint64 elapsed_time = 0;
    qint64 ctrl_update = 0;
    qint64 update_sent = 0;
    qint64 run_time_MS = 15000;
    int index = 0;
    int flag = 1;

    //updateTime();
    clock = QDateTime::currentMSecsSinceEpoch();
    start_time = clock;

    if (status != INIT)
    {
        qDebug() << "Not initialized by server: " << "No trajectory available.";
        qDebug() << "Terminating Virtual Object";
        return;
    }

    status = RUNNING;
    while(index < traj.size())
    {
        // Get the reference point
        chronos_dopm_pt ref_point = traj[index];
        while(ref_point.tRel < elapsed_time)
        {
            ref_point = traj[index++];
        }

        // Check if heartbeat deadline has passed
        /*if(clock-heab_recieved_time > HEARTBEAT_TIME)
        {
            qDebug() << "ERROR: Heartbeat not recieved.";
            status = STOPPED;
            return;

        }
        */

        // Calculate control signal ~ 100 Hz
        // Send control signal ~ 100 Hz
        clock = QDateTime::currentMSecsSinceEpoch();
        elapsed_time = clock-start_time;
        data.time = elapsed_time;
        if (clock-ctrl_update > 5)
        {
            control_object(index);
            ctrl_update = clock;
        }

        // Read and send information ~ 50Hz
        clock = QDateTime::currentMSecsSinceEpoch();
        elapsed_time = clock-start_time;
        data.time = elapsed_time;
        if (clock-update_sent > 20)
        {
            // Send monr
            cClient->sendMonr(getMONR());
            update_sent = clock;
        }

        elapsed_time = clock-start_time;
        data.time = elapsed_time;

        /*
        if (flag){
            data.x=0; data.y=0;
            //emit updated_state(this->id,(qint32) elapsed_time,x,y);

        }
        else
        {
            data.x=5; data.y=5;
            //emit updated_state(this->id,(qint32) elapsed_time,x,y);

        }*/



        // Send vizualizer update
        emit updated_state(data);
        QThread::msleep(2);
    }
    qDebug() << "Done.";
    status = INIT;

}
/*
LocPoint VirtualObject::getCurrentState()
{
    LocPoint ret_val;
    ret_val.setTime(clock-start_time);
    ret_val.setXY(x,y);
    return ret_val;
}*/
/*
void VirtualObject::sendCurrentState()
{
    emit updated_state(this->id,
                       (qint32) elapsed_time,
                       x,
                       y,
                       mRefLat,
                       mRefLon,
                       mRefAlt);
} */

int VirtualObject::connectToServer(int udpSocket,int tcpSocket)
{
    // Perhaps check if sockets are not taken?
    if (!cClient->startServer(udpSocket,tcpSocket)){
        return -1;
    }

    // Make connections
    // Connection to OSEM
    connect(cClient,SIGNAL(handle_osem(chronos_osem)),
            this,SLOT(handleOSEM(chronos_osem)));
    connect(cClient,SIGNAL(handle_dopm(QVector<chronos_dopm_pt>)),
            this,SLOT(handleDOPM(QVector<chronos_dopm_pt>)));

    connect(cClient,SIGNAL(handle_heab(chronos_heab)),
            this,SLOT(handleHEAB(chronos_heab)));
    // make connection to send the MONR
    //connect(this,SIGNAL(send_monr(chronos_monr)),
    //        cClient,SLOT(transmitMONR(chronos_osem)));
    return 0;
}

int VirtualObject::getID()
{
    return data.ID;
}

void VirtualObject::updateTime()
{
    clock = QDateTime::currentMSecsSinceEpoch();
}

void VirtualObject::control_object(int curr_idx_point)
{
    // Only does object = ref_point
    // TODO: add dynamics

    if (curr_idx_point == 0 || curr_idx_point >= traj.size())
        return;

    chronos_dopm_pt next = traj[curr_idx_point];
    chronos_dopm_pt prev = traj[curr_idx_point-1];

    // Find the unity vector
    double deltaY = next.y-prev.y;
    double deltaX = next.x-prev.x;

    double dist = sqrt(deltaY*deltaY+deltaX*deltaX);

    double ex = dist ? deltaX/dist : deltaX;
    double ey = dist ? deltaY/dist : deltaY;

    data.x = prev.x + ex * data.speed / 1000.0 *(data.time-prev.tRel);
    data.y = prev.y + ey * data.speed / 1000.0 *(data.time-prev.tRel);
/*
    data.acc = ref_point.accel;
    data.heading = ref_point.heading;
    data.speed = ref_point.speed;
    data.x = ref_point.x;
    data.y = ref_point.y;
    */

    data.acc = prev.accel;
    data.heading = prev.heading;
    data.speed = prev.speed;
}

chronos_monr VirtualObject::getMONR()
{
    chronos_monr monr;

    //utility::xyzToLlh(data.x,data.y,0,&monr.lat,&monr.lon,&monr.alt);
    xyz_to_llh(&monr.lat,&monr.lon,&monr.alt);
    monr.heading = data.heading;
    monr.speed = data.speed;
    monr.direction = 0; // The car is drivning forward
    monr.status = status;

    //QDate ETSI_date();
    QDateTime ETSI_time(QDate(2004,01,01));
    ETSI_time.setOffsetFromUtc(0);

    monr.ts = QDateTime::currentMSecsSinceEpoch() - ETSI_time.toMSecsSinceEpoch();

    //monr.ts = data.time;
    return monr;
}

void VirtualObject::xyz_to_llh(double *lat,double *lon, double *alt)
{
    *lat = data.mRefLat + data.y * 180.0 / (M_PI * (double) EARTH_RADIUS) ;
    *lon = data.mRefLon + data.x * 180.0 / (M_PI * (double) EARTH_RADIUS * cos(*lat));
    *alt = data.mRefAlt + data.z;
}


// SLOTS

void VirtualObject::handleOSEM(chronos_osem msg)
{
    data.mRefLat = msg.lat;
    data.mRefLon = msg.lon;
    data.mRefAlt = msg.alt;

    //utility::llhToXyz(msg.lat,msg.lon,msg.alt,&data.x,&data.y,&data.z);
    mRefHeading = msg.heading;
    emit updated_state(data);
}

void VirtualObject::handleDOPM(QVector<chronos_dopm_pt> msg)
{
    switch (status) {
    case IDLE:
        traj = msg;

        // Set the current position to be the first point in trajectory file
        if (msg.size()>0){
            chronos_dopm_pt firstPos = msg[0];
            data.x = firstPos.x;
            data.y = firstPos.y;
            data.heading = firstPos.heading;
            data.speed = firstPos.speed;
            data.acc = firstPos.accel;

            status = INIT;

            emit new_trajectory(msg);
            emit updated_state(data);
        }
        else
        {
            qDebug() << "ERROR: No points in DOPM.";
        }



        break;
    default:
        break;
    }

}
void VirtualObject::handleHEAB(chronos_heab msg)
{
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    //qint64 e_time = currentTime-heab_recieved_time;
    //qDebug() << QString::number(e_time);
    heab_recieved_time = currentTime;
}




