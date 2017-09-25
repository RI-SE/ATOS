#include "virtualobject.h"

VirtualObject::VirtualObject(int id)
{

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
        status    // status
    };

    //this->id = id;

    updateTime();

    cClient = new Chronos();

    // Connect signals to SLOTs


    // Start the chronos object
    //cClient->startServer(53240, 53241);
}

VirtualObject::VirtualObject(int id,double rLat,double rLon,double rAlt)
{

    start_time = 0;
    clock = 0;
    time_adjustment=0;

    data= {
        id, // id
        0,  // time
        0,  // x
        0,  // y
        0,  // z
        0,  // heading
        0,  // speed
        0,  // acceleration
        status    // status
    };

    mRefLat = rLat;
    mRefLon = rLon;
    mRefAlt = rAlt;

    cClient = new Chronos();

}

VirtualObject::~VirtualObject() {
    delete cClient;
}


void VirtualObject::run()
{
    qDebug() << "Virtual Object Started";

    qint64 elapsed_time = 0; // Time since the start of the execution
    qint64 ctrl_update = 0; // Time since the last control signal update
    qint64 update_sent = 0; // Time since the last MONR message
    int index = 0;
    bool init_start = false;



    //updateTime();
    while (!shutdown){

        if (status == ERROR)
        {
            // Do something error related
            // This will generate a deadlock
        }
        else
        {
            if (status != pendingStatus)
            {
                qDebug() << "STATUS: " << QString::number(pendingStatus);
            }
            // Update with any new pending status
            status = pendingStatus;
        }

        if(status == STOP)
        {
            // Do something during stopped state
        }
        else if(status == ABORT)
        {
            // Do something during abort
        }

        if (status == INIT)
        {
            // Do something in the init state
        }
        else if(status == ARMED)
        {
            // Reset the running index to the first trajectory point
            index = 0;
            init_start = true;
            sleep_time = 20;
        }
        else if(status == DISARMED)
        {
            // Do something during disarmed state
        }
        else if(status == RUNNING)
        {
            sleep_time = 2;
            clock = QDateTime::currentMSecsSinceEpoch();
            // Is this the first iteration?
            if (index == 0 && init_start)
            {
                start_time = clock;
                init_start = false;
            }
            if(index < traj.size())
            {
                elapsed_time = clock - start_time;
                // Get the reference point
                chronos_dopm_pt ref_point = traj[index];
                while(ref_point.tRel < elapsed_time && index < traj.size())
                {
                    ref_point = traj[index++];
                }

                if (status == ABORT )
                {
                    qDebug() << "ABORT: Aborting test scenario.";
                    break;
                }
                // Check if heartbeat deadline has passed
                if(clock-heab_recieved_time > HEARTBEAT_TIME)
                {
                    qDebug() << "ERROR: Heartbeat not recieved.";
                    //pendingStatus = ERROR;

                }


                // Calculate control signal ~ 100 Hz
                clock = QDateTime::currentMSecsSinceEpoch();
                elapsed_time = clock-start_time;
                data.time = elapsed_time;
                if (clock - ctrl_update > 5)
                {
                    control_object(index,clock-ctrl_update);
                    ctrl_update = clock;
                }

                // Read and send information ~ 50Hz
                clock = QDateTime::currentMSecsSinceEpoch();
                elapsed_time = clock-start_time;
                data.time = elapsed_time;

                if (elapsed_time > 30000)
                {
                    //time_adjustment = 500;
                }

            }
            else
            {
                pendingStatus = STOP;
            }
        }

        clock = QDateTime::currentMSecsSinceEpoch();
        if (clock - update_sent > 20)
        {
            // Send monr
            cClient->sendMonr(getMONR());
            update_sent = clock;
        }

        // Send vizualizer update
        data.status = status;
        emit updated_state(data);
        QThread::msleep(sleep_time);

    }
    qDebug() << "Done.";
    emit thread_done(data.ID);

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
    qDebug() << "Connected to Sockets:" << QString::number(udpSocket) << QString::number(tcpSocket);

    // Make connections
    // Connection to OSEM
    connect(cClient,SIGNAL(handle_osem(chronos_osem)),
            this,SLOT(handleOSEM(chronos_osem)));
    connect(cClient,SIGNAL(handle_dopm(QVector<chronos_dopm_pt>)),
            this,SLOT(handleDOPM(QVector<chronos_dopm_pt>)));
    connect(cClient,SIGNAL(handle_heab(chronos_heab)),
            this,SLOT(handleHEAB(chronos_heab)));
    connect(cClient,SIGNAL(handle_ostm(chronos_ostm)),
            this,SLOT(handleOSTM(chronos_ostm)));
    connect(cClient,SIGNAL(handle_strt(chronos_strt)),
            this,SLOT(handleSTRT(chronos_strt)));
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

void VirtualObject::control_object(int curr_idx_point, qint64 deltaT)
{
    // Only does object = ref_point
    // TODO: add dynamics

    double t = (double) deltaT / 1000.0;

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

    //double s = next.speed * t - 0.5 * next.accel * t * t;

    data.x = prev.x + ex * data.speed / 1000.0 *(data.time-prev.tRel);
    data.y = prev.y + ey * data.speed / 1000.0 *(data.time-prev.tRel);

    //data.x = data.x + ex * s;
    //data.y = data.y + ey * s;
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
    *lat = mRefLat + data.y * 180.0 / (M_PI * (double) EARTH_RADIUS) ;
    *lon = mRefLon + data.x * 180.0 / (M_PI * (double) EARTH_RADIUS * cos(*lat));
    *alt = mRefAlt + data.z;
}


// SLOTS

void VirtualObject::handleOSEM(chronos_osem msg)
{
    switch (status) {
    case INIT:
    case DISARMED:
        mRefLat = msg.lat;
        mRefLon = msg.lon;
        mRefAlt = msg.alt;

        //utility::llhToXyz(msg.lat,msg.lon,msg.alt,&data.x,&data.y,&data.z);
        mRefHeading = msg.heading;

        hasOSEM = true;

        emit new_OSEM(msg);
        break;
    default:
        break;
    }

}

void VirtualObject::handleDOPM(QVector<chronos_dopm_pt> msg)
{
    switch (status) {
    case INIT:
    case DISARMED:


        // Set the current position to be the first point in trajectory file
        if (msg.size()>0){
            chronos_dopm_pt firstPos = msg[0];
            data.x = firstPos.x;
            data.y = firstPos.y;
            data.heading = firstPos.heading;
            data.speed = firstPos.speed;
            data.acc = firstPos.accel;

            // set the new trajectory
            traj = msg;

            hasDOPM = true;

            emit new_trajectory(data.ID,msg);
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
    heab_recieved_time = currentTime;

    switch (msg.status) {
    case 0x02:
        pendingStatus = ABORT;
        qDebug() << "ABOTRING!";
        break;
    default:
        break;
    }
}

void VirtualObject::handleOSTM(chronos_ostm msg)
{

    qDebug() << "ARM-message!";
    switch (msg.armed) {
    case 0x01:
        if ((status == INIT || status == DISARMED ||
                status == STOP || status == ABORT) &&
                hasDOPM && hasOSEM)
            // Maybe status == ERROR as well
        {
            pendingStatus = ARMED;
        }
        break;
    case 0x02:
        if (status == STOP || status == ABORT ||
                status == ARMED)
            // Maybe status == ERROR as well
        {
            pendingStatus = DISARMED;
        }
        break;
    default:
        break;
    }
}
void VirtualObject::handleSTRT(chronos_strt msg)
{
    switch (msg.type) {
    case 0x02:
        if(status == ARMED)
        {
            pendingStatus = RUNNING;
        }
        break;

    case 0x01:
        // Add code for starting at specified time
        break;
    default:
        break;
    }
}

void VirtualObject::stopSimulation()
{
    shutdown = true;
}


