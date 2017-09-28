#include "virtualobject.h"

VirtualObject::VirtualObject(int id)
{

    //start_time = 0;
    //clock = 0;

    data= {
        id, // id
        0,  // time
        0,  // x
        0,  // y
        0,  // z
        0,  // heading
        0,  // speed
        0,  // acceleration
        status,    // status
        isMaster    // MASTER?
    };

    //this->id = id;

    //updateTime();

    cClient = new Chronos();

    // Connect signals to SLOTs


    // Start the chronos object
    //cClient->startServer(53240, 53241);
}

VirtualObject::VirtualObject(int id,double rLat,double rLon,double rAlt)
{

    //start_time = 0;
    //clock = 0;
    //time_adjustment=0;

    data= {
        id, // id
        0,  // time
        0,  // x
        0,  // y
        0,  // z
        0,  // heading
        0,  // speed
        0,  // acceleration
        status,    // status
        isMaster    // MASTER?
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

    qint64 clock = 0;       // Current time
    qint64 start_time = 0;  // Start time of the execution of a trajectory
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
        }
        else if(status == DISARMED)
        {
            // Do something during disarmed state
        }
        else if(status == RUNNING)
        {
            sleep_time = 50;
            clock = QDateTime::currentMSecsSinceEpoch();
            // Is this the first iteration?
            if (index == 0 && init_start)
            {
                index = 1;
                start_time = clock;
                init_start = false;
            }
            if(index < traj.size() - 1)
            {
                elapsed_time = clock - start_time;
                data.time = elapsed_time;
                // Get the reference point
                chronos_dopm_pt ref_point = traj[index];
                // TODO: This loop has to change to enable sync points
                while(ref_point.tRel < elapsed_time && index < traj.size()-1)
                {
                    ref_point = traj[index++];
                }

                if (status == ABORT )
                {
                    qDebug() << "ABORT: Aborting test scenario.";
                    break;
                }
                qint64 time_since_HB = clock-heab_recieved_time;
                // Check if heartbeat deadline has passed
                if(time_since_HB > HEARTBEAT_TIME)
                {
                    qDebug() << "ERROR: Heartbeat not recieved.";
                    pendingStatus = ERROR;

                }


                // Calculate control signal
                /*
                clock = QDateTime::currentMSecsSinceEpoch();
                elapsed_time = clock-start_time;
                data.time = elapsed_time;*/
                if (clock - ctrl_update > 5)
                {
                    //double vel[2];
                    // Calculate the desired control signal
                    //control_function(vel,ref_point,data);

                    //update_system(vel,clock-ctrl_update, ref_point);


                    control_object(traj[index],traj[index-1]);
                    ctrl_update = clock;
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
        data.isMaster = isMaster;
        emit updated_state(data);
        QThread::msleep(sleep_time);

    }
    qDebug() << "Done.";
    emit thread_done(data.ID);

}


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
    connect(cClient,SIGNAL(handle_sypm(chronos_sypm)),
            this,SLOT(handleSYPM(chronos_sypm)));
    connect(cClient,SIGNAL(handle_mtsp(chronos_mtsp)),
            this,SLOT(handleMTSP(chronos_mtsp)));
    // make connection to send the MONR
    //connect(this,SIGNAL(send_monr(chronos_monr)),
    //        cClient,SLOT(transmitMONR(chronos_osem)));
    return 0;
}

int VirtualObject::getID()
{
    return data.ID;
}
/*
void VirtualObject::updateTime()
{
    clock = QDateTime::currentMSecsSinceEpoch();
}*/

void VirtualObject::control_object(chronos_dopm_pt next,chronos_dopm_pt prev)
{
    // Only does object = ref_point
    // TODO: add dynamics

    //double t = (double) deltaT / 1000.0;

    //if (curr_idx_point == 0 || curr_idx_point >= traj.size())
      //  return;

    //chronos_dopm_pt next = traj[curr_idx_point];
    //chronos_dopm_pt prev = traj[curr_idx_point-1];

    // Find the change in both directions
    double deltaY = next.y-prev.y;
    double deltaX = next.x-prev.x;

    // Calculate the constant velocity between the two reference points
    double new_vx = deltaX / (double) (next.tRel-prev.tRel) ;
    double new_vy = deltaY / (double) (next.tRel-prev.tRel) ;

    // Calculate the length of the velocity vector
    double actual_speed = sqrt(new_vx*new_vx+new_vy*new_vy);
    /*
    qDebug() << "Speed Current: actual=" << QString::number(actual_speed)
             << "\nSpeed REF: ref_speed_1=" << QString::number(next.speed)
             << "\nSpeed REF: ref_speed_0=" << QString::number(prev.speed);
    */
    //double dist = sqrt(deltaY*deltaY+deltaX*deltaX);

    //double ex = dist ? deltaX/dist : deltaX;
    //double ey = dist ? deltaY/dist : deltaY;

    //data.x = prev.x + ex * actual_speed / 1000.0 *l(data.time-prev.tRel);
    //data.y = prev.y + ey * actual_speed / 1000.0 *(data.time-prev.tRel);


    // Update the current position based on the time that has
    // passed since the last reference point
    data.x = prev.x + new_vx * (double)(data.time-prev.tRel);
    data.y = prev.y + new_vy * (double)(data.time-prev.tRel);

    // Update the current state variables
    data.acc = (actual_speed - data.speed)/(data.time-prev.tRel); //prev.accel;
    data.heading = prev.heading;
    data.speed = actual_speed;//prev.speed;
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

int VirtualObject::findRefPoint(quint64 tRel, uint fromIndex)
{
    if (traj.size()<2)
    {
        qDebug() << "findRefPoint ERROR: No traj file loaded";
        return -1;
    }

    if (traj.last().tRel < tRel)
    {
        qDebug() << "findRefPoint ERROR: Relative time out of reach";
        return -1;
    }

    if (traj.size() <= fromIndex )
    {
        qDebug() << "Index out of bounds";
    }

    for (int i = fromIndex; i<traj.size()-1;i++)
    {
        if(traj[i+1].tRel > tRel && tRel > traj[i].tRel) return i+1;
    }
    qDebug() << "Cannot find a valid point";
    return -1;

}

void VirtualObject::control_function(double* vel,chronos_dopm_pt ref, VOBJ_DATA data )
{

    double deltaY = ref.y-data.y;
    double deltaX = ref.x-data.x;

    qint64 deltaT = (ref.tRel - data.time);
    if (deltaT < 0.0) qDebug() << "Time is negative. Something is wrong.";

    vel[0] = deltaT ? deltaX/ (double) (deltaT /1000.0) : 0;
    vel[1] = deltaT ? deltaY/ (double) (deltaT /1000.0) : 0;
}

void VirtualObject::update_system(double *vel,qint64 deltaT, chronos_dopm_pt ref)
{
    double prev_x = data.x;
    double prev_y = data.y;
    double prev_v = data.speed;



    data.x = prev_x + vel[0] * (double) deltaT/1000.0;
    data.y = prev_y + vel[1] * (double) deltaT/1000.0;
    data.z = ref.z;

    data.speed = utility::twoNorm(vel,2);
    data.acc = (data.speed - prev_v)/(deltaT/1000.0);
    data.heading = ref.heading;
    data.time += deltaT;
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
    //qDebug() << "Time since last HB: " << QString::number(currentTime-heab_recieved_time);
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

void VirtualObject::handleSYPM(chronos_sypm msg)
{
    if ((status == DISARMED || status == INIT) &&
            hasDOPM && hasOSEM)
    {
        if (traj.last().tRel < msg.sync_point)
        {
            qDebug() << "SYPM ERROR: Not a valid sync point";
        }
        else
        {
            if (findRefPoint(msg.sync_point,0))
            {
                sync_points.append(msg.sync_point);
                qDebug() << "Object updated with SYPM\nTime t=" << QString::number(msg.sync_point);
                isMaster = false;
            }
        }
    }
}

void VirtualObject::handleMTSP(chronos_mtsp msg)
{
    // Double-check what the status should be
    if (status == ARMED || status == RUNNING)
    {
        // Do something
    }
}

void VirtualObject::stopSimulation()
{
    shutdown = true;
}


