#include "virtualobject.h"

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
        status,    // statusF
        isMaster    // MASTER?
    };

    mRefLat = rLat;
    mRefLon = rLon;
    mRefAlt = rAlt;

    cClient = new Chronos();

    distribution = new std::normal_distribution<double>(0.0,1.0);


}

VirtualObject::~VirtualObject() {
    traj.clear();
    delete distribution;
    delete cClient;
}


void VirtualObject::run()
{
    qDebug() << "Virtual Object Started";

    qint64 clock = 0;       // Current time
    qint64 start_time = 0;  // Start time of the execution of a trajectory
    qint64 simulation_time = 0;   // The total time for the entire simulation
    qint64 elapsed_time = 0; // Time since the start of the execution
    qint64 ctrl_update = 0; // Time since the last control signal update
    qint64 update_sent = 0; // Time since the last MONR message
    quint64 current_ETSI_time = 0;

    // Test variables
    qint64 tmod = 0; // Adding or subtracting time (MTPS)
    int mtps = 0; // Which mtsp test is currently checked

    int ref_index= 0; // Keeps track of which reference is currently followed

    bool init_start = false;
    bool traj_simulation_only = false; // Set true to only output the trajectory points
    bool send_monr_idependently = true;
    chronos_dopm_pt ref_point;      // Placeholder for the reference point
    chronos_dopm_pt prev_ref_point; // Placeholder for the previous ref. point

    while (!shutdown){
        // Loop as long as a shutdown signal has not been sent

        if (status == ERROR)
        {

            // Do something error related
            // This will generate a deadlock

            emit simulation_stop(getID());
        }
        else
        {

            if (status != pendingStatus) // Whenever a state is changed
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
            ref_index = 0;
            init_start = true;
            mtps = 0;

            elapsed_time = 0;



            simulation_time = ((chronos_dopm_pt) (traj.last())).tRel;
        }
        else if(status == DISARMED)
        {
            // Do something during disarmed state
        }
        else if(status == RUNNING_STANDBY)
        {
            current_ETSI_time = QDateTime::currentMSecsSinceEpoch() -
                        MS_FROM_1970_TO_2004_NO_LEAP_SECS + DIFF_LEAP_SECONDS_UTC_ETSI*1000;
            if(current_ETSI_time>=start_ETSI_time) pendingStatus = RUNNING;
        }
        else if(status == RUNNING && traj_simulation_only)
        {
            sleep_time = 5;
            clock = QDateTime::currentMSecsSinceEpoch();
            if (ref_index == 0 && init_start)
            {
                //ref_index = 1;
                //ref_point = traj[ref_index];
                //prev_ref_point = traj[0];

                ref_index = 0;
                ref_point = traj[ref_index];

                start_time = clock;
                init_start = false;
                emit simulation_start(getID());
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
                emit simulation_stop(getID());
            }
            if(ref_index < traj.size() - 1 && elapsed_time < simulation_time)
            {
                elapsed_time = clock - start_time;
                if (elapsed_time > ref_point.tRel)
                {
                    // Update the state
                    data.acc = ref_point.accel;
                    data.heading = ref_point.heading;
                    data.speed = ref_point.speed;
                    data.time = elapsed_time;
                    data.x = ref_point.x;
                    data.y = ref_point.y;
                    data.z = ref_point.z;

                    // Send MONR
                    if (sendMONREnabled && !send_monr_idependently)
                    {
                        cClient->sendMonr(getMONR());
                    }
                    // Update the point
                    ref_index++;
                    ref_point = traj[ref_index];
                }
            }
            else
            {
                pendingStatus = STOP;
                emit simulation_stop(getID());
            }

        }
        else if(status == RUNNING)
        {
            sleep_time = 5;

            clock = QDateTime::currentMSecsSinceEpoch();
            // Is this the first iteration?
            if (ref_index == 0 && init_start)
            {
                ref_index = 1;
                ref_point = traj[ref_index];
                prev_ref_point = traj[0];

                start_time = clock;
                init_start = false;
                emit simulation_start(getID());
            }
            if(ref_index < traj.size() - 1 && elapsed_time < simulation_time)
            {
                elapsed_time = clock - start_time;
                data.time = elapsed_time;
                // Get the reference point
                int index_before_update = ref_index;

                // Set a static point to start deviating
                ref_index = false && elapsed_time > 20000 && getID() == 0
                        ? ref_index : findRefPoint(elapsed_time,ref_index,tmod);
                /*
                if (elapsed_time >10000 && getID()==0 && mtps == 0 && true)
                {
                    //qDebug() << "Modifying refs";
                    tmod=2000;
                    mtps++;
                    //traj[ref_index].tRel += tmod;
                }


                if (elapsed_time >20000 && getID()==0 && mtps == 1 && true)
                {
                    //qDebug() << "Modifying refs";
                    tmod=-1000;
                    mtps++;
                    //traj[ref_index].tRel += tmod;
                }*/


                if (ref_index-index_before_update)
                {
                    // When the index has changed, make sure that the previous reference
                    // point is also updated.
                    prev_ref_point = ref_point;
                    // Set the new reference point
                    ref_point = traj[ref_index];
                    // Add time offset
                    ref_point.tRel +=tmod;
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
                    emit simulation_stop(getID());
                }
                if (clock - ctrl_update > 0)
                {
                    // Move the object along a linear trajectory with constant velocity
                    // from the previous position to the new position
                    control_object(ref_point,prev_ref_point);
                    ctrl_update = clock;
                }
            }
            else
            {
                pendingStatus = STOP;
                emit simulation_stop(getID());
            }
        }

        clock = QDateTime::currentMSecsSinceEpoch();
        if (clock - update_sent > MONR_SEND_TIME_INTERVAL && sendMONREnabled && send_monr_idependently)
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

    return 0;
}

int VirtualObject::getID()
{
    return data.ID;
}


void VirtualObject::control_object(chronos_dopm_pt next,chronos_dopm_pt prev)
{
    // Find the change in both directions
    double deltaY = next.y-prev.y;
    double deltaX = next.x-prev.x;

    // Calculate the constant velocity between the two reference points
    double new_vx = deltaX / (double) (next.tRel-prev.tRel) ;
    double new_vy = deltaY / (double) (next.tRel-prev.tRel) ;

    // Calculate the length of the velocity vector
    double actual_speed = sqrt(new_vx*new_vx+new_vy*new_vy);

    // Update the current position based on the time that has
    // passed since the last reference point
    data.x = prev.x + new_vx * (double)(data.time-prev.tRel);
    data.y = prev.y + new_vy * (double)(data.time-prev.tRel);

    // Update the current state variables
    data.acc = (actual_speed - data.speed)/(data.time-prev.tRel); //prev.accel;
    data.heading = prev.heading;
    data.speed = actual_speed;
}

chronos_monr VirtualObject::getMONR()
{
    chronos_monr monr;    
    double x_noise = 0.0;//(*distribution)(generator);
    double y_noise = 0.0;//(*distribution)(generator);

    if (status == RUNNING && isMeasurementNoiseEnabled)
    {
        x_noise = (*distribution)(generator);
        y_noise = (*distribution)(generator);
    }

    double xyz[3] = {data.x + x_noise, data.y + y_noise, data.z};
    double iLlh[3] = { mRefLat , mRefLon , mRefAlt};
    double llh[3] = { 0 , 0 , 0}; // Long/Lat pos for the object
    // Calculate the Long/Lat pos from the ENU coordinates
    utility::enuToLlh(iLlh,xyz,llh);


    monr.lat=llh[0];
    monr.lon=llh[1];
    monr.alt=llh[2];
    monr.heading = data.heading;
    monr.speed = data.speed;
    monr.direction = 0; // The car is drivning forward
    monr.status = status;

    //QDateTime ETSI_time(QDate(2004,01,01));
    //ETSI_time.setOffsetFromUtc(0);
    //monr.ts = QDateTime::currentMSecsSinceEpoch() - ETSI_time.toMSecsSinceEpoch() + ;

    monr.ts = QDateTime::currentMSecsSinceEpoch() -
                MS_FROM_1970_TO_2004_NO_LEAP_SECS +
                    DIFF_LEAP_SECONDS_UTC_ETSI*1000;

    return monr;
}



int VirtualObject::findRefPoint(qint64 tRel, uint fromIndex, qint64 refTimeOffset)
{
    if (traj.size()<2)
    {
        qDebug() << "findRefPoint ERROR: No traj file loaded";
        return -1;
    }

    if (traj.last().tRel + refTimeOffset < tRel)
    {
        //qDebug() << "findRefPoint ERROR: Relative time out of reach";
        return traj.size()-1;
    }

    if (traj.size() <= (int)fromIndex )
    {
        qDebug() << "Index out of bounds";
    }
    int i=fromIndex;
    for (; i<traj.size()-1;i++)
    {
        if(traj[i].tRel + refTimeOffset > tRel ) return i;
        //&& tRel > traj[i].tRel + refTimeOffset
    }
    qDebug() << "Cannot find a valid point";
    return i;

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
        qDebug() << "OSEM:\n" << QString::number(mRefLat,'f',8) << "\n"
                 << QString::number(mRefLon,'f',8) << "\n"
                 << QString::number(mRefAlt,'f',8);

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
    case 0x01:
        if(status == ARMED)
        {
            pendingStatus = RUNNING;
        }
        break;

    case 0x02:
        // Add code for starting at specified time
        if(status == ARMED)
        {
            start_ETSI_time = msg.ts;
            pendingStatus = RUNNING_STANDBY;
        }
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
            if (findRefPoint(msg.sync_point,0,0))
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
    qDebug() << "MTSP recieved";
    // Double-check what the status should be
    if (status == ARMED || status == RUNNING)
    {
        // Do something

        time_adjustment = msg.ts;
    }
}

void VirtualObject::MONREnabledChanged(int ID, bool status)
{
    if (ID == getID()) sendMONREnabled = status;
}

void VirtualObject::handleMeasurementNoiseToggle(int ID, bool checked, double stddev)
{
    if (ID != getID()) return;
    //if (abs(stddev - distribution->stddev()) > 0.000001)
    //{
        delete distribution;
        distribution = new std::normal_distribution<double>(0.0, stddev);
    //}
    isMeasurementNoiseEnabled = checked;
}

void VirtualObject::stopSimulation()
{
    shutdown = true;
}


