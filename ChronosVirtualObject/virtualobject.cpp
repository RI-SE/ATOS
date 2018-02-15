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


    //cClient = new Chronos();

    iClient = new ISOcom();

    distribution = new std::normal_distribution<double>(0.0,1.0);


}

VirtualObject::~VirtualObject() {
    traj.clear();
    delete distribution;
    delete iClient;
}


void VirtualObject::run()
{
    qDebug() << "Virtual Object Started";

    qint64 clock = 0;       // Current time
    qint64 start_time = 0;  // Start time of the execution of a trajectory
    quint64 start_ETSI_time = 0;
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
    qint64 time_since_update = 0; // Used in simulated trajectory

    bool test_simulation = false;
    bool send_monr_idependently = true;
    chronos_dopm_pt ref_point;      // Placeholder for the reference point
    chronos_dopm_pt prev_ref_point; // Placeholder for the previous ref. point

    while (!shutdown){
        // Loop as long as a shutdown signal has not been sent

        if (status == ERROR)
        {

            // Do something error related
            // This will generate a deadlock
            //qDebug() << "ERROR";
            //emit simulation_stop(getID());
            //shutdown = true;
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

            time_since_update = 0;

            test_simulation = traj_simulation_only;

            simulation_time = ((chronos_dopm_pt) (traj.last())).tRel;
        }
        else if(status == DISARMED)
        {
            // Do something during disarmed state
        }
        else if(status == RUNNING_STANDBY)
        {
            current_ETSI_time = utility::getCurrentETSItimeMS();
            if(current_ETSI_time>=start_ETSI_time)
            {
                pendingStatus = RUNNING;
            }
        }
        else if(status == RUNNING && test_simulation)
        {
            sleep_time = 5;
            clock = utility::getCurrentUTCtimeMS();
            if (ref_index == 0 && init_start)
            {

                ref_index = 0;
                ref_point = traj[ref_index];

                start_time = clock;
                start_ETSI_time = utility::getETSItimeFromUTCtimeMS(clock);
                first_mtsp_received =false;
                init_start = false;
                emit simulation_start(getID());
            }
            if (status == ABORT )
            {
                qDebug() << "ABORT: Aborting test scenario.";
                break;
            }
            qint64 time_since_HB = utility::getETSItimeFromUTCtimeMS(clock)
                    -last_received_heab_time_from_server;
            // Check if heartbeat deadline has passed
            if(time_since_HB > HEARTBEAT_TIME)
            {
                qDebug() << "ERROR: Heartbeat not recieved.";
                pendingStatus = ERROR;
                emit simulation_stop(getID());
            }
            if(ref_index < traj.size() - 1 )//&& elapsed_time < simulation_time)
            {
                elapsed_time = clock - start_time;
                qint64 deltaT = clock - time_since_update;
                if (double(deltaT) > 50.0 + 100.0*delay_simulation_factor)
                {
                    qDebug() << "Delay = " << QString::number(100*delay_simulation_factor);
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
                        //iClient->sendMonr(getMONR());
                    }
                    // Update the point
                    ref_index++;
                    ref_point = traj[ref_index];
                    time_since_update = clock;
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
            clock = utility::getCurrentUTCtimeMS();
            // Is this the first iteration?
            if (ref_index == 0 && init_start)
            {
                ref_index = 1;
                ref_point = traj[ref_index];
                prev_ref_point = traj[0];

                start_time = clock;
                start_ETSI_time = utility::getETSItimeFromUTCtimeMS(clock);
                first_mtsp_received =false;
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
                qint64 time_since_HB = utility::getETSItimeFromUTCtimeMS(clock)
                        -last_received_heab_time_from_server;
                // Check if heartbeat deadline has passed
                if(time_since_HB > HEARTBEAT_TIME && false)
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

        clock = utility::getCurrentUTCtimeMS();
        if (clock - update_sent > MONR_SEND_TIME_INTERVAL && sendMONREnabled && send_monr_idependently)
        {
            // Send monr
            iClient->sendMonr(getMONR());
            update_sent = clock;
        }

        // Send vizualizer update
        data.status = status;
        data.isMaster = isMaster;
        //data.mtsp = time_adjustment;
        emit updated_state(data);
        QThread::msleep(sleep_time);

    }
    qDebug() << "Done.";
    emit thread_done(data.ID);

}


int VirtualObject::connectToServer(int udpSocket,int tcpSocket)
{
    // Perhaps check if sockets are not taken?
    if (!iClient->startServer(udpSocket,tcpSocket)){
        return -1;
    }
    qDebug() << "Connected to Sockets:" << QString::number(udpSocket) << QString::number(tcpSocket);

    // Make connections

    connect(iClient,SIGNAL(osem_processed(osem)),
            this,SLOT(handleOSEM(osem)));
    connect(iClient,SIGNAL(ostm_processed(ostm)),
            this,SLOT(handleOSTM(ostm)));
    connect(iClient,SIGNAL(strt_processed(strt)),
            this,SLOT(handleSTRT(strt)));
    connect(iClient,SIGNAL(heab_processed(heab)),
            this,SLOT(handleHEAB(heab)));
    /*
    connect(iClient,SIGNAL(handle_osem(chronos_osem)),
            this,SLOT(handleOSEM(chronos_osem)));

    connect(iClient,SIGNAL(handle_dopm(QVector<chronos_dopm_pt>)),
            this,SLOT(handleDOPM(QVector<chronos_dopm_pt>)));
    connect(iClient,SIGNAL(handle_heab(chronos_heab)),
            this,SLOT(handleHEAB(chronos_heab)));
    connect(iClient,SIGNAL(handle_ostm(chronos_ostm)),
            this,SLOT(handleOSTM(chronos_ostm)));
    connect(iClient,SIGNAL(handle_strt(chronos_strt)),
            this,SLOT(handleSTRT(chronos_strt)));
    connect(iClient,SIGNAL(handle_sypm(chronos_sypm)),
            this,SLOT(handleSYPM(chronos_sypm)));
    connect(iClient,SIGNAL(handle_mtsp(chronos_mtsp)),
            this,SLOT(handleMTSP(chronos_mtsp)));
    connect(iClient,SIGNAL(handle_tcm(chronos_tcm)),
            this,SLOT(handleTCM(chronos_tcm)));*/


    return 0;
}

int VirtualObject::getID()
{
    return data.ID;
}

void VirtualObject::getRefLLH(double &lat, double &lon, double &alt)
{
    lat = mRefLat;
    lon = mRefLon;
    alt = mRefAlt;
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

monr VirtualObject::getMONR()
{
    monr msg;
    /*
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
    */
    msg.time_stamp = utility::getCurrentETSItimeMS();
    msg.x = (int32_t)(data.x * 1e3);
    msg.y = (int32_t)(data.y * 1e3);
    msg.z = (int32_t)(data.z * 1e3);
    msg.heading = (uint16_t)(data.heading * 1e1);
    msg.lon_speed = (int16_t)(data.speed*1e2);
    msg.lat_speed = 0;
    msg.lon_acc = (int16_t)(data.acc*1e1);
    msg.lat_acc = 0;
    msg.drive_direction = 0;
    switch (status) {
    case INIT:
        msg.object_state = ISO_OBJECT_STATE_INIT;
        msg.ready_to_arm = ISO_OBJECT_INTERNAL_STATE_NOT_READY_TO_ARM;
        break;
    case ARMED:
        msg.object_state = ISO_OBJECT_STATE_ARMED;
        msg.ready_to_arm = ISO_OBJECT_INTERNAL_STATE_READY_TO_ARM;
        break;
    case DISARMED:
        msg.object_state = ISO_OBJECT_STATE_DISARMED;
        msg.ready_to_arm = ISO_OBJECT_INTERNAL_STATE_READY_TO_ARM;
        break;
    case RUNNING:
    case RUNNING_STANDBY:
        msg.object_state = ISO_OBJECT_STATE_RUNNING;
        msg.ready_to_arm = ISO_OBJECT_INTERNAL_STATE_NOT_READY_TO_ARM;
        break;
    case STOP:
    case ABORT:
    case ERROR:
        msg.object_state = ISO_OBJECT_STATE_POST_RUN;
        msg.ready_to_arm = ISO_OBJECT_INTERNAL_STATE_NOT_READY_TO_ARM;
        break;
    default:
        break;
    }


    return msg;
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
    qDebug() << "End of trajectoy file.";
    return i;

}

// SLOTS

void VirtualObject::handleOSEM(osem msg)
{
    switch (status) {
    case INIT:
    case DISARMED:
        mRefLat = (double) msg.lat / 1e7;
        mRefLon = (double) msg.lon / 1e7;
        mRefAlt = (double) msg.alt /1e2;

        //utility::llhToXyz(msg.lat,msg.lon,msg.alt,&data.x,&data.y,&data.z);
        //mRefHeading = msg.heading;
        qDebug() << "OSEM:\n" << QString::number(mRefLat,'f',8) << "\n"
                 << QString::number(mRefLon,'f',8) << "\n"
                 << QString::number(mRefAlt,'f',8);

        hasOSEM = true;

        emit new_origin(mRefLat,mRefLon,mRefAlt);
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

void VirtualObject::handleLoadedDOPM(int ID, QVector<chronos_dopm_pt> msg)
{
    if (getID() != ID) return;
    handleDOPM(msg);
}

void VirtualObject::handleHEAB(heab msg)
{
    //qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    //qDebug() << "Time since last HB: " << QString::number(currentTime-last_received_heab_time_from_server);
    last_received_heab_time_from_server = msg.tx_time;

    switch (msg.cc_status) {
    case ISO_CC_STATUS_EMERGENCY_ABORT:
        pendingStatus = ABORT;
        qDebug() << "ABOTRING!";
        break;
    case ISO_CC_STATUS_OK:
        break;
    default:
        break;
    }
}

void VirtualObject::handleOSTM(ostm msg)
{

    qDebug() << "ARM-message!";
    switch (msg.state_change) {
    case ISO_OBJECT_STATE_ARMED:
        if ((status == INIT || status == DISARMED ||
                status == STOP || status == ABORT) &&
                hasDOPM && hasOSEM)
            // Maybe status == ERROR as well
        {
            qDebug() << "ARMING";
            pendingStatus = ARMED;
        }
        break;
    case ISO_OBJECT_STATE_DISARMED:
        if (status == STOP || status == ABORT ||
                status == ARMED || status == INIT)
            // Maybe status == ERROR as well
        {
            qDebug() << "DISARMING";
            pendingStatus = DISARMED;
        }
        break;
    default:
        break;
    }
}
void VirtualObject::handleSTRT(strt msg)
{
    if(status == ARMED)
    {
        start_ETSI_time = msg.abs_start_time;

        char buffer[30];
        utility::getDateTimeFromUTCtime(utility::getCurrentUTCtimeMS(),buffer,30);

        qDebug() << "Time at receiving: " << buffer;

        utility::getDateTimeFromETSItime(msg.abs_start_time,buffer,30);

        qDebug() << "Start time received: " << buffer;
        pendingStatus = RUNNING;//RUNNING_STANDBY;
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
            if (findRefPoint(msg.stop_point,0,0))
            {
                sync_stop_points.append(msg.stop_point);
                qDebug() << "Sync stop time = " << QString::number(msg.stop_point);
            }
        }
    }
}

void VirtualObject::handleMTSP(chronos_mtsp msg)
{
    qDebug() << "MTSP recieved";
    if (!first_mtsp_received)
    {
        first_mtsp = msg.ts;
        first_mtsp_received =true;
    }
    // Double-check what the status should be
    if (status == ARMED || status == RUNNING)
    {
        // Do something



        //qint64 temp = QDateTime::currentMSecsSinceEpoch()
        //        - MS_FROM_1970_TO_2004_NO_LEAP_SECS
        //        + DIFF_LEAP_SECONDS_UTC_ETSI*1000;

        time_adjustment = (qint64) (msg.ts - first_mtsp);
        qDebug() << "MTSP: " << QString::number(time_adjustment);
        emit forward_mtsp(getID(),data.time,time_adjustment);
        //data.mtsp = time_adjustment;

    }
}

void VirtualObject::handleTCM(chronos_tcm msg)
{
    (void)msg;
    if (status == INIT || status == ARMED || status == DISARMED)
    {
        isTCMReceived = true;
        TAA_ID = msg.trigger_id;
        TAA_trigger_type = msg.trigger_type;
        TAA_action = msg.action;
        TAA_delay = msg.trigger_delay;
        emit forward_tcm(getID(),isTCMReceived);
    }
}
void VirtualObject::triggerOccured(int ID)
{
    if (getID() == ID)
    {
        chronos_tom tom;
        tom.trigger_id = TAA_ID;
        tom.trigger_type = TAA_trigger_type;
        quint64 currentTime = QDateTime::currentMSecsSinceEpoch()-MS_FROM_1970_TO_2004_NO_LEAP_SECS + DIFF_LEAP_SECONDS_UTC_ETSI*1000;
        tom.trigger_etsi_time=currentTime + TAA_delay;
        //iClient->sendTOM(tom);
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

void VirtualObject::handleTrajSimDelayToggle(int ID, bool checked, double delayFactor)
{
    if (ID != getID()) return;

    traj_simulation_only = checked;
    delay_simulation_factor = delayFactor;
}

void VirtualObject::stopSimulation()
{
    shutdown = true;
}


