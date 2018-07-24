#include "virtualobject.h"

VirtualObject::VirtualObject(int id,double rLat,double rLon,double rAlt)
{

    data.ID = id;
    data.time = 0;
    data.x = 0.0;
    data.y = 0.0;
    data.z = 0.0;
    data.heading = 0.0;
    data.speed = 0.0;
    data.acc = 0.0;
    data.status = status;
    data.isMaster = isMaster;



    mRefLat = rLat;
    mRefLon = rLon;
    mRefAlt = rAlt;

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

    chronos_dopm_pt ref_point;      // Placeholder for the reference point

    while (!shutdown){
        // Loop as long as a shutdown signal has not been sent

        status = pendingStatus;

        switch (status) {
        case INIT:
            initObjectState();
            pendingStatus = DISARMED;
            break;
        case DISARMED:
            break;
        case ARMED:
            break;
        case RUNNING:

            sleep_time = 5;
            timedata.clock = utility::getCurrentUTCtimeMS();
            timedata.elapsed_time = timedata.clock - timedata.test_start_time;

            // Check if heartbeat deadline has passed
            if(utility::getCurrentUTCtimeMS()-timedata.HEAB_rec_time > HEARTBEAT_TIME && false)
            {
                qDebug() << "ERROR: Heartbeat not recieved.";
                pendingStatus = POSTRUN;
                emit simulation_stop(getID());
            }
            if(reference_point_index < traj.size() - 1 )
            {
                // Update the reference point
                ref_point = traj[reference_point_index];

                if (ref_point.tRel < timedata.elapsed_time)
                {
                    qDebug() << "EST_TIME:" << timedata.elapsed_time;
                    //qDebug() << "Delay = " << QString::number(100*delay_simulation_factor);
                    // Update the state
                    data.acc = ref_point.accel;
                    data.heading = ref_point.heading;
                    data.speed = ref_point.speed;
                    //data.time = timedata.elapsed_time;
                    data.x = ref_point.x;
                    data.y = ref_point.y;
                    data.z = ref_point.z;


                    // Update the point
                    reference_point_index++;

                }
            }
            else
            {
                pendingStatus = POSTRUN;
                emit simulation_stop(getID());
            }


            break;
        case POSTRUN:
            pendingStatus = DISARMED;
            break;
        case REMOTECTRL:
            break;
        default:
            break;
        }


        timedata.clock = utility::getCurrentUTCtimeMS();
        if (timedata.clock - timedata.monr_send_time > MONR_SEND_TIME_INTERVAL)
        {
            if (sendMONREnabled){
                // Send monr
                iClient->sendMonr(getMONR());
            }

            timedata.monr_send_time = timedata.clock;
        }

        // Send vizualizer update
        data.status = status;
        data.isMaster = isMaster;
        data.time = timedata.elapsed_time;
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

bool VirtualObject::initObjectState()
{
    if (status == RUNNING) return false;

    reference_point_index = 0;

    // Init time struct
    timedata.test_start_time = 0;
    timedata.elapsed_time = 0;
    timedata.HEAB_rec_time = 0;
    timedata.ctrl_update_time = 0;

    if(hasDOPM)
    {
        data.x = traj[0].x;
        data.y = traj[0].y;
        data.z = traj[0].z;
        data.acc = 0;
        data.speed  = 0;
        data.heading = traj[0].heading;
    }
    else
    {
        data.x = 0;
        data.y = 0;
        data.z = 0;
        data.acc = 0;
        data.speed  = 0;
        data.heading = 0;
    }

    return true;
}

void VirtualObject::control_object(chronos_dopm_pt next,chronos_dopm_pt prev)
{
    // Find the change in both directions
    double deltaY = next.y-prev.y;
    double deltaX = next.x-prev.x;

    // Calculate the constant velocity between the two reference points
    double new_vx = deltaX / static_cast<double> (next.tRel-prev.tRel) ;
    double new_vy = deltaY / static_cast<double> (next.tRel-prev.tRel) ;

    // Calculate the length of the velocity vector
    double actual_speed = sqrt(new_vx*new_vx+new_vy*new_vy);

    // Update the current position based on the time that has
    // passed since the last reference point
    data.x = prev.x + new_vx * static_cast<double>(data.time-prev.tRel);
    data.y = prev.y + new_vy * static_cast<double>(data.time-prev.tRel);

    // Update the current state variables
    data.acc = (actual_speed - data.speed)/(data.time-prev.tRel); //prev.accel;
    data.heading = prev.heading;
    data.speed = actual_speed;
}

monr VirtualObject::getMONR()
{
    monr msg;

    msg.time_stamp = utility::getCurrentETSItimeMS();
    msg.x = static_cast<int32_t>(data.x * 1e3);
    msg.y = static_cast<int32_t>(data.y * 1e3);
    msg.z = static_cast<int32_t>(data.z * 1e3);
    msg.heading = static_cast<uint16_t>(data.heading  * 1e2);
    msg.lon_speed = static_cast<int16_t>(data.speed*1e2);
    msg.lat_speed = 0;
    msg.lon_acc = static_cast<int16_t>(data.acc*1e1);
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

    default:
        break;
    }

    //printf("<Time:%ld;X:%f;Y:%f;Z:%f;H:%f;V:%f;>\n",msg.time_stamp,data.x,data.y,data.z,data.heading,data.speed);
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

    if (traj.size() <= static_cast<int>(fromIndex) )
    {
        qDebug() << "Index out of bounds";
    }
    int i=static_cast<int>(fromIndex);
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
        mRefLat = static_cast<double>(msg.lat / 1e10);
        mRefLon = static_cast<double>(msg.lon / 1e10);
        mRefAlt = static_cast<double>(msg.alt /1e2);

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
            //data.speed = firstPos.speed;
            //data.acc = firstPos.accel;

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


    //last_received_heab_time_from_server = msg.tx_time;

    // TODO: make use of the time sent from the server

    timedata.HEAB_rec_time = utility::getCurrentUTCtimeMS();

    switch (msg.cc_status) {
    case ISO_CC_STATUS_EMERGENCY_ABORT:
        pendingStatus = POSTRUN;
        qDebug() << "ABORT RECEIVED";
        break;
    case ISO_CC_STATUS_OK:
        break;
    default:
        break;
    }
}

void VirtualObject::handleOSTM(ostm msg)
{

    qDebug() << "OSTM received";
    switch (msg.state_change) {
    case ISO_OBJECT_STATE_ARMED:
        if ((status == DISARMED ) &&
                hasDOPM && hasOSEM)
            // Maybe status == ERROR as well
        {
            qDebug() << "ARMING";
            initObjectState();
            pendingStatus = ARMED;
        }
        break;
    case ISO_OBJECT_STATE_DISARMED:
        if (status == POSTRUN)
            // Maybe status == ERROR as well
        {
            qDebug() << "DISARMING";
            pendingStatus = DISARMED;
        }
        break;
    default:
        qDebug() << "Unable to handle OSTM.";
        break;
    }
}
void VirtualObject::handleSTRT(strt msg)
{
    if(status == ARMED)
    {
        //start_ETSI_time = msg.GPSsecOfWeek_start_time;

        timedata.test_start_time = utility::getCurrentUTCtimeMS();

        char buffer[30];
        utility::getDateTimeFromUTCtime(utility::getCurrentUTCtimeMS(),buffer,30);

        qDebug() << "Time at receiving: " << buffer;

        //utility::getDateTimeFromETSItime(msg.abs_start_time,buffer,30);

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


