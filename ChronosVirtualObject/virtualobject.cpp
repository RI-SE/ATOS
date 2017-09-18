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
    qint64 run_time_MS = 15000;
    int flag = 1;

    updateTime();
    //clock = QDateTime::currentMSecsSinceEpoch();
    start_time = clock;

    while(elapsed_time < run_time_MS)
    {
        //clock = QDateTime::currentMSecsSinceEpoch();
        updateTime();
        elapsed_time = clock-start_time;
        data.time = elapsed_time;
        if (run_time_MS/2 < elapsed_time && flag){
            qDebug() << "Reached half way.";
            flag = 0;
        }
        if (flag){
            data.x=0; data.y=0;
            //emit updated_state(this->id,(qint32) elapsed_time,x,y);

        }
        else
        {
            data.x=5; data.y=5;
            //emit updated_state(this->id,(qint32) elapsed_time,x,y);

        }
        emit updated_state(data);
        QThread::msleep(10);
    }
    qDebug() << "Done.";


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

// SLOTS

void VirtualObject::handleOSEM(chronos_osem msg)
{
    data.mRefLat = msg.lat;
    data.mRefLon = msg.lon;
    data.mRefAlt = msg.alt;
    mRefHeading = msg.heading;
    emit updated_state(data);
}

void VirtualObject::handleDOPM(chronos_dopm_pt msg)
{

}




