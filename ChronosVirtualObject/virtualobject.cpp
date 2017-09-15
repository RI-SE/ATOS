#include "virtualobject.h"

VirtualObject::VirtualObject(qint8 id)
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

    this->id = id;

    updateTime();
}

VirtualObject::~VirtualObject() {}


void VirtualObject::run()
{
    qDebug() << "Virtual Object Started";

    qint64 elapsed_time = 0;
    qint64 run_time_MS = 10000;
    int flag = 1;

    updateTime();
    //clock = QDateTime::currentMSecsSinceEpoch();
    start_time = clock;

    while(elapsed_time < run_time_MS)
    {
        //clock = QDateTime::currentMSecsSinceEpoch();
        updateTime();
        elapsed_time = clock-start_time;
        if (run_time_MS/2 < elapsed_time && flag){
            qDebug() << "Reached half way.";
            flag = 0;
        }
        emit updated_position(0,0,(long) elapsed_time);

        QThread::msleep(10);
    }
    qDebug() << "Done.";


}

void VirtualObject::updateTime()
{
    clock = QDateTime::currentMSecsSinceEpoch();
}
