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
        if (flag){
            x=0; y=0;
            emit updated_state(this->id,(qint32) elapsed_time,x,y);
        }
        else
        {
            x=5; y=5;
            emit updated_state(this->id,(qint32) elapsed_time,x,y);
        }
        QThread::msleep(10);
    }
    qDebug() << "Done.";


}

LocPoint VirtualObject::getCurrentState()
{
    LocPoint ret_val;
    ret_val.setTime(clock-start_time);
    ret_val.setXY(x,y);
    return ret_val;
}

void VirtualObject::updateTime()
{
    clock = QDateTime::currentMSecsSinceEpoch();
}
