#ifndef MSCPITERPRETER_H
#define MSCPITERPRETER_H
#include <QDebug>
#include <QByteArray>


namespace MSCP {
    void readServerResponse();

    bool msgToHTTPPOSTByteArray(const QString &IPaddress,const QString &msg, QByteArray &bytearray);
}


#endif // MSCPITERPRETER_H
