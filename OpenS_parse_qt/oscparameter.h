#pragma once
#ifndef OSCPARAMETER_H
#define OSCPARAMETER_H

#include <QObject>
#include "osclib.h"


typedef qint8 OSCParameterType;

class OSCParameter : public OSCType
{
public:



    enum ParameterType {
        TYPE_INTEGER,
        TYPE_STRING,
        TYPE_DOUBLE
    };


    OSCParameter(const QString &name = "",
                            const QString &type = "",
                            const QString &value = "");

    OSCParameter(const OSCParameter &other);
    ~OSCParameter();

    OSCParameterType getOSCParameterType();

    void printobject();


private:
    QString m_name;
    QString m_type;
    QString m_value;
};

#endif // OSCPARAMETER_H
