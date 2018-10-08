#include "oscparameter.h"

OSCParameter::OSCParameter(const QString &name, const QString &type, const QString &value)
{
    m_name = name;
    m_type = type;
    m_value = value;
}

OSCParameter::OSCParameter(const OSCParameter &other)
{
    this->m_name = other.m_name;
    this->m_type = other.m_type;
    this->m_value = other.m_value;
}

OSCParameter::~OSCParameter(){}

OSCParameterType OSCParameter::getOSCParameterType()
{
    static QHash<QString, qint8> map = {
        {"double", TYPE_DOUBLE},
        {"integer", TYPE_INTEGER},
        {"string", TYPE_STRING}
    };
    return  map.value(m_type);
}

void OSCParameter::printobject()
{
    qDebug() << "OSCParameter( " << m_name << "," << m_type<< "," << m_value <<" )";

}
