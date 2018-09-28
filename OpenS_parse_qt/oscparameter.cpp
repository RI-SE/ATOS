#include "oscparameter.h"

OSCParameter::OSCParameter(const QString &name, const QString &type, const QString &value)
{
    m_name = name;
    m_type = type;
    m_value = value;
}


OSCParameterType OSCParameter::getOSCParameterType()
{
    static QHash<QString, qint8> map = {
        {"double", TYPE_DOUBLE},
        {"integer", TYPE_INTEGER},
        {"string", TYPE_STRING}
    };
    return  map.value(m_type);
}
