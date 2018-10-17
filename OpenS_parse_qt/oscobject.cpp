#include "oscobject.h"

OSCObject::OSCObject(QString name)
{
    this->m_name = name;
}

OSCObject::OSCObject(const OSCObject & other)
{
    this->m_name = other.m_name;
}

OSCObject::~OSCObject() {}

void OSCObject::printobject()
{
    qDebug() << "OSCObject(" << this->getName() << ")";
}

bool OSCObject::isSame(QString other)
{
    return m_name.compare(other) == 0;
}
