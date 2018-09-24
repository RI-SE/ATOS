#include "oscactor.h"

OSCActor::OSCActor(QString name)
{
    this->m_name = name;
}

OSCActor::OSCActor(const OSCActor & other)
{
    this->m_name = other.m_name;
}

OSCActor::~OSCActor() {}

void OSCActor::printobject()
{
    qDebug() << "OSCActor(" << this->getName() << ")";
}

bool OSCActor::isSame(QString other)
{
    return m_name.compare(other) == 0;
}
