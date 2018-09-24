#include "osccondition.h"

OSCCondition::OSCCondition(QString name,OSCAction *action)
{
    this->m_name = name;
    this->m_action = action;
}
