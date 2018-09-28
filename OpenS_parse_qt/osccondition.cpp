#include "osccondition.h"

OSCCondition::OSCCondition(QString name,OSCPrivateAction *action)
{
    this->m_name = name;
    this->m_action = action;
}
