#ifndef ISOACTION_H
#define ISOACTION_H

#include "action.h"

#include <netinet/in.h>

class ISOAction : public Action
{
public:
    ISOAction();
    ActionReturnCode_t execute(void) override;
private:
    in_addr_t targetObjAddr = 0;
};

#endif
