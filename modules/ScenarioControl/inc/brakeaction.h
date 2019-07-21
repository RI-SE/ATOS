#include "action.h"

#include <netinet/in.h>

class BrakeAction : public Action
{
public:
    BrakeAction();
    ActionReturnCode_t execute(void) override;
private:
    in_addr_t brakeObjAddr = 0;
};
