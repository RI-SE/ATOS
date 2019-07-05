#ifndef ACTION_H
#define ACTION_H

#include <cstdint>

class Action
{
public:
    typedef enum {
        NO_ACTION,
        ACTION_TYPE_1,
        ACTION_TYPE_2,
        ACTION_SET_SPEED,
        ACTION_SET_DISTANCE,
        ACTION_SET_ACCELERATION,
        ACTION_LANE_CHANGE,
        ACTION_LANE_OFFSET,
        ACTION_SET_POSITION

                 } ActionType_t;
    typedef uint16_t ActionID_t;
    Action(ActionID_t actionID);

    virtual Action

private:
    ActionID_t actionID;
};

#endif
