#include <set>

#include "trigger.h"
#include "action.h"

class Causality
{
public:
    typedef enum {OK, NOT_OK} CausalityReturnCode_t;

    typedef enum {OR, AND} TriggerOperator_t;

    Causality(Trigger t, Action a, TriggerOperator_t op = OR);
    Causality(Action a, TriggerOperator_t op = OR);
    Causality(TriggerOperator_t op = OR);

    void addTrigger(Trigger t) { triggers.insert(t); }
    void removeTrigger(Trigger t) { triggers.erase(t); }

    std::set<Trigger> getTriggers(void) { return triggers; }

    void setOperator(TriggerOperator_t op) { oper = op; }
    void setRelationOR(void) { setOperator(OR); }
    void setRelationAND(void) { setOperator(AND); }

    TriggerOperator_t getOperator(void) { return oper; }
    std::string getOperatorString(void);

    void setAction(Action a) { action = a; }
    Action getAction(void) { return action; }

private:
    TriggerOperator_t oper;
    std::set<Trigger> triggers;
    Action action;
};
