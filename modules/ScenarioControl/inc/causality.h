#include <set>
#include <iostream>

#include "trigger.h"
#include "action.h"

class Causality
{
public:
    typedef enum {OK, NOT_OK} CausalityReturnCode_t;

    typedef enum {OR, AND} TriggerOperator_t;

    Causality(Trigger* t, Action* a, TriggerOperator_t op = AND);
    Causality(Action* a, TriggerOperator_t op = AND);
    Causality(TriggerOperator_t op = AND);

    void addTrigger(Trigger* t) { triggers.insert(t); }
    void removeTrigger(Trigger* t) { triggers.erase(t); }

    std::set<Trigger*> getTriggers(void) const { return triggers; }
    Trigger* getTriggerByID(Trigger::TriggerID_t id) const;

    void setOperator(TriggerOperator_t op) { oper = op; }
    void setRelationOR(void) { setOperator(OR); }
    void setRelationAND(void) { setOperator(AND); }

    TriggerOperator_t getOperator(void) const { return oper; }
    std::string getOperatorString(void) const;

    void refresh(void) const;
    bool isActive(void) const;

    void addAction(Action* a) { actions.insert(a); }
    void removeAction(Action* a) { actions.erase(a); }

    std::set<Action*> getActions(void) const { return actions; }
    Action* getActionByID(Action::ActionID_t id) const;

    bool operator==(const Causality &other) const;
    bool operator<(const Causality &other) const;

    /*! To string */
    friend std::ostream& operator<<(std::ostream &strm, const Causality &causality) {
        return strm << ((causality.getOperator() == OR) ? "OR" : "AND") <<
                       "-CAUSALITY LINKING " << causality.getTriggers().size() <<
                       " TRIGGER" << (causality.getTriggers().size() == 1 ? "" : "S") <<
                       " TO " << causality.getActions().size() <<
                       " ACTION" << (causality.getActions().size() == 1 ? "" : "S");
    }

private:
    TriggerOperator_t oper;
    std::set<Trigger*> triggers;
    std::set<Action*> actions;
};
