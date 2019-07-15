
#include <set>

#include "trigger.h"
#include "action.h"
#include "causality.h"

class Scenario
{
public:
    typedef enum {OK, NOT_OK, DUPLICATE_ELEMENT} ScenarioReturnCode_t;

    Scenario();

    ScenarioReturnCode_t linkTriggersWithActions(std::set<Trigger*> ts, std::set<Action*> as);
    ScenarioReturnCode_t linkTriggersWithAction(std::set<Trigger*> ts, Action* a);
    ScenarioReturnCode_t linkTriggerWithActions(Trigger* t, std::set<Action*> a);
    ScenarioReturnCode_t linkTriggerWithAction(Trigger* t, Action* a);

    ScenarioReturnCode_t addTrigger(Trigger &t);
    ScenarioReturnCode_t addAction(Action &a);

    Trigger* getTriggerByID(Trigger::TriggerID_t id);
    std::set<Causality> getCausalities(void);

    void updateTrigger(Trigger::TriggerID_t id);

    void refresh(void);
private:
    std::set<Causality> causalities;
    std::set<Trigger> allTriggers;
    std::set<Action> allActions;
};
