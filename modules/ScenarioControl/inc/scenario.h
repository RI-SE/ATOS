
#include <list>

#include "trigger.h"
#include "action.h"
#include "causality.h"

class Scenario
{
public:
    typedef enum {OK, NOT_OK} ScenarioReturnCode_t;

    Scenario();

    ScenarioReturnCode_t addCausality(std::set<Trigger> ts, Action a);
    ScenarioReturnCode_t addCausality(Trigger t, Action a);
    ScenarioReturnCode_t addCausality(Causality c);
private:
    std::list<Causality> causalities;
};
