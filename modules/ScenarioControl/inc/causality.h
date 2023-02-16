/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef CAUSALITY_H
#define CAUSALITY_H

#include <set>
#include <iostream>

#include "trigger.h"
#include "action.h"
namespace ATOS {
    class Causality
    {
    public:
        typedef enum {OK, NOT_OK} CausalityReturnCode_t;

        typedef enum {OR, AND} TriggerOperator_t;

        Causality(Trigger* tp, Action* ap, TriggerOperator_t op = AND);
        Causality(Action* ap, TriggerOperator_t op = AND);
        Causality(TriggerOperator_t op = AND);

        void addTrigger(Trigger* tp) { triggers.insert(tp); }
        void removeTrigger(Trigger* tp) { triggers.erase(tp); }

        std::set<Trigger*> getTriggers(void) const { return triggers; }
        Trigger* getTriggerByID(Trigger::TriggerID_t id) const;

        void setOperator(TriggerOperator_t op) { oper = op; }
        void setRelationOR(void) { setOperator(OR); }
        void setRelationAND(void) { setOperator(AND); }

        TriggerOperator_t getOperator(void) const { return oper; }
        std::string getOperatorString(void) const;

        void executeIfActive(ROSChannels::ExecuteAction::Pub&) const;
        bool isActive(void) const;

        void addAction(Action* ap) { actions.insert(ap); }
        void removeAction(Action* ap) { actions.erase(ap); }

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
}
#endif
