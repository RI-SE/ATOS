/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef SCENARIO_H
#define SCENARIO_H

#include <set>
#include <fstream>

#include "trigger.h"
#include "action.h"
#include "causality.h"
#include "journal.hpp"
#include "roschannel.hpp"

#include "ad-xolib/xodr.h"
#include "ad-xolib/xosc.h"

#include "loggable.hpp"

namespace ATOS {
    class Scenario : public Loggable
    {
    public:
        typedef enum {OK, NOT_OK, DUPLICATE_ELEMENT, INVALID_ARGUMENT, NOT_FOUND} ScenarioReturnCode_t;

        Scenario(const std::string scenarioFilePath, 
                 const std::string openDriveFilePath, 
                 const std::string openScenarioFilePath,  
                 rclcpp::Logger log);
        ~Scenario();

        void initialize(const std::string scenarioFilePath, 
                        const std::string openDriveFilePath, 
                        const std::string openScenarioFilePath);
        void loadOpenDrive(const std::string openDriveFilePath);
        void loadOpenScenario(const std::string openScenarioFilePath);

        ScenarioReturnCode_t linkTriggersWithActions(std::set<Trigger*> tps, std::set<Action*> aps);
        ScenarioReturnCode_t linkTriggersWithAction(std::set<Trigger*> tps, Action* ap);
        ScenarioReturnCode_t linkTriggerWithActions(Trigger* tp, std::set<Action*> aps);
        ScenarioReturnCode_t linkTriggerWithAction(Trigger* tp, Action* ap);

        ScenarioReturnCode_t addTrigger(Trigger* tp);
        ScenarioReturnCode_t addAction(Action* ap);

        std::shared_ptr<std::set<Causality>> getCausalities();

        template<typename T>
        ScenarioReturnCode_t updateTrigger(Trigger::TriggerID_t id, T value)
        {
            Trigger::TriggerReturnCode_t retval;
            for (Trigger* tp : allTriggers)
            {
                if (tp->getID() == id)
                {
                    retval = tp->update(value);
                    if (retval == Trigger::TRIGGER_OCCURRED) {
                        std::string type = Trigger::getTypeAsString(tp->getTypeCode());
                        RCLCPP_DEBUG(get_logger(), "Triggered ID %u", id);
                        JournalRecordData(JOURNAL_RECORD_EVENT, "Trigger %s (ID %u) occurred", type.c_str(), id);
                    }
                    return (retval == Trigger::NOT_OK) ? INVALID_ARGUMENT : OK;
                }
            }
            return NOT_FOUND;
        }

        void resetISOTriggers(void);
        void executeTriggeredActions(ROSChannels::ExecuteAction::Pub&) const;
        void reset(void);
        void clear(void);

        ScenarioReturnCode_t updateTrigger(const ObjectDataType&);

    private:
        std::shared_ptr<OpenDRIVE> openDriveObject;
        std::shared_ptr<OpenSCENARIO> openScenarioObject;
        std::shared_ptr<std::set<Causality>> causalities;
        std::set<Trigger*> allTriggers;
        std::set<Action*> allActions;

        void parseScenarioFile(std::ifstream &file);
        void parseScenarioFileLine(const std::string &line);
        void splitLine(const std::string &line, const char delimiter, std::vector<std::string> &result);
        std::set<Trigger*> parseTriggerConfiguration(const std::string &config);
        std::set<Action*> parseActionConfiguration(const std::string &config);

    };
}
#endif
