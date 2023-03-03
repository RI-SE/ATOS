/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef ACTION_H
#define ACTION_H

#include <cstdint>
#include <iostream>
#include <vector>
#include <set>
#include <netinet/in.h>

#include "util.h"
#include "iso22133.h"
#include "roschannels/triggeractionchannels.hpp"
#include "loggable.hpp"

namespace ATOS {

    #define ACTION_NUMBER_PARAMETER_FIELDS 3

    class Action : public Loggable
    {
    public:
        /*! Typedefs */
        typedef ActionType_t ActionTypeCode_t;

        typedef ActionTypeParameter_t ActionParameter_t;

        typedef enum {
            OK,
            NOT_OK,
            INVALID_ARGUMENT,
            NO_REMAINING_RUNS
        } ActionReturnCode_t;

        typedef uint16_t ActionID_t;

        /*! Constructor */
        Action(rclcpp::Logger, ActionID_t actionID = 0, ActionTypeCode_t actionTypeCode = ACTION_NONE, uint32_t allowedNumberOfRuns = 1);
        
        /*! Destructor */
        virtual ~Action() { parameters.clear(); }

        /*! Getters */
        ActionID_t getID() const { return actionID; }
        ActionTypeCode_t getTypeCode() const { return actionTypeCode; }

        /*! Run the action once, if allowed */
        virtual ActionReturnCode_t execute(ROSChannels::ExecuteAction::Pub&);

        /*! Reset to start state */
        ActionReturnCode_t reset(void);

        /*! To string */
        friend std::ostream& operator<<(std::ostream &strm, const Action &act) {
            return strm << "ACTION ID " << act.actionID <<
                        " TYPE " << getTypeAsString(act.getTypeCode()) <<
                        " PARAMETERS " << act.getParametersString();
        }

        static std::string getTypeAsString(ActionTypeCode_t type);
        static std::string getParameterAsString(ActionParameter_t param);
        std::string getParametersString(void) const;
        ROSChannels::ActionConfiguration::message_type getConfigurationMessageData(void) const;
        in_addr_t getObjectIP(void) const { return actionObjectIP; }
        std::string getObjectIPAsString(void) const;
        void setObjectIP(in_addr_t ipAddr) { actionObjectIP = ipAddr; }

        void setExecuteDelayTime(struct timeval tm);
        struct timeval getExecuteDelayTime(void) const;

        ActionReturnCode_t appendParameter(ActionParameter_t actionParameter);
        virtual ActionReturnCode_t appendParameter(std::string parameterString);

        virtual ActionReturnCode_t parseParameters() = 0;

        static ActionTypeCode_t asTypeCode(const std::string &typeCodeString);

    protected:
        ActionTypeCode_t actionTypeCode = ACTION_NONE;
        ActionID_t actionID = 0;
        uint32_t remainingAllowedRuns = 0;
        uint32_t maxAllowedRuns = 0;
        std::vector<ActionParameter_t> parameters;
        uint32_t actionDelayTime_qms = 0;
        in_addr_t actionObjectIP = 0;

        ActionReturnCode_t checkActionParameter(ActionParameter_t actionParameter) const;
        virtual ActionParameter_t asParameterCode(const std::string &parameterCodeString) const;
        static char toUpper(const char c);

    private:
        virtual const std::set<ActionParameter_t> getAcceptedParameters(void) const
        {
            std::set<ActionParameter_t> accParams;
            accParams.insert(ACTION_PARAMETER_SET_FALSE);
            accParams.insert(ACTION_PARAMETER_SET_TRUE);
            accParams.insert(ACTION_PARAMETER_RELEASE);
            accParams.insert(ACTION_PARAMETER_PRESS);
            accParams.insert(ACTION_PARAMETER_SET_VALUE);
            accParams.insert(ACTION_PARAMETER_MIN);
            accParams.insert(ACTION_PARAMETER_MAX);
            accParams.insert(ACTION_PARAMETER_X);
            accParams.insert(ACTION_PARAMETER_Y);
            accParams.insert(ACTION_PARAMETER_Z);
            accParams.insert(ACTION_PARAMETER_VS_BRAKE_WARNING);
            accParams.insert(ACTION_PARAMETER_UNAVAILABLE);
            return accParams;
        }
    };
}
#endif
