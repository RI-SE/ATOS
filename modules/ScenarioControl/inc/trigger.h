#ifndef TRIGGER_H
#define TRIGGER_H

#include <cstdint>
#include <stdexcept>
#include <set>
#include <iostream>
#include <vector>
#include <netinet/in.h>

#include "util.h"
#include "iso22133.h"
#include "maestro_interfaces/msg/trigger_event.hpp"

using maestro_interfaces::msg::TriggerEvent;

class Trigger
{
public:
    /*! Typedefs */
    typedef TriggerType_t TriggerTypeCode_t;

    typedef TriggerTypeParameter_t TriggerParameter_t;

    typedef uint16_t TriggerID_t;

    typedef enum {
        OK,
        NOT_OK,
        INVALID_ARGUMENT,
        TRIGGER_OCCURRED,
        NO_TRIGGER_OCCURRED
    } TriggerReturnCode_t;


    /*! Constructor */
    Trigger(TriggerID_t triggerID, TriggerTypeCode_t triggerType);


    /*! Destructor */
    virtual ~Trigger();


    /*! Getters */
    virtual TriggerTypeCode_t getTypeCode() const { return triggerTypeCode; }
    TriggerID_t getID() const { return triggerID; }
    std::vector<TriggerParameter_t> getParameters() const { return parameters; }
    bool isActive() const;
    in_addr_t getObjectIP(void) const { return triggerObjectIP; }
	std::string getObjectIPAsString(void) const;

    bool operator==(const Trigger &other) const { return (other.triggerID == triggerID) && isSimilar(other); }
    bool isSimilar(const Trigger &other) const;

    /*! Setters */
    void setID(TriggerID_t triggerID) { this->triggerID = triggerID; }
    void setObjectIP(in_addr_t ipAddr) { triggerObjectIP = ipAddr; }

    /*!
     * \brief appendParameter Appends an ISO parameter to the parameters list.
     * \param triggerParameter Parameter to append
     * \return Value according to ::TriggerReturnCode_t
     */
    TriggerReturnCode_t appendParameter(TriggerParameter_t triggerParameter);
    virtual TriggerReturnCode_t appendParameter(std::string parameterString);

    /*!
     * \brief parseParameters Parse the parameters list into an appropriate Trigger mode.
     * \return Value according to ::TriggerReturnCode_t
     */
    virtual TriggerReturnCode_t parseParameters() = 0;


    /*! To string */
    friend std::ostream& operator<<(std::ostream &strm, const Trigger &trig) {
        return strm << "TRIGGER ID " << trig.triggerID <<
                       " TYPE " << getTypeAsString(trig.getTypeCode()) <<
                       " PARAMETERS " << trig.getParametersString();
    }

    static std::string getTypeAsString(TriggerTypeCode_t typeCode);
    static std::string getParameterAsString(TriggerParameter_t param);
    std::string getParametersString(void) const;
    TRCMData getConfigurationMessageData(void) const;

    /*!
     * \brief update Update tracked signal (i.e. signal which causes the trigger to occur).
     * Inheriting classes should override the appropriate function(s)
     * - e.g. a trigger tracking a floating point trigger should override
     * update(float) and update(double)
    */
    virtual TriggerReturnCode_t update(void)    { throw std::invalid_argument("Invalid signal type"); }
    virtual TriggerReturnCode_t update(struct timeval)    { throw std::invalid_argument("Invalid signal type"); }
	virtual TriggerReturnCode_t update(bool, struct timeval)    { throw std::invalid_argument("Invalid signal type bool"); }
	virtual TriggerReturnCode_t update(char, struct timeval)    { throw std::invalid_argument("Invalid signal type char"); }
	virtual TriggerReturnCode_t update(int, struct timeval)     { throw std::invalid_argument("Invalid signal type int"); }
	virtual TriggerReturnCode_t update(float, struct timeval)   { throw std::invalid_argument("Invalid signal type float"); }
	virtual TriggerReturnCode_t update(double, struct timeval)  { throw std::invalid_argument("Invalid signal type double"); }
	virtual TriggerReturnCode_t update(CartesianPosition, struct timeval) { throw std::invalid_argument("Invalid signal type cartesian position"); }
	virtual TriggerReturnCode_t update(TriggerEvent::SharedPtr) { throw std::invalid_argument("Invalid signal type TREO data"); }
	virtual TriggerReturnCode_t update(ObjectDataType) { throw std::invalid_argument("Invalid signal type monitor data"); }


    static TriggerTypeCode_t asTypeCode(std::string typeCodeString);
protected:
    TriggerReturnCode_t checkTriggerParameter(TriggerParameter_t triggerParameter) const;
    TriggerTypeCode_t triggerTypeCode;
    TriggerReturnCode_t wasTriggeredByLastUpdate = NOT_OK; //!< State saving the last result of update
    std::vector<TriggerParameter_t> parameters;
    virtual TriggerParameter_t asParameterCode(const std::string &parameterCodeString) const;
    static char toUpper(const char c);
    in_addr_t triggerObjectIP = 0;

private:
    TriggerID_t triggerID;

    virtual const std::set<TriggerParameter_t> getAcceptedParameters() const
    {
        std::set<TriggerParameter_t> accParams;
        accParams.insert(TRIGGER_PARAMETER_UNAVAILABLE);
        return accParams;
    }

    virtual TriggerReturnCode_t checkIfTriggered(void) const = 0;
};

#endif // TRIGGER_H
