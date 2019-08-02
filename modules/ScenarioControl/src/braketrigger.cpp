#include <sys/time.h>
#include "maestroTime.h"
#include "braketrigger.h"

#define ACCELERATION_OF_GRAVITY_M_S2 9.81
#define DEFAULT_RETARDATION_THRESHOLD_G 0.1

BrakeTrigger::BrakeTrigger(Trigger::TriggerID_t triggerID) : BooleanTrigger(triggerID, Trigger::TriggerTypeCode_t::TRIGGER_BRAKE)
{
    setBrakeRetardationThreshold(DEFAULT_RETARDATION_THRESHOLD_G * ACCELERATION_OF_GRAVITY_M_S2);
}

void BrakeTrigger::setBrakeRetardationThreshold(double threshold_m_s2)
{
    brakeRetardationThreshold_m_s2 = fabs(threshold_m_s2);
}

BrakeTrigger::TriggerReturnCode_t BrakeTrigger::update(double velocityMeasurement, struct timeval measurementTime)
{
    static struct timeval lastMeasurementTime;
    struct timeval timeDifference;
    double deltaT, velocityInnovation, accelerationInnovation;
    static double velocityEstimate, accelerationEstimate;
    constexpr double velocityInnovationWeight = 0.85;
    constexpr double accelerationInnovationWeight = 0.4;

    if (!timerisset(&lastMeasurementTime))
    {
        // Initialize
        velocityEstimate = velocityMeasurement;
        accelerationEstimate = 0;
        return OK;
    }

    // Calculate time difference and save current time
    timersub(&measurementTime, &lastMeasurementTime, &timeDifference);
    deltaT = timeDifference.tv_sec + timeDifference.tv_usec * 1000000;
    lastMeasurementTime = measurementTime;

    // Prediction step
    velocityEstimate = velocityEstimate + accelerationEstimate * deltaT;
    accelerationEstimate = accelerationEstimate;

    // Update step
    accelerationInnovation = (velocityMeasurement - velocityEstimate) / deltaT;
    velocityInnovation = (velocityMeasurement - velocityEstimate);
    accelerationEstimate = accelerationEstimate + accelerationInnovationWeight * accelerationInnovation;
    velocityEstimate = velocityEstimate + velocityInnovationWeight * velocityInnovation;

    // Check for negative acceleration
    return update(accelerationEstimate < -brakeRetardationThreshold_m_s2, measurementTime);
}

/*!
 * \brief BooleanTrigger::parseParameters Parses the parameter vector and sets the trigger mode accordingly
 * \return Value according to ::TriggerReturnCode_t
 */
Trigger::TriggerReturnCode_t BrakeTrigger::parseParameters()
{
    if (parameters.size() == 1)
    {
        switch (parameters.front())
        {
        case TRIGGER_PARAMETER_PRESSED:
        case TRIGGER_PARAMETER_TRUE:
        case TRIGGER_PARAMETER_HIGH:
            mode = HIGH;
            isStateTrue = false;
            wasStateTrue = false;
            return OK;
        case TRIGGER_PARAMETER_RELEASED:
        case TRIGGER_PARAMETER_FALSE:
        case TRIGGER_PARAMETER_LOW:
            mode = LOW;
            isStateTrue = true;
            wasStateTrue = true;
            return OK;
        case TRIGGER_PARAMETER_RISING_EDGE:
            mode = EDGE_RISING;
            isStateTrue = false;
            wasStateTrue = false;
            return OK;
        case TRIGGER_PARAMETER_FALLING_EDGE:
            mode = EDGE_FALLING;
            isStateTrue = false;
            wasStateTrue = false;
            return OK;
        case TRIGGER_PARAMETER_ANY_EDGE:
            mode = EDGE_ANY;
            isStateTrue = false;
            wasStateTrue = false;
            return OK;
        default:
            return INVALID_ARGUMENT;
        }
    }
    else return INVALID_ARGUMENT;
}


