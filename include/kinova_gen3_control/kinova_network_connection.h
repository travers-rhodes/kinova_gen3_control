#ifndef KINOVA_NETWORK_CONNECTION_H 
#define KINOVA_NETWORK_CONNECTION_H 

#include <TransportClientUdp.h>
#include <TransportClientTcp.h>
#include <RouterClient.h>
#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>

// This object, when constructed, creates network connections to the KinovaGen3.
// And deletes those network connections when this object is destructed.
// We wrap all these connection functions in their own class so we can easily mock these connections
// when we want to test our controller code without a robot.
class KinovaNetworkConnection
{
  public:
    virtual ~KinovaNetworkConnection() {};
    // you must construct these methods in your implementation
    virtual void BaseSetServoingMode(const Kinova::Api::Base::ServoingModeInformation& servoing_mode) = 0;
    virtual void ActuatorSetControlMode(const Kinova::Api::ActuatorConfig::ControlModeInformation& servoing_mode, int actuator_device_id) = 0;
    virtual void CyclicRefresh(const Kinova::Api::BaseCyclic::Command& base_comand) = 0;
    virtual Kinova::Api::BaseCyclic::Feedback CyclicRefreshFeedback() = 0;
};

#endif // KINOVA_NETWORK_CONNECTION_H 
