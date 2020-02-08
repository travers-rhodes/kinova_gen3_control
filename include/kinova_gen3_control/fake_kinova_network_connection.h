#ifndef FAKE_KINOVA_NETWORK_CONNECTION_H 
#define FAKE_KINOVA_NETWORK_CONNECTION_H 

#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>
#include <kinova_gen3_control/kinova_network_connection.h>
#include <ros/ros.h>

// This object, when constructed, creates fake network connections.
// This is a mock of a KinovaNetworkConnection
class FakeKinovaNetworkConnection : public KinovaNetworkConnection
{
  public:
    FakeKinovaNetworkConnection();
    ~FakeKinovaNetworkConnection();
    // you must construct these methods in your implementation
    virtual void BaseSetServoingMode(const Kinova::Api::Base::ServoingModeInformation& servoing_mode);
    virtual void ActuatorSetControlMode(const Kinova::Api::ActuatorConfig::ControlModeInformation& servoing_mode, int actuator_device_id);
    virtual void CyclicRefresh(const Kinova::Api::BaseCyclic::Command& base_comand);
    virtual Kinova::Api::BaseCyclic::Feedback CyclicRefreshFeedback();
};

#endif // FAKE_KINOVA_NETWORK_CONNECTION_H 
