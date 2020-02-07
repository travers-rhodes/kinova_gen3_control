#include <TransportClientUdp.h>
#include <TransportClientTcp.h>
#include <RouterClient.h>
#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>
#include <kinova_gen3_control/kinova_network_connection.h>

// This object, when constructed, creates useable network connections to the KinovaGen3.
// And deletes those network connections when this object is destructed.
class KinovaGen3NetworkConnection : public KinovaNetworkConnection
{
  public:
    KinovaGen3NetworkConnection();
    ~KinovaGen3NetworkConnection();
    
    void BaseSetServoingMode(const Kinova::Api::Base::ServoingModeInformation& servoing_mode);
    void ActuatorSetControlMode(const Kinova::Api::ActuatorConfig::ControlModeInformation& control_mode, int actuator_device_id);
    void CyclicRefresh(const Kinova::Api::BaseCyclic::Command& base_comand);
    Kinova::Api::BaseCyclic::Feedback CyclicRefreshFeedback();

  private:
    Kinova::Api::ActuatorConfig::ActuatorConfigClient *actuator_config_;
    Kinova::Api::Base::BaseClient *base_;
    Kinova::Api::BaseCyclic::BaseCyclicClient *base_cyclic_;

    Kinova::Api::TransportClientTcp *transport_;
    Kinova::Api::TransportClientUdp *transport_cyclic_;
    Kinova::Api::RouterClient *router_;
    Kinova::Api::RouterClient *router_cyclic_;
    Kinova::Api::SessionManager *session_manager_;
    Kinova::Api::SessionManager *session_manager_cyclic_;
};
