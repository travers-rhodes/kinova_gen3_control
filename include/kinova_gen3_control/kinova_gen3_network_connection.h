#include <TransportClientUdp.h>
#include <TransportClientTcp.h>
#include <RouterClient.h>
#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>

// This object, when constructed, creates useable network connections to the KinovaGen3.
// And deletes those network connections when this object is destructed.
class KinovaGen3NetworkConnection
{
  public:
    KinovaGen3NetworkConnection();
    ~KinovaGen3NetworkConnection();

    Kinova::Api::ActuatorConfig::ActuatorConfigClient *actuator_config;
    Kinova::Api::Base::BaseClient *base;
    Kinova::Api::BaseCyclic::BaseCyclicClient *base_cyclic;

  private:
    Kinova::Api::TransportClientTcp *transport_;
    Kinova::Api::TransportClientUdp *transport_cyclic_;
    Kinova::Api::RouterClient *router_;
    Kinova::Api::RouterClient *router_cyclic_;
    Kinova::Api::SessionManager *session_manager_;
    Kinova::Api::SessionManager *session_manager_cyclic_;
};
