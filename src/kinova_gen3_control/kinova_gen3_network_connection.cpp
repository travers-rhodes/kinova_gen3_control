#include <kinova_gen3_control/kinova_gen3_network_connection.h>

#define IP_ADDRESS "192.168.1.10"
#define PORT 10000
#define CYCLIC_PORT 10001

KinovaGen3NetworkConnection::KinovaGen3NetworkConnection()
{
  auto error_callback = [](Kinova::Api::KError err){ cout << "_________ callback error _________" << err.toString(); };
  transport_ = new Kinova::Api::TransportClientTcp();
  transport_cyclic_ = new Kinova::Api::TransportClientUdp();
  router_ = new Kinova::Api::RouterClient(transport_, error_callback);
  router_cyclic_ = new Kinova::Api::RouterClient(transport_cyclic_, error_callback);
  actuator_config = new Kinova::Api::ActuatorConfig::ActuatorConfigClient(router_);
  base = new Kinova::Api::Base::BaseClient(router_);
  base_cyclic = new Kinova::Api::BaseCyclic::BaseCyclicClient(router_cyclic_);
  session_manager_ = new Kinova::Api::SessionManager(router_);
  session_manager_cyclic_ = new Kinova::Api::SessionManager(router_cyclic_);


  transport_->connect(IP_ADDRESS, PORT);
  transport_cyclic_->connect(IP_ADDRESS, CYCLIC_PORT);
  // Set session data connection information
  auto create_session_info = Kinova::Api::Session::CreateSessionInfo();
  create_session_info.set_username("admin");
  create_session_info.set_password("admin");
  create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
  create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

  // Session manager service wrapper
  std::cout << "Creating session for communication" << std::endl;
  session_manager_->CreateSession(create_session_info);
  session_manager_cyclic_->CreateSession(create_session_info);
  std::cout << "Sessions created" << std::endl;
}

KinovaGen3NetworkConnection::~KinovaGen3NetworkConnection() 
{
  // Close API session
  session_manager_->CloseSession();
  session_manager_cyclic_->CloseSession();

  // Deactivate the router and cleanly disconnect from the transport object
  router_->SetActivationStatus(false);
  transport_->disconnect();
  router_cyclic_->SetActivationStatus(false);
  transport_cyclic_->disconnect();

  delete transport_;
  delete transport_cyclic_;
  delete router_;
  delete router_cyclic_;
  delete actuator_config;
  delete base;
  delete base_cyclic;
  delete session_manager_;
  delete session_manager_cyclic_;
}
