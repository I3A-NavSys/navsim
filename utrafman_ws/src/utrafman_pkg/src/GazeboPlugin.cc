#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_ros/node.hpp>
#include "utrafman_msgs/srv/test.hpp"



namespace gazebo
{
  class MiPlugin : public WorldPlugin
  {
    private:
        event::ConnectionPtr updateConnection;



    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      gazebo_ros::Node::SharedPtr rosNode = gazebo_ros::Node::CreateWithArgs("mi_plugin_node", _world->Name());
      rclcpp::Service<utrafman_msgs::srv::Test>::SharedPtr      rosSrv_Test;

      
      // ROS2 UTRAFMAN services
      rosSrv_Test = rosNode->create_service<utrafman_msgs::srv::Test>(
          "utrafman/Test",
          std::bind(&MiPlugin::rosSrvFn_Test, this,
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));



     // Conectar el callback de actualización de Gazebo
      // this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      //     std::bind(&MiPlugin::OnUpdate, this));
      
      
      printf("Hello World!  LOAD\n");

      
    }

    void rosSrvFn_Test(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<utrafman_msgs::srv::Test::Request>  request,   
              std::shared_ptr<utrafman_msgs::srv::Test::Response> response)  
    {
        gzmsg << "ROS2 service called: UTRAFMAN Test" << std::endl;
        printf("LLAMADA AL SERVICIO\n");

        response->sum = request->a + request->b;
    }


    void Init()
    {
        printf("Hello World!  INIT\n");
    }

  



    // void OnUpdate()
    // {
    //   // Lógica de actualización de Gazebo
    //   printf("hola ");
    // }
  };
  GZ_REGISTER_WORLD_PLUGIN(MiPlugin)
} // namespace gazebo
