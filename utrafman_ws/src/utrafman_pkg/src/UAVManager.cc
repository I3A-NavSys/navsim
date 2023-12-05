#include "rclcpp/rclcpp.hpp"
// #include <boost/format.hpp>
// #include <ignition/math/Pose3.hh>

#include "gazebo/gazebo.hh"
#include <gazebo_ros/node.hpp>
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
//#include "gazebo_msgs/srv/delete_entity.hpp"

// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include "std_msgs/Bool.h"
// #include "ros/callback_queue.h"
// #include "ros/subscribe_options.h"

#include "utrafman_msgs/srv/test.hpp"
#include "utrafman_msgs/srv/deploy_uav.hpp"
// #include "utrafman/remove_model.h"
// #include "utrafman/teletransport.h"


namespace gazebo
{
    class UAVManager : public WorldPlugin
    {
        private:

            int iteration;

            // Gazebo
            physics::WorldPtr    world;
            event::ConnectionPtr updateConnector;

            // ROS2 Node
            rclcpp::Node::SharedPtr      rosNode;
            
            // ROS2 UTRAFMAN services
            rclcpp::Service<utrafman_msgs::srv::Test>::SharedPtr      rosSrv_Test;
            rclcpp::Service<utrafman_msgs::srv::DeployUAV>::SharedPtr rosSrv_DeployUAV;

        public:

            void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
            {
                // gzmsg << "UTRAFMAN UAVManager plugin: loading" << std::endl;
                // printf("UTRAFMAN UAVManager plugin: loading\n");
            

                // Store world pointer
                this->world = _parent;


                // Periodic event
                this->updateConnector = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&UAVManager::OnWorldUpdateBegin, this));  


                 // ROS2 node
                rclcpp::init(0, nullptr);
                this->rosNode = rclcpp::Node::make_shared("UAVManager");

                    

                // // CLOCK
                this->iteration = 0;


                // ROS2 UTRAFMAN services
                this->rosSrv_Test = this->rosNode->create_service<utrafman_msgs::srv::Test>(
                    "AirSpace/Test",
                    std::bind(&UAVManager::rosSrvFn_Test, this,
                            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

                this->rosSrv_DeployUAV = this->rosNode->create_service<utrafman_msgs::srv::DeployUAV>(
                    "AirSpace/DeployUAV",
                    std::bind(&UAVManager::rosSrvFn_DeployUAV, this,
                            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));


                //  printf("UTRAFMAN UAVManager plugin: loaded\n");

            }


            void Init()
            {
                // printf("UTRAFMAN UAVManager plugin: inited\n");
      
            }



            void OnWorldUpdateBegin()
            {
                // printf("UTRAFMAN UAVManager plugin: OnWorldUpdateBegin\n");
                // Procesar eventos ROS 2
                rclcpp::spin_some(rosNode);
            }


            void rosSrvFn_Test(
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<utrafman_msgs::srv::Test::Request>  request,   
                      std::shared_ptr<utrafman_msgs::srv::Test::Response> response)  
            {
                gzmsg << "ROS2 service called: UTRAFMAN Test" << std::endl;
                response->sum = request->a + request->b;
            }

  
            void rosSrvFn_DeployUAV(
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<utrafman_msgs::srv::DeployUAV::Request>  request,   
                      std::shared_ptr<utrafman_msgs::srv::DeployUAV::Response> response)  
            {
                // gzmsg << "ROS2 service called: UTRAFMAN DeployUAV" << std::endl;

                std::string modelTXT = R"(
                    <?xml version="1.0" ?>
                    <sdf version="1.7">
                    <model name="base_drone_deployed"> 
                    <pose>0 0 1</pose>
                    <static>false</static>	  
                    <link name="link">  
                    <collision name="collision">
                        <geometry>
                            <box>
                            <size>0.5 0.5 0.50</size>
                            </box>
                        </geometry>
                        <surface>
                            <friction>
                            <ode>
                                <mu>100.0</mu>
                                <mu2>50.0</mu2>
                                <slip1>0.0</slip1>
                                <slip2>0.0</slip2>
                            </ode>
                            </friction>
                        </surface>
                    </collision>
                    <visual name="visual">
                        <geometry>
                            <box>
                            <size>0.5 0.5 0.50</size>
                            </box>
                        </geometry>
                    </visual>
                    </link>
                    </model>
                    </sdf>
                )";
                // printf("SDF string:\n\n%s\n\n",model.c_str());



                // Convert from model string to SDF format
                sdf::SDF modelSDF;
                modelSDF.SetFromString(modelTXT);
                // modelSDF.PrintValues();


                // Model modification
                sdf::ElementPtr modelElement = modelSDF.Root()->GetElement("model");
                modelElement->GetAttribute("name")->SetFromString("nuevo_nombre");
                sdf::ElementPtr poseElement = modelElement->GetElement("pose");
                ignition::math::Pose3d initPose(1.0, 0.0, 10.0, 1.0, 1.0, 1.0);
                poseElement->Set(initPose);
                // modelSDF.PrintValues();

                // Insert the model in the world
                // this->world->InsertModelString(model);
                this->world->InsertModelSDF(modelSDF);
                
   

                // Imprimir los nombres de todos los modelos
                // physics::Model_V models = this->world->Models();
                // for (const auto &model : models)
                // {
                //     std::cout << "Model: " << model->GetName() << std::endl;
                // }



                // gzmsg << "UTRAFMAN service DeployUAV: model deployed" << std::endl;
                response->status = true;

            }

    };

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(UAVManager)

}
