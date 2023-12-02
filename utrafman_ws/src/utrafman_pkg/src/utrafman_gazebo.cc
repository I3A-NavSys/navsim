#include "rclcpp/rclcpp.hpp"
// #include <boost/format.hpp>
// #include <ignition/math/Pose3.hh>

#include "gazebo/gazebo.hh"
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
// #include "utrafman/deploy_UAV.h"
// #include "utrafman/remove_model.h"
// #include "utrafman/teletransport.h"


namespace gazebo
{
    class utrafman_gazebo : public WorldPlugin
    {
        private:
            // //Number of UAVs in the simulation
            // int num_uavs = 0;

            //Gazebo
            physics::WorldPtr world;
            event::ConnectionPtr updateConnection;

            //ROS 
            rclcpp::Node::SharedPtr rosNode;
            rclcpp::Service<utrafman_msgs::srv::Test>::SharedPtr rosSrv_Test;
            rclcpp::Service<utrafman_msgs::srv::DeployUAV>::SharedPtr rosSrv_DeployUAV;

            // ros::ServiceServer transport_service;
            // ros::ServiceServer remove_service;
            // //ROS callback queue
            // //ros::CallbackQueue rosQueue;
            // //Spinners
            // //ros::AsyncSpinner rosSpinners = ros::AsyncSpinner(1, &this->rosQueue);

        public:

            // bool handleService(
            //                 const std::shared_ptr<rmw_request_id_t> request_header,
            //                 const std::shared_ptr<utrafman_pkg::srv::DeployUAV::Request>  request,
            //                       std::shared_ptr<utrafman_pkg::srv::DeployUAV::Response> response)
            // {
            //     // Aquí implementa la lógica de tu servicio
            //     gzmsg << "Recibida solicitud de servicio con argumento: " << request->argumento << std::endl;

            //     // Puedes realizar alguna lógica aquí y enviar una respuesta
            //     response->status = true;

            //     // Retorna true si el servicio se manejó correctamente
            //     return true;
            // }



            // bool deployUAV_Service(utrafman::deploy_UAV::Request &req, utrafman::deploy_UAV::Response &res) {
            //     //SDF object, model pointer and model string from the request
            //     sdf::SDF sdf_object;
            //     sdf::ElementPtr model_ptr;
            //     std::string model = req.modelSDF;

            //     //Convert from model string to SDF format
            //     sdf_object.SetFromString(model);

            //     //Get the model
            //     model_ptr = sdf_object.Root()->GetElement("model");

            //     //Insert the model in the world
            //     this->parent->InsertModelSDF(sdf_object);

            //     //Increase the number of UAVs
            //     this->num_uavs++;

            //     ROS_INFO("A new UAV has been deployed. Total UAVs: %i", this->num_uavs);
            //     return true;
            // }

            // bool remove_callback(utrafman::remove_model::Request &req, utrafman::remove_model::Response &res) {
            //     res.success.data = false;

            //     //Sent message to a topic to kill the drone
            //     std::string topic_name = "/drone/" + std::to_string(req.uavId) + "/kill";
            //     ros::Publisher topic_pub = this->rosNode->advertise<std_msgs::Bool>(topic_name, 10);
            //     topic_pub.publish(std_msgs::Bool());

            //     //Wait a few seconds (to be sure that the drone has destroyed all its resources)
            //     ros::Duration(1.0).sleep();

            //     //Call Gazebo deleteModel service
            //     ros::ServiceClient gazebo_remove_service = this->rosNode->serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
            //     gazebo_msgs::DeleteModel srv;
            //     srv.request.model_name = "drone_" + std::to_string(req.uavId);
            //     gazebo_remove_service.call(srv);

            //     //Check if the model has been removed
            //     if (srv.response.success) {
            //         res.success.data = true;
            //         //Decrease the number of UAVs
            //         this->num_uavs--;
            //     }

            //     ROS_INFO("A UAV has been removed. Total UAVs: %i", this->num_uavs);
            //     return true;
            // }

            // bool transport_callback(utrafman::teletransport::Request &req, utrafman::teletransport::Response &res) {
            //     //Get the model
            //     physics::ModelPtr drone = this->parent->ModelByName("drone_" + std::to_string(req.uavId));
            //     res.success.data = false;

            //     //Check if the model exists
            //     if (drone != NULL) {
            //         //Set the new pose
            //         ignition::math::Pose3d pose(req.pose.position.x, req.pose.position.y, req.pose.position.z, req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z);
            //         drone->SetWorldPose(pose, true);
            //         res.success.data = true;
            //     }
            //     return true;
            // }

            // void removeModel(const std_msgs::String::ConstPtr& msg)
            // {
            //     ROS_INFO("Elimando drone_%s", msg->data.c_str());
            //     //Eliminamos el modelo con el nombre indicado
            //     //this->parent->RemoveModel("drone_" + std::string(msg->data.c_str())); //Esta implementacion no funciona (hay que liberar los recursos antes)
            //     this->parent->ModelByName("drone_" + std::string(msg->data.c_str()))->Fini();
            // }





            void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
            {
            
                // gzmsg << "Loading UTRAFMAN Gazebo plugin" << std::endl;
                //printf("Loading UTRAFMAN Gazebo plugin\n");

                //Store world pointer
                this->world = _parent;

                // Configurar el evento OnUpdate
                this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&utrafman_gazebo::OnUpdate, this));


                // Check if ROS 2 is initialized
                if (!rclcpp::ok()) {
                    int argc = 0;
                    char **argv = NULL;
                    rclcpp::init(argc, argv);
                }

                // ROS2 node
                this->rosNode = rclcpp::Node::make_shared("utrafman_node");

                // ROS2 services
                this->rosSrv_Test = this->rosNode->create_service<utrafman_msgs::srv::Test>(
                    "utrafman/Test",
                    std::bind(&utrafman_gazebo::rosSrvFn_Test, this,
                            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

                this->rosSrv_DeployUAV = this->rosNode->create_service<utrafman_msgs::srv::DeployUAV>(
                    "utrafman/DeployUAV",
                    std::bind(&utrafman_gazebo::rosSrvFn_DeployUAV, this,
                            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));


                // this->insert_service = this->rosNode->advertiseService("/utm/airspace/deploy_UAV", &UTRAFMAN_gazebo::insert_callback, this);
                // this->remove_service = this->rosNode->advertiseService("/godservice/remove_model", &UTRAFMAN_gazebo::remove_callback, this);
                // this->transport_service = this->rosNode->advertiseService("/godservice/transport_model", &UTRAFMAN_gazebo::transport_callback, this);





                gzmsg << "UTRAFMAN Gazebo plugin loaded" << std::endl;
                // printf("UTRAFMAN Gazebo plugin loaded\n");

                // Iniciar el bucle de spin
                // rclcpp::spin(this->rosNode);



            }


            void OnUpdate()
            {
                // printf("UTRAFMAN Gazebo plugin OnUpdate\n\n");
                // Procesar eventos ROS 2
                rclcpp::spin_some(rosNode);
            }


            void rosSrvFn_Test(
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<utrafman_msgs::srv::Test::Request>  request,   
                      std::shared_ptr<utrafman_msgs::srv::Test::Response> response)  
            {
                gzmsg << "UTRAFMAN service called: Test" << std::endl;
                response->sum = request->a + request->b;
            }


  
            void rosSrvFn_DeployUAV(
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<utrafman_msgs::srv::DeployUAV::Request>  request,   
                      std::shared_ptr<utrafman_msgs::srv::DeployUAV::Response> response)  
            {
                // gzmsg << "UTRAFMAN service called: DeployUAV" << std::endl;

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
                // printf("El modelo es:\n\n%s\n\n",model.c_str());



                // Convert from model string to SDF format
                sdf::SDF modelSDF;
                modelSDF.SetFromString(modelTXT);
                modelSDF.PrintValues();

                // Accedo al elemento 'model'
                sdf::ElementPtr modelElement = modelSDF.Root()->GetElement("model");

                // Modificar el nombre del modelo
                modelElement->GetAttribute("name")->SetFromString("nuevo_nombre");

                // Modificar la posición del modelo (ejemplo: trasladar 1 unidad en el eje X)
                sdf::ElementPtr poseElement = modelElement->GetElement("pose");
                ignition::math::Pose3d nuevaPose(1.0, 0.0, 3.0, 1.0, 1.0, 1.0);
                poseElement->Set(nuevaPose);

                modelSDF.PrintValues();



                // Insert the model in the world
                this->world->InsertModelSDF(modelSDF);
                // this->world->InsertModelString(model);
                
   
                // Imprimir los nombres de todos los modelos
                // physics::Model_V models = this->world->Models();
                // for (const auto &model : models)
                // {
                //     std::cout << "Model: " << model->GetName() << std::endl;
                // }



                // Increase the number of UAVs
                // this->num_uavs++;

                gzmsg << "UTRAFMAN service DeployUAV: model deployed" << std::endl;
                response->status = true;

            }

    };

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(utrafman_gazebo)

}
