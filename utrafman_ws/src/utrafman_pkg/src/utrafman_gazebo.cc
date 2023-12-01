#include <ignition/math/Pose3.hh>
#include <boost/format.hpp>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
//#include "gazebo_msgs/srv/delete_entity.hpp"


#include "rclcpp/rclcpp.hpp"


 
// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include "std_msgs/Bool.h"
// #include "ros/callback_queue.h"
// #include "ros/subscribe_options.h"

#include "utrafman_msgs/srv/deploy_uav.hpp"
// #include "/home/usuario/code/utrafman_ros2/utrafman_ws/install/utrafman_pkg/include/utrafman_pkg/utrafman_pkg/srv/deploy_uav.hpp"


// #include "utrafman/deploy_UAV.h"
// #include "utrafman/remove_model.h"
// #include "utrafman/teletransport.h"


namespace gazebo
{
    class UTRAFMAN_gazebo : public WorldPlugin
    {
        private:
            // //Number of UAVs in the simulation
            // int num_uavs = 0;

            //World pointer
            physics::WorldPtr parent;
            //ROS node handle
            rclcpp::Node::SharedPtr rosNode;

            // //ROS services
            // rclcpp::Service<utrafman_pkg::srv::DeployUAV>::SharedPtr DeployUAV_service;


            // ros::ServiceServer insert_service;
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
            
                gzmsg << "UTRAFMAN gazebo plugin loaded" << std::endl;



                //Store world pointer
                this->parent = _parent;

                // Check if ROS 2 is initialized
                if (!rclcpp::ok()) {
                    int argc = 0;
                    char **argv = NULL;
                    rclcpp::init(argc, argv);
                }

                // Create a ROS 2 node
                this->rosNode = std::make_shared<rclcpp::Node>("UTRAFMANsimd");


                // Crear el servicio
                // this->DeployUAV_service = this->rosNode->create_service<utrafman_pkg::srv::DeployUAV>(
                //     "deploy_uav", std::bind(&UTRAFMAN_gazebo::handleService, this,
                //                                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));



                // this->insert_service = this->rosNode->advertiseService("/utm/airspace/deploy_UAV", &UTRAFMAN_gazebo::insert_callback, this);
                // this->remove_service = this->rosNode->advertiseService("/godservice/remove_model", &UTRAFMAN_gazebo::remove_callback, this);
                // this->transport_service = this->rosNode->advertiseService("/godservice/transport_model", &UTRAFMAN_gazebo::transport_callback, this);

            }


   


    };

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(UTRAFMAN_gazebo)

}
