#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

#include "rclcpp/rclcpp.hpp"
#include "navsim_msgs/srv/deploy_model.hpp"
#include "navsim_msgs/srv/remove_model.hpp"
// #include "navsim/teletransport.h"


namespace gazebo
{
class World : public WorldPlugin
{

private:

// Gazebo
physics::WorldPtr    world;
event::ConnectionPtr updateConnector;
common::Time currentTime;

// ROS2 Node
rclcpp::Node::SharedPtr rosNode;

// ROS2 NAVSIM topics
rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr rosPub_SimTime;
common::Time prevTimePubTime;
double TimePubPeriod = 0.1;    // seconds


// ROS2 NAVSIM services
rclcpp::Service<navsim_msgs::srv::DeployModel>::SharedPtr rosSrv_DeployModel;
rclcpp::Service<navsim_msgs::srv::RemoveModel>::SharedPtr rosSrv_RemoveModel;
common::Time prevRosCheckTime;
double RosCheckPeriod = 0.1;   // seconds


public:

void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    // gzmsg << "NAVSIM World plugin: loading" << std::endl;
    // printf("NAVSIM World plugin: loading\n");


    // Store world pointer
    world = _parent;


    // Periodic event
    updateConnector = event::Events::ConnectWorldUpdateBegin(
        std::bind(&World::OnWorldUpdateBegin, this));  


    // ROS2 node
    if (!rclcpp::ok()) 
    {   
        // Esta función solo debe usarse una vez por aplicación.
        // Si ROS ya está iniciado, esta llamada retorna la función actual sin avisar!
        rclcpp::init(0, nullptr);
    }
    rosNode = rclcpp::Node::make_shared("World");

    // ROS2 NAVSIM topics
    rosPub_SimTime  = rosNode->create_publisher<builtin_interfaces::msg::Time>(
        "World/Time", 1);


    // ROS2 NAVSIM services

    rosSrv_DeployModel = rosNode->create_service<navsim_msgs::srv::DeployModel>(
        "World/DeployModel",
        std::bind(&World::rosSrvFn_DeployModel, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    rosSrv_RemoveModel = rosNode->create_service<navsim_msgs::srv::RemoveModel>(
        "World/RemoveModel",
        std::bind(&World::rosSrvFn_RemoveModel, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));


    //  printf("NAVSIM World plugin: loaded\n");

}


void Init()
{
    // printf("NAVSIM World plugin: inited\n");

    currentTime = world->SimTime();
    prevTimePubTime  = currentTime;
    prevRosCheckTime = currentTime;


}



void OnWorldUpdateBegin()
{
    // printf("NAVSIM World plugin: OnWorldUpdateBegin\n");

    currentTime = world->SimTime();
    TimeBroadcast();

    // ROS2 events proceessing
    CheckROS();
}



void CheckROS()
{
    
    // Check if the simulation was reset
    if (currentTime < prevRosCheckTime)
        prevRosCheckTime = currentTime; // The simulation was reset

    double interval = (currentTime - prevRosCheckTime).Double();
    if (interval < RosCheckPeriod) return;

    prevRosCheckTime = currentTime;

    // ROS2 events proceessing
    rclcpp::spin_some(rosNode);

}




// void rosSrvFn_Time(
//     const std::shared_ptr<rmw_request_id_t> request_header,
//     const std::shared_ptr<navsim_msgs::srv::Time::Request>  request,   
//             std::shared_ptr<navsim_msgs::srv::Time::Response> response)  
// {
//     // printf("NAVSIM World plugin: Service Time called\n");

//     if (request->reset)
//     {
//         // printf("Simulation reset\n");
//         world->ResetTime();
//     }

//     common::Time simTime = world->SimTime();
//     // printf("time: %.2f\n",simTime.Double());

//     response->time.sec = simTime.sec;
//     response->time.nanosec = simTime.nsec;
// }


void rosSrvFn_DeployModel(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navsim_msgs::srv::DeployModel::Request>  request,   
          std::shared_ptr<navsim_msgs::srv::DeployModel::Response> response)  
{
    // printf("NAVSIM World plugin: DeployModel\n");

    // printf("Model SDF:  %s\n", request->model_sdf.c_str());
    // printf("Model name: %s\n", request->name.c_str());
    // printf("Model pose: %.2f %.2f %.2f - %.2f\n",
    //     request->pos.x, request->pos.y, request->pos.z,
    //     request->rot);


    // std::string modelTXT = R"(
    //     <?xml version="1.0" ?>
    //     <sdf version="1.7">
    //     <model name="base_drone_deployed"> 
    //     <pose>0 0 1</pose>
    //     <static>false</static>	  
    //     <link name="link">  
    //     <collision name="collision">
    //         <geometry>
    //             <box>
    //             <size>0.5 0.5 0.50</size>
    //             </box>
    //         </geometry>
    //         <surface>
    //             <friction>
    //             <ode>
    //                 <mu>100.0</mu>
    //                 <mu2>50.0</mu2>
    //                 <slip1>0.0</slip1>
    //                 <slip2>0.0</slip2>
    //             </ode>
    //             </friction>
    //         </surface>
    //     </collision>
    //     <visual name="visual">
    //         <geometry>
    //             <box>
    //             <size>0.5 0.5 0.50</size>
    //             </box>
    //         </geometry>
    //     </visual>
    //     </link>
    //     </model>
    //     </sdf>
    // )";
    // printf("SDF string:\n\n%s\n\n",model.c_str());


    // Convert from model string to SDF format
    sdf::SDF modelSDF;
    modelSDF.SetFromString(request->model_sdf);
    // modelSDF.PrintValues();


    // Model modification
    sdf::ElementPtr modelElement = modelSDF.Root()->GetElement("model");
    modelElement->GetAttribute("name")->SetFromString(request->name);
    sdf::ElementPtr poseElement = modelElement->GetElement("pose");
    ignition::math::Pose3d initPose(
        request->pos.x, request->pos.y, request->pos.z, 
        request->rot.x, request->rot.y, request->rot.z);
    poseElement->Set(initPose);
    // modelSDF.PrintValues();

    // Insert the model in the world
    // this->world->InsertModelString(model);
    world->InsertModelSDF(modelSDF);
    


    // Imprimir los nombres de todos los modelos
    // physics::Model_V models = this->world->Models();
    // for (const auto &model : models)
    // {
    //     std::cout << "Model: " << model->GetName() << std::endl;
    // }



    // gzmsg << "NAVSIM service DeployModel: model deployed" << std::endl;
    response->status = true;

}


void rosSrvFn_RemoveModel(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navsim_msgs::srv::RemoveModel::Request>  request,   
          std::shared_ptr<navsim_msgs::srv::RemoveModel::Response> response)  
{
    printf("NAVSIM World plugin: RemoveModel\n");
    physics::ModelPtr model = this->world->ModelByName(request->name);

    if(!model)
    {
        printf("UAV not found: %s \n", request->name.c_str());
        return;
    }

    world->RemoveModel(model);
    printf("UAV %s removed from air space\n", request->name.c_str());
 
}




void TimeBroadcast()
{
    // printf("WORLD Time broadcast \n");

    // Check if the simulation was reset
    if (currentTime < prevTimePubTime)
        prevTimePubTime = currentTime; // The simulation was reset

    double interval = (currentTime - prevTimePubTime).Double();
    if (interval < TimePubPeriod) return;

    // printf("WORLD transmitting time \n");
    // printf("current time: %.3f \n\n", currentTime.Double());

    prevTimePubTime = currentTime;

    
    builtin_interfaces::msg::Time msg;
    msg.sec = currentTime.sec;
    msg.nanosec = currentTime.nsec;

    rosPub_SimTime->publish(msg);


}



};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(World)

}
