#include "gazebo/gazebo.hh"
#include <gazebo/physics/physics.hh>

#include "rclcpp/rclcpp.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
// #include <stdio.h>

#include "navsim_msgs/msg/telemetry.hpp"
#include "navsim_msgs/msg/remote_command.hpp"



namespace gazebo {

class UAM_aerotaxi_cmd : public ModelPlugin {

private:

////////////////////////////////////////////////////////////////////////
// Gazebo

physics::ModelPtr    model;
physics::LinkPtr     link;
event::ConnectionPtr updateConnector;



////////////////////////////////////////////////////////////////////////
// ROS2

std::string UAVname;
rclcpp::Node::SharedPtr rosNode;

rclcpp::Publisher<navsim_msgs::msg::Telemetry>::SharedPtr rosPub_Telemetry;
common::Time prevTelemetryPubTime;
double TelemetryPeriod = 0.2;    // seconds

rclcpp::Subscription<navsim_msgs::msg::RemoteCommand>::SharedPtr rosSub_RemoteCommand;
common::Time prevCommandCheckTime;
double CommandCheckPeriod = 0.1;     // seconds





////////////////////////////////////////////////////////////////////////
// quadcopter parameters

const double g    = 9.8;
const double mass = 2000;


// Velocidad de rotación de los motores (rad/s)
double w_rotor_NE;
double w_rotor_NW;
double w_rotor_SE;
double w_rotor_SW;

// angular velocity of the rotors
const double w_max = 62.8319;       // rad/s = 600rpm
const double w_hov = 41.8879;       // rad/s = 400rpm
const double w_min = 0;             // rad/s =   0rpm

//Rotors position at distance 15cms from the center of mass, inclination 45º from the axes
ignition::math::Vector3<double> pos_CM = ignition::math::Vector3<double>( 0.00,  0.00, 0.00);   // center of mass
ignition::math::Vector3<double> pos_NE = ignition::math::Vector3<double>( 1.00, -1.80, 0.50);   
ignition::math::Vector3<double> pos_NW = ignition::math::Vector3<double>( 1.00,  1.80, 0.50);
ignition::math::Vector3<double> pos_SE = ignition::math::Vector3<double>(-2.50, -1.50, 0.50);
ignition::math::Vector3<double> pos_SW = ignition::math::Vector3<double>(-2.50,  1.50, 0.50);

// Aero-dynamic thrust force constant
// Force generated by the rotors is FT = kFT * w²
const double kFT_N = 3.9895;        // north side
const double kFT_S = 1.5958;        // south side


//Aero-dynamic drag force constant
//Moment generated by the rotors is MDR = kMDR * w²
const double kMDR_N = 5.9683;
const double kMDR_S = 1.4921;


//Aero-dynamic drag force constant per axis
//Drag force generated by the air friction, opposite to the velocity is FD = -kFD * r_dot*|r_dot| (depends on the shape of the object in each axis).
const double kFDx = 3.0625;
const double kFDy = 4.0000;
const double kFDz = 7.8400;


//Aero-dynamic drag moment constant per axis
//Drag moment generated by the air friction, opposite to the angular velocity is MD = -kMD * rpy_dot*|rpy_dot| (depends on the shape of the object in each axis).
const double kMDx = 34.0439;
const double kMDy = 44.3280;
const double kMDz = 20.5427;


////////////////////////////////////////////////////////////////////////
// Control matrices variables
Eigen::Matrix<double,8,1> x;  // model state
Eigen::Matrix<double,4,1> y;  // model output
Eigen::Matrix<double,4,8> Kx; // state control matrix
Eigen::Matrix<double,4,4> Ki; // error control matrix
Eigen::Matrix<double,4,1> Hs; // hovering speed
Eigen::Matrix<double,4,1> u;  // input (rotors speeds)
Eigen::Matrix<double,4,1> r;  // model reference
Eigen::Matrix<double,4,1> e;  // model error
Eigen::Matrix<double,4,1> E;  // model acumulated error

int E_max = 150;                // maximun model acumulated error
common::Time prevControlTime = 0; // Fecha de la ultima actualizacion del control de bajo nivel 
        

////////////////////////////////////////////////////////////////////////
// Aircraft sensor data

common::Time  currentTime;

// earth position and rotation
// double Px = 0; 
// double Py = 0;
// double Pz = 0;
double ePhi   = 0; 
double eTheta = 0;
double ePsi   = 0;

ignition::math::Vector3<double> eV;        // earth   linear velocity
ignition::math::Vector3<double> hV;        // horizon linear velocity
ignition::math::Vector3<double> bV;        // body    linear velocity
ignition::math::Vector3<double> bW;        // body   angular velocity
    



////////////////////////////////////////////////////////////////////////
// Navigation parameters

common::Time CommandExpTime;
bool  rotors_on = false;

// Command
bool   cmd_on   = false;     // (bool)   rotors active 
double cmd_velX = 0;         // (m/s)    commanded forward  speed
double cmd_velY = 0;         // (m/s)    commanded lateral  speed
double cmd_velZ = 0;         // (m/s)    commanded vertical speed
double cmd_rotZ = 0;         // (rad/s)  commanded angular  speed

// Command limits
double cmd_velX_MAX = 35;    // (m/s)          max forward  speed  (126 kms/h)
double cmd_vel_step = 10;    // (m/s)          max forward  speed step
double cmd_velY_MAX = 10;    // (m/s)          max lateral  speed
double cmd_velZ_MAX = 10;    // (m/s)          max vertical speed
double cmd_rotZ_MAX =  1;    // (rad/s)        max angular  speed



////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

public: 
void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    // printf("UAM AEROTAXI plugin: loading\n");

    // Get information from the model
    model = _parent;
    UAVname = model->GetName();
    link = model->GetLink("dronelink");

    // Periodic event
    updateConnector = event::Events::ConnectWorldUpdateBegin(
        std::bind(&UAM_aerotaxi_cmd::OnWorldUpdateBegin, this));  

    // ROS2
    if (rclcpp::ok()) 
    {
        rosNode = rclcpp::Node::make_shared(this->UAVname);

        rosPub_Telemetry = rosNode->create_publisher<navsim_msgs::msg::Telemetry>(
            "/NavSim/" + UAVname + "/Telemetry", 10);

        rosSub_RemoteCommand = rosNode->create_subscription<navsim_msgs::msg::RemoteCommand>(
            "/NavSim/" + UAVname + "/RemoteCommand", 10,
            std::bind(&UAM_aerotaxi_cmd::rosTopFn_RemoteCommand, this, 
                    std::placeholders::_1));
            
    
    }
    else
    {   
        std::cout << "\x1B[2J\x1B[H";       // Clear screen
        printf("\nERROR: NavSim world plugin is not running ROS2!\n\n");
    }


    //Control matrices

    // Poles = [-2 -2 -3 -3 -4 -4 -4 -4 -5 -5 -5 -5]

    // Kx <<  -26.3432,  -92.3065,   -5.1944,  -18.2013,   17.1429,  -20.4300,    5.8305,   19.2342,
    //         26.3432,  -92.3065,    5.1944,  -18.2013,  -17.1429,  -20.4300,   -5.8305,   19.2342,
    //       -105.3728,  230.7664,  -20.7777,   45.5032,  -51.4286,   51.0751,   23.3219,   19.2342,
    //        105.3728,  230.7664,   20.7777,   45.5032,   51.4286,   51.0751,  -23.3219,   19.2342;

    // Ki <<  -15.9195,    4.5432,   42.7428,   38.0952,
    //        -15.9195,   -4.5432,   42.7428,  -38.0952,
    //         39.7987,   18.1729,   42.7428, -114.2857,
    //         39.7987,  -18.1729,   42.7428,  114.2857;





    // Poles = [-1 -1 -1 -1 -2 -2 -2 -2 -3 -3 -4 -4]

    // Kx <<  -12.9861,  -45.5032,   -3.7103,  -13.0009,    5.7143,   -6.6331,    1.8930,    6.4114,
    //         12.9861,  -45.5032,    3.7103,  -13.0009,   -5.7143,   -6.6331,   -1.8930,    6.4114,
    //        -51.9443,  113.7581,  -14.8412,   32.5023,  -17.1429,   16.5828,    7.5721,    6.4114,
    //         51.9443,  113.7581,   14.8412,   32.5023,   17.1429,   16.5828,   -7.5721,    6.4114;

    // Ki <<   -3.1839,    0.9086,    4.2743,    3.8095,
    //         -3.1839,   -0.9086,    4.2743,   -3.8095,
    //          7.9597,    3.6346,    4.2743,  -11.4286,
    //          7.9597,   -3.6346,    4.2743,   11.4286;


    // Poles = [-0.5 -0.5 -0.5 -0.5 -1 -1 -1 -1 -2 -2 -3 -3]

    Kx <<   -5.1944,  -18.2013,   -2.4117,   -8.4506,    2.8571,   -1.5256,    0.4354,    3.2057,
             5.1944,  -18.2013,    2.4117,   -8.4506,   -2.8571,   -1.5256,   -0.4354,    3.2057,
           -20.7777,   45.5032,   -9.6468,   21.1265,   -8.5714,    3.8140,    1.7416,    3.2057,
            20.7777,   45.5032,    9.6468,   21.1265,    8.5714,    3.8140,   -1.7416,    3.2057;

    Ki <<   -0.3980,    0.1136,    1.0686,    0.9524,
            -0.3980,   -0.1136,    1.0686,   -0.9524,
             0.9950,    0.4543,    1.0686,   -2.8571,
             0.9950,   -0.4543,    1.0686,    2.8571;
                
	// Linearization point
    Hs << w_hov, w_hov, w_hov, w_hov;
    //    std::cout  << Hs << " \n\n";

	//Reference to follow
	r << 0, 0, 0, 0;

	//Cumulative error in the reference following
	E << 0, 0, 0, 0;

	//Control input signal (rotor real speeds)
	u << w_hov, w_hov, w_hov, w_hov;

}



void Init()
{
    // printf("AeroTaxi Init\n");

    currentTime = model->GetWorld()->SimTime();
    prevTelemetryPubTime = currentTime;
    prevCommandCheckTime = currentTime;

////////////////////////////asumimos un comando!!!
    // cmd_on   =  true  ;
    // cmd_velX =  5.0;
    // cmd_velY =  0.0;
    // cmd_velZ =  0.2;
    // cmd_rotZ =  0.1; 
    // printf("UAV command: ON: %.0d       velX: %.1f  velY: %.1f  velZ: %.1f       rotZ: %.1f \n\n", 
    //         cmd_on, cmd_velX, cmd_velY, cmd_velZ, cmd_rotZ);
////////////////////////////
}



void OnWorldUpdateBegin()
{
    // Clear screen
    // std::cout << "\x1B[2J\x1B[H";
    // printf("AeroTaxi plugin: OnWorldUpdateBegin\n");

    // Read data from aircraft sensors
    ReadSensors();

    // Check ROS2 subscriptions
    CheckSubs();
    
    // // Platform low level control
    ServoControl();
    PlatformDynamics();

    // Telemetry communication
    Telemetry();

}




void ReadSensors()
{
    // Get current simulation time
    currentTime = model->GetWorld()->SimTime();

    // aircraft position & rotation
    ignition::math::Pose3<double> pose = model->WorldPose();
    // Px = pose.X(); 
    // Py = pose.Y();
    // Pz = pose.Z();
    ePhi   = pose.Roll(); 
    eTheta = pose.Pitch();
    ePsi   = pose.Yaw();

    // aircraft velocities
    eV = model->WorldLinearVel();        // earth   linear velocity
    bV = model->RelativeLinearVel();     // body    linear velocity
    bW = model->RelativeAngularVel();    // body   angular velocity

    pose.Set(0,0,0,ePhi,eTheta,0);
    ignition::math::Quaterniond body2horizon = pose.Rot();
    hV = body2horizon.RotateVector(bV);  // horizon linear velocity

}    




void rosTopFn_RemoteCommand_BUENO(const std::shared_ptr<navsim_msgs::msg::RemoteCommand> msg)
// This function listen and follow remote commands
{
    // printf("Data received in topic Remote Pilot\n");
    // printf("Received RemoteCommand: uav=%s, on=%d, cmd=[%f, %f, %f, %f], duration=(%d, %d)\n",
    //        msg->uav_id.c_str(), 
    //        msg->on, 
    //        msg->vel.linear.x, msg->vel.linear.y, msg->vel.linear.z,
    //        msg->vel.angular.z, 
    //        msg->duration.sec, msg->duration.nanosec);
    
    //Velocities commanded in horizon axes
    cmd_on   =  msg->on;
    cmd_velX =  msg->vel.linear.x;
    cmd_velY =  msg->vel.linear.y;
    cmd_velZ =  msg->vel.linear.z;
    cmd_rotZ =  msg->vel.angular.z; 

    common::Time duration;
    duration.sec  = msg->duration.sec;
    duration.nsec = msg->duration.nanosec;
    CommandExpTime = currentTime + duration;
    // printf("current control time: %.3f \n", currentTime.Double());
    // printf("command duration: %.3f \n", duration.Double());
    // printf("command expiration time: %.3f \n\n", CommandExpTime.Double());
}




void rosTopFn_RemoteCommand(const std::shared_ptr<navsim_msgs::msg::RemoteCommand> msg)
// This function listen and follow remote commands
{
    // printf("Data received in topic Remote Pilot\n");
    // printf("Received RemoteCommand: uav=%s, on=%d, cmd=[%f, %f, %f, %f], duration=(%d, %d)\n",
    //        msg->uav_id.c_str(), 
    //        msg->on, 
    //        msg->vel.linear.x, msg->vel.linear.y, msg->vel.linear.z,
    //        msg->vel.angular.z, 
    //        msg->duration.sec, msg->duration.nanosec);
    

    // Get the command
    cmd_on   = msg->on;
    cmd_velX = msg->vel.linear.x;
    cmd_velY = msg->vel.linear.y;
    cmd_velZ = msg->vel.linear.z;
    cmd_rotZ = msg->vel.angular.z;

    common::Time duration;
    duration.sec  = msg->duration.sec;
    duration.nsec = msg->duration.nanosec;
    CommandExpTime = currentTime + duration;
    // printf("current control time: %.3f \n", currentTime.Double());
    // printf("command duration: %.3f \n", duration.Double());
    // printf("command expiration time: %.3f \n\n", CommandExpTime.Double());


    // Corregimos comandos fuera de rango
    if (cmd_velX < -cmd_velX_MAX)    cmd_velX = -cmd_velX_MAX;
    if (cmd_velX >  cmd_velX_MAX)    cmd_velX =  cmd_velX_MAX;

    if (cmd_velY < -cmd_velY_MAX)    cmd_velY = -cmd_velY_MAX;
    if (cmd_velY >  cmd_velY_MAX)    cmd_velY =  cmd_velY_MAX;

    if (cmd_velZ < -cmd_velZ_MAX)    cmd_velZ = -cmd_velZ_MAX;
    if (cmd_velZ >  cmd_velZ_MAX)    cmd_velZ =  cmd_velZ_MAX;

    if (cmd_rotZ < -cmd_rotZ_MAX)    cmd_rotZ = -cmd_rotZ_MAX;
    if (cmd_rotZ >  cmd_rotZ_MAX)    cmd_rotZ =  cmd_rotZ_MAX;




    // fix velocity X
    // printf("cmd_velX: %.2f \n", cmd_velX);
    double dif = cmd_velX - hV.X();
    if (dif >  cmd_vel_step)    cmd_velX = hV.X() + cmd_vel_step;
    else 
    if (dif < -cmd_vel_step)    cmd_velX = hV.X() - cmd_vel_step;
    // printf("cmd_velX: %.2f \n\n", cmd_velX);



    // fix rotation command
    double factor = exp(-0.065 * eV.Length());
    // printf("cmd_velX: %.2f \n", eV.Length());
    // printf("Factor:   %.2f \n\n", factor);
    cmd_rotZ *= factor;




    // // Aceleración tangencial acotada
    // ignition::math::Vector3<double> Ve  = V_c - hV;  // error en velocidad lineal
    // double AT = Ve.Length();
    // if (AT > 0)
    // {
    //     if (AT > A_MAX)
    //     {
    //         // acotamos AT
    //         AT = A_MAX;
    //     }
    //     // Acotamos velocidad lineal comandada
    //     V_c = hV + Ve * AT/Ve.Length();
    // }
    

    // // Aceleración normal acotada
    // double We  = W_c - bW.Z();      // error en velocidad angular
    // double AN = abs(We);
    // if (AN > 0)
    // {
    //     if (AN > A_MAX - AT)
    //     {
    //         // acotamos AN
    //         AN = A_MAX - AT;
    //     }
    //     // Acotamos velocidad angular comandada
    //     W_c = bW.Z() +  We * AN/abs(We);
    // }
    

}




ignition::math::Vector3<double> Horizon2Body(ignition::math::Vector3<double> hV)
// Transforma un vector expresado en eje horizonte a eje cuerpo
{
    ignition::math::Pose3<double> pose;
    pose.Set(0,0,0,ePhi,eTheta,0);
    ignition::math::Quaterniond orientation = pose.Rot();
    ignition::math::Vector3d bV = orientation.RotateVectorReverse(hV);
    // std::cout  << "bV:  " << bV  << " \n\n";
    return bV;
}




void commandOff()
{
    cmd_on   = false;
    cmd_velX = 0;
    cmd_velY = 0;
    cmd_velZ = 0;
    cmd_rotZ = 0; 
}




void hover()
{
    commandOff();
    cmd_on = true;
}




void rotorsOff()
{

    // Apagamos motores
    w_rotor_NE = 0;
    w_rotor_NW = 0;
    w_rotor_SE = 0;
    w_rotor_SW = 0;
    rotors_on = false;

    // Reset del control
    E << 0, 0, 0, 0;
   
}




void ServoControl()
{
    // This function converts 
    // [bVx bVy bVz bWz]     a navigation command (desired velocity vector and rotation)
    // [Wnw Wne Wsw Wse]     to speeds of the four rotors

    // printf("AEROTAXI plugin: ServoControl\n");


    if (cmd_on == false)
    {
        rotorsOff();
        return;
    }


    // Check if the simulation was reset
    if  (currentTime < prevControlTime)
    {
        prevControlTime = currentTime; 

        commandOff();
        rotorsOff();
        return;
    }


    // Check if the command has expired
    if (CommandExpTime < currentTime)
    {
        hover(); 
    }


    // Check if the fly starts
    if (rotors_on == false)
    {
        rotors_on = true;
        prevControlTime = currentTime; 
    }
    // printf("previous control time: %.3f \n", prevControlTime.Double());

    double interval = (currentTime - prevControlTime).Double();
    // printf("control interval (seconds): %.6f \n", interval);
    
    prevControlTime = currentTime;


    //Velocities commanded in body axes
    ignition::math::Vector3<double> h_cmd = ignition::math::Vector3<double>(cmd_velX, cmd_velY, cmd_velZ);
    // printf("h_cmd: %.2f  %.2f  %.2f \n", cmd_velX, cmd_velY, cmd_velZ);
    ignition::math::Vector3 b_cmd2 = Horizon2Body(h_cmd);
    // printf("b_cmd: %.2f  %.2f  %.2f \n\n", b_cmd2.X(),b_cmd2.Y(),b_cmd2.Z());


    Eigen::Matrix<double,3,1> b_cmd;
    b_cmd(0,0) = b_cmd2.X();
    b_cmd(1,0) = b_cmd2.Y();
    b_cmd(2,0) = b_cmd2.Z();
    // std::cout  << "b_cmd:  " << b_cmd.transpose()  << " \n\n";

	// Assign the model state
    x(0,0) = ePhi;
    x(1,0) = eTheta;
    x(2,0) = bW.X();
    x(3,0) = bW.Y();
    x(4,0) = bW.Z();
    x(5,0) = bV.X();
    x(6,0) = bV.Y();
    x(7,0) = bV.Z();
    // std::cout  << "x:  " << x.transpose()  << " \n\n";

	// Assign the model output
    y(0,0) = bV.X();
    y(1,0) = bV.Y();
    y(2,0) = bV.Z();
    y(3,0) = bW.Z();
    // std::cout  << "y:  " << y.transpose()  << " \n\n";

	// Assign the model reference to be followed
    r(0,0) = b_cmd(0,0);    // bXdot
    r(1,0) = b_cmd(1,0);    // bYdot
    r(2,0) = b_cmd(2,0);    // bZdot
    r(3,0) = cmd_rotZ;      // hZdot
    // std::cout  << "r:  " << r.transpose()  << " \n\n";


	// Error between the output and the reference (between the commanded velocity and the drone velocity)
	e = y - r;
    // std::cout  << "e:  " << e.transpose()  << " \n\n";

	// Cumulative error
    E = E + (e * interval);
    // std::cout  << "E:  " << E.transpose()  << " \n";

    if (E(0, 0) >  E_max)   E(0, 0) =  E_max;
    if (E(0, 0) < -E_max)   E(0, 0) = -E_max;
    if (E(1, 0) >  E_max)   E(1, 0) =  E_max;
    if (E(1, 0) < -E_max)   E(1, 0) = -E_max;
    if (E(2, 0) >  E_max)   E(2, 0) =  E_max;
    if (E(2, 0) < -E_max)   E(2, 0) = -E_max;
    if (E(3, 0) >  E_max)   E(3, 0) =  E_max;
    if (E(3, 0) < -E_max)   E(3, 0) = -E_max;
    // std::cout  << "E:  " << E.transpose()  << " \n\n";

    
    // control del sistema dinamico
    u = Hs - Kx * x - Ki * E;
    // std::cout  << "u: " << u.transpose()  << " \n\n";
   
	// Saturating the rotors speed in case of exceeding the maximum or minimum rotations
	if (u(0, 0) > w_max) u(0, 0) = w_max;
	if (u(0, 0) < w_min) u(0, 0) = w_min;
	if (u(1, 0) > w_max) u(1, 0) = w_max;
	if (u(1, 0) < w_min) u(1, 0) = w_min;
	if (u(2, 0) > w_max) u(2, 0) = w_max;
	if (u(2, 0) < w_min) u(2, 0) = w_min;
	if (u(3, 0) > w_max) u(3, 0) = w_max;
	if (u(3, 0) < w_min) u(3, 0) = w_min;
    // std::cout  << "u: " << u.transpose()  << " \n\n";


    // Asignamos la rotación de los motores
    w_rotor_NE = u(0, 0);
    w_rotor_NW = u(1, 0);
    w_rotor_SE = u(2, 0);
    w_rotor_SW = u(3, 0);

}




void PlatformDynamics()
{
    // Esta funcion traduce 
    // la velocidad de rotacion de los 4 motores
    // a fuerzas y torques del solido libre

    // con esto simulamos rotacion de sustentacion
    // w_rotor_NE = w_hov;
    // w_rotor_NW = w_rotor_NE;
    // w_rotor_SE = w_rotor_NE;
    // w_rotor_SW = w_rotor_NE;


	//Apply the thrust force to the drone
    ignition::math::Vector3<double> FT_NE = ignition::math::Vector3<double>(0, 0, kFT_N * pow(w_rotor_NE, 2));
    ignition::math::Vector3<double> FT_NW = ignition::math::Vector3<double>(0, 0, kFT_N * pow(w_rotor_NW, 2));
    ignition::math::Vector3<double> FT_SE = ignition::math::Vector3<double>(0, 0, kFT_S * pow(w_rotor_SE, 2));
    ignition::math::Vector3<double> FT_SW = ignition::math::Vector3<double>(0, 0, kFT_S * pow(w_rotor_SW, 2));
    link->AddLinkForce(FT_NE, pos_NE);
    link->AddLinkForce(FT_NW, pos_NW);
    link->AddLinkForce(FT_SE, pos_SE);
    link->AddLinkForce(FT_SW, pos_SW);


	//Apply the drag moment to the drone
    ignition::math::Vector3<double> MDR_NE = ignition::math::Vector3<double>(0, 0, kMDR_N * pow(w_rotor_NE, 2));
    ignition::math::Vector3<double> MDR_NW = ignition::math::Vector3<double>(0, 0, kMDR_N * pow(w_rotor_NW, 2));
    ignition::math::Vector3<double> MDR_SE = ignition::math::Vector3<double>(0, 0, kMDR_S * pow(w_rotor_SE, 2));
    ignition::math::Vector3<double> MDR_SW = ignition::math::Vector3<double>(0, 0, kMDR_S * pow(w_rotor_SW, 2));
    //    printf("MDR  = %.15f\n",MDR_NE.Z() - MDR_NW.Z() - MDR_SE.Z() + MDR_SW.Z());
    link->AddRelativeTorque(MDR_NE - MDR_NW - MDR_SE + MDR_SW);


	// Apply the air friction force to the drone
    ignition::math::Vector3<double> linear_vel = model->RelativeLinearVel();
    ignition::math::Vector3<double> FD = ignition::math::Vector3<double>(
        -kFDx * linear_vel.X() * fabs(linear_vel.X()),
        -kFDy * linear_vel.Y() * fabs(linear_vel.Y()),
        -kFDz * linear_vel.Z() * fabs(linear_vel.Z()));
    // printf("drone relative vel \nbZ  %.2f\n|Z| %.2f\nFDz %.2f \n\n",
    //      linear_vel.Z(), fabs(linear_vel.Z()), -kFDz * linear_vel.Z() * fabs(linear_vel.Z()) );
    link->AddLinkForce(FD, pos_CM);


	// Apply the air friction moment to the drone
    ignition::math::Vector3<double> angular_vel = model->RelativeAngularVel();
    ignition::math::Vector3<double> MD = ignition::math::Vector3<double>(
        -kMDx * angular_vel.X() * fabs(angular_vel.X()),
        -kMDy * angular_vel.Y() * fabs(angular_vel.Y()),
        -kMDz * angular_vel.Z() * fabs(angular_vel.Z()));
    link->AddRelativeTorque(MD);

}




void CheckSubs()
{
    
    // Check if the simulation was reset
    common::Time currentTime = model->GetWorld()->SimTime();

    if (currentTime < prevCommandCheckTime)
        prevCommandCheckTime = currentTime; // The simulation was reset

    double interval = (currentTime - prevCommandCheckTime).Double();
    if (interval < CommandCheckPeriod) return;

    // printf("UAV %s checking ROS2 subscriptions \n", UAVname.c_str());
    // printf("current time: %.3f \n\n", currentTime.Double());

    prevCommandCheckTime = currentTime;

    // ROS2 events proceessing
    rclcpp::spin_some(rosNode);

}




void Telemetry()
{
    // printf("UAV Telemetry \n");

    // Check if the simulation was reset
    common::Time currentTime = model->GetWorld()->SimTime();

    if (currentTime < prevTelemetryPubTime)
        prevTelemetryPubTime = currentTime; // The simulation was reset

    double interval = (currentTime - prevTelemetryPubTime).Double();
    if (interval < TelemetryPeriod) return;

    // printf("UAV %s transmitting telemetry \n", UAVname.c_str());
    // printf("current time: %.3f \n\n", currentTime.Double());

    prevTelemetryPubTime = currentTime;


    // Getting model status
    ignition::math::Pose3<double> pose = model->WorldPose();
    ignition::math::Vector3<double> linear_vel = model->WorldLinearVel();
    ignition::math::Vector3<double> angular_vel = model->RelativeAngularVel();
    // printf("drone xyz =  %.2f  %.2f  %.2f \n", pose.X(), pose.Y(), pose.Z());
    // printf("drone YPR =  %.2f  %.2f  %.2f \n", pose.Yaw(), pose.Pitch(), pose.Roll());
    // printf("drone         vel xyz =  %.2f  %.2f  %.2f\n",  linear_vel.X(),  linear_vel.Y(),  linear_vel.Z());
    // printf("drone angular vel xyz =  %.2f  %.2f  %.2f\n", angular_vel.X(), angular_vel.Y(), angular_vel.Z());


    navsim_msgs::msg::Telemetry msg;

    msg.pose.position.x    = pose.X();
    msg.pose.position.y    = pose.Y();
    msg.pose.position.z    = pose.Z();
    msg.pose.orientation.x = pose.Roll();
    msg.pose.orientation.y = pose.Pitch();
    msg.pose.orientation.z = pose.Yaw();
    msg.pose.orientation.w = 0;

    msg.velocity.linear.x  = linear_vel.X();
    msg.velocity.linear.y  = linear_vel.Y();
    msg.velocity.linear.z  = linear_vel.Z();
    msg.velocity.angular.x = angular_vel.X();
    msg.velocity.angular.y = angular_vel.Y();
    msg.velocity.angular.z = angular_vel.Z();
    
    // msg.wip = 42;
    // msg.fpip = true;
    msg.time.sec = currentTime.sec;
    msg.time.nanosec = currentTime.nsec;

    rosPub_Telemetry->publish(msg);


}




}; // class

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(UAM_aerotaxi_cmd)
} // namespace gazebo
