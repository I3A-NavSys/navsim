#include "gazebo/gazebo.hh"
#include <gazebo/physics/physics.hh>

#include "rclcpp/rclcpp.hpp"

#include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <stdio.h>



// #include "gazebo/physics/physics.hh"

namespace gazebo {

class DCnavigation : public ModelPlugin {

private:

// Gazebo
physics::ModelPtr model;
physics::LinkPtr  link;
event::ConnectionPtr updateConnector;


// ROS2 Node
rclcpp::Node::SharedPtr rosNode;

/*
   // ROS management
private:
   ros::NodeHandle *rosnode_;

   // Odometry publisher
private:
   ros::Publisher pub_;

private:
   common::Time last_odom_publish_time;

private:
   double odom_publish_rate = 10; // updates per second

   // Control subscriber
private:
   ros::Subscriber sub_;

private:
   std::string topic_subscripted_ = "bus_command";

private:
   ros::CallbackQueue queue_;

private:
   boost::thread callback_queue_thread_;

*/




// quadcopter parameters

// Posicion de los rotores
// distancia: 25cms, inclinacion: 45º
ignition::math::Vector3<double> pos_CM = ignition::math::Vector3<double>(0, 0, 0); // centro de masas
ignition::math::Vector3<double> pos_NE = ignition::math::Vector3<double>(0.1768, -0.1768, 0);
ignition::math::Vector3<double> pos_NW = ignition::math::Vector3<double>(0.1768, 0.1768, 0);
ignition::math::Vector3<double> pos_SE = ignition::math::Vector3<double>(-0.1768, -0.1768, 0);
ignition::math::Vector3<double> pos_SW = ignition::math::Vector3<double>(-0.1768, 0.1768, 0);

// Margen de velocidad de los motores
const double w_max = 1.5708e+03; // rad/s = 15000rpm
const double w_min = 0;          // rad/s =     0rpm

/*  Fuerza de empuje aerodinamico
    La fuerza principal generada por los rotores
    FT = kFT * w²  
    Asumimos que 
        FT_max = 1kg = 9.8N
    por tanto, queda que... */
const double kFT = 3.9718e-06;

/*  Momento de arrastre de los rotores
    Momento que experimenta el rotor en sentido contrario a su velocidad
    MDR = kMDR * w²
    Asumimos que 
        ...
    por tanto, queda que... */
const double kMDR = 1.3581e-07;

/*  Fuerza de arrastre aerodinamico.
    Fuerza de rozamiento con el aire, contraria a la velocidad.
    FD = -kFD * r_dot*|r_dot|  
    Depende de la forma del objeto en cada eje.  */

/*  Ejes horizontales:
    Asumimos 
        rozamiento similar en ambos ejes (aunque el fuselaje no sea igual)
        Vh_max = 20km/h = 5.5556m/s  (velocidad horizontal maxima)
        roll_max = 30º = 0.5236rad   (inclinacion maxima)
    operando
        FTh_max = 4*FT_max * sin(roll_max)
        FTh_max = FDh_max = 19.6
    por tanto queda que...  */
const double kFDx = 0.6350;
const double kFDy = 0.6350;

/*  Eje vertical:
    Debe verificarse a velocidad limite ascendente que
        FTmax * 4 = Fg + FD_max
    Asumimos que 
        Vz_max = 3m/s  (maxima velocidad de ascenso)
    que nos dará una velocidad limite de descenso de 
        Vz_lim = 2.7689m/s
    operando
        FT_max = 9.8N
        Fg = 1.840gr * 9.8m/s
        FD_max = 21.1681N
    por tanto queda que...  */
const double kFDz = 2.3520;

/*  Momento de arrastre aerodinamico.
    Momento de rozamiento con el aire que sufre el drone al girar.
    Es contrario a la velocidad angular.
    MD = -kMD * rpy_dot * |rpy_dot|  
    Depende de la forma del objeto en cada eje.  */

/*  Ejes horizontales:
    Asumimos 
        rozamiento similar en ambos ejes (aunque el fuselaje no sea igual)
        escenario sin gravedad
        el drone es propulsado por dos rotores del mismo lado a maxima velocidad
        la velocidad angular maxima que alcanza es  Vrp_max = 2 * 2*pi;
    operando
        kMDxy =  2 * FT_max * sin(deg2rad(45))^2 / Vrp_max^2
    por tanto queda que...  */
const double kMDx = 0.0621;
const double kMDy = 0.0621;

/*  Eje vertical:
    Debe verificarse a velocidad limite de rotación sobre el eje Z que
        ...
    Asumimos que 
        Vyaw_max = 4*pi rad/s  (maxima velocidad de rotacion en Z de 2rev/s)
        w_hov2                 (velocidad del rotor para que dos rotores mantengan la sustentacion)
    Ya teniamos que
        MDR = kMDR * w²
        MDz = kMDz * Vyaw²
    operando
        MDz  = MDR             (el rozamiento con el aire compensa el efecto de los rotores)
        kMDz = kMDR* (2 * w_hov2²) / Vyaw_max²
    por tanto queda que...  */
const double kMDz = 0.0039;


// Comandos
double cmd_on   = 0;
double cmd_velX = 0.0;
double cmd_velY = 0.0;
double cmd_velZ = 0.0;
double cmd_rotZ = 0.0;


// Control matrices
Eigen::Matrix<double, 8, 1> x;    // model state
Eigen::Matrix<double, 4, 1> y;    // model output
Eigen::Matrix<double, 4, 8> Kx;   // state control matrix
Eigen::Matrix<double, 4, 4> Ky;   // error control matrix
Eigen::Matrix<double, 4, 1> Hs;   // hovering speed
Eigen::Matrix<double, 4, 1> Wr;   // rotors speeds
Eigen::Matrix<double, 4, 1> r;    // model reference
Eigen::Matrix<double, 4, 1> e;    // model error
Eigen::Matrix<double, 4, 1> E;    // model acumulated error
common::Time prev_iteration_time; // Time to integrate the acumulated error


public:
//Executed when the plugin is loaded in Gazebo
void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    printf("DRONE CHALLENGE Drone plugin: loading\n");

    // Store pointers to the model
    this->model = _parent;
    this->link = this->model->GetLink("dronelink");

 
    // Periodic event
    this->updateConnector = event::Events::ConnectWorldUpdateBegin(
        std::bind(&DCnavigation::OnWorldUpdateBegin, this));  

    // ROS2

///////////


    // Initiates control matrices
    /*
    Kx << -178.5366, -178.5366, -21.9430, -21.9430,  55.6290, -64.9335,  64.9335,  285.3230,
           178.5366, -178.5366,  21.9430, -21.9430, -55.6290, -64.9335, -64.9335,  285.3230,
          -178.5366,  178.5366, -21.9430,  21.9430, -55.6290,  64.9335,  64.9335,  285.3230,
           178.5366,  178.5366,  21.9430,  21.9430,  55.6290,  64.9335, -64.9335,  285.3230;
//    std::cout  << Kx << " \n\n";              
            
    Ky << -85.4924,  85.4924,  921.8127,  179.7244,
          -85.4924, -85.4924,  921.8127, -179.7244,
           85.4924,  85.4924,  921.8127, -179.7244,
           85.4924, -85.4924,  921.8127,  179.7244;
//    std::cout  << Ky << " \n\n";
*/

    Kx << -334.1327, -334.1327, -29.9223, -29.9223,  72.7456, -167.9315,  167.9315, 373.1147,
           334.1327, -334.1327,  29.9223, -29.9223, -72.7456, -167.9315, -167.9315, 373.1147,
          -334.1327,  334.1327, -29.9223,  29.9223, -72.7456,  167.9315,  167.9315, 373.1147,
           334.1327,  334.1327,  29.9223,  29.9223,  72.7456,  167.9315, -167.9315, 373.1147;

    Ky << -0.3078,  0.3078,  1.5803,  0.3081,
          -0.3078, -0.3078,  1.5803, -0.3081,
           0.3078,  0.3078,  1.5803, -0.3081,
           0.3078, -0.3078,  1.5803,  0.3081;
    
    Ky = Ky * 1.0e+03;

    Hs << sqrt(0.300 * 9.8 / 4 / kFT), sqrt(0.300 * 9.8 / 4 / kFT), sqrt(0.300 * 9.8 / 4 / kFT), sqrt(0.300 * 9.8 / 4 / kFT);
    //    std::cout  << Hs << " \n\n";

    Wr << 0, 0, 0, 0;
    //    std::cout  << Wr << " \n\n";

    r << 0, 0, 0, 0;
    //    std::cout  << r  << " \n\n";

    E << 0, 0, 0, 0;
    //    std::cout  << E  << " \n\n";
   

}



void OnWorldUpdateBegin()
{
    // printf("DRONE CHALLENGE Drone plugin: OnWorldUpdateBegin\n");
    // Procesar eventos ROS2
    // rclcpp::spin_some(rosNode);



    // con esto simulamos rotacion de sustentacion
    double w_rotor_NE = sqrt(0.300*9.81/4 / kFT);  // = 430.18 con 4 rotores
    double w_rotor_NW = w_rotor_NE;
    double w_rotor_SE = w_rotor_NE;
    double w_rotor_SW = w_rotor_NE;


    // aplicamos fuerzas/momentos por empuje de rotores
    ignition::math::Vector3<double> FT_NE = ignition::math::Vector3<double>(0, 0, kFT * pow(w_rotor_NE, 2));
    ignition::math::Vector3<double> FT_NW = ignition::math::Vector3<double>(0, 0, kFT * pow(w_rotor_NW, 2));
    ignition::math::Vector3<double> FT_SE = ignition::math::Vector3<double>(0, 0, kFT * pow(w_rotor_SE, 2));
    ignition::math::Vector3<double> FT_SW = ignition::math::Vector3<double>(0, 0, kFT * pow(w_rotor_SW, 2));
    link->AddLinkForce(FT_NE, pos_NE);
    link->AddLinkForce(FT_NW, pos_NW);
    link->AddLinkForce(FT_SE, pos_SE);
    link->AddLinkForce(FT_SW, pos_SW);

    // aplicamos momentos por arrastre de rotores
    ignition::math::Vector3<double> MDR_NE = ignition::math::Vector3<double>(0, 0, kMDR * pow(w_rotor_NE, 2));
    ignition::math::Vector3<double> MDR_NW = ignition::math::Vector3<double>(0, 0, kMDR * pow(w_rotor_NW, 2));
    ignition::math::Vector3<double> MDR_SE = ignition::math::Vector3<double>(0, 0, kMDR * pow(w_rotor_SE, 2));
    ignition::math::Vector3<double> MDR_SW = ignition::math::Vector3<double>(0, 0, kMDR * pow(w_rotor_SW, 2));
    //    printf("MDR  = %.15f\n",MDR_NE.Z() - MDR_NW.Z() - MDR_SE.Z() + MDR_SW.Z());
    link->AddRelativeTorque(MDR_NE - MDR_NW - MDR_SE + MDR_SW);

    // aplicamos fuerza de rozamiento con el aire
    // ignition::math::Vector3<double> FD = ignition::math::Vector3<double>(
    //     -kFDx * linear_vel.X() * fabs(linear_vel.X()),
    //     -kFDy * linear_vel.Y() * fabs(linear_vel.Y()),
    //     -kFDz * linear_vel.Z() * fabs(linear_vel.Z()));
    // printf("drone relative vel \nbZ  %.2f\n|Z| %.2f\nFDz %.2f \n\n",
    //      linear_vel.Z(), fabs(linear_vel.Z()), -kFDz * linear_vel.Z() * fabs(linear_vel.Z()) );
    // link->AddLinkForce(FD, pos_CM);

    // aplicamos momento de rozamiento con el aire
    // ignition::math::Vector3<double> MD = ignition::math::Vector3<double>(
    //     -kMDx * angular_vel.X() * fabs(angular_vel.X()),
    //     -kMDy * angular_vel.Y() * fabs(angular_vel.Y()),
    //     -kMDz * angular_vel.Z() * fabs(angular_vel.Z()));
    // link->AddRelativeTorque(MD);





}



};


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DCnavigation)
} // namespace gazebo
