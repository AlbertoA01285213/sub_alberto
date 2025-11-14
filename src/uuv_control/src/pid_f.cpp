#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <array>
#include <string>

// --- Mensajes de ROS ---
// Mensajes estándar para publicar el estado
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/wrench.hpp"

// --- Tu librería de control original ---
// (Asegúrate de que la ruta de inclusión sea correcta en tu CMakeLists.txt)
#include "clibs/controller_pid_f/lib_pid_f.hpp"

// --- Tus mensajes de trayectoria personalizados ---
// (Asumiendo que tu paquete de mensajes se llama 'uuv_msgs' como en tu Python)
#include "uuv_msgs/msg/trayectory.hpp"
#include "uuv_msgs/msg/trajectory_point.hpp"

// --- Utilidades ---
// Para convertir de Euler a Cuaternión (para el mensaje Pose)
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // No es estándar, pero si usas tf2_ros, está disponible.
                                               // Si no, haremos la conversión manual.

using namespace std::chrono_literals;

/**
 * @class PidControllerNode
 * @brief Nodo de ROS 2 que implementa el controlador PID 6-DOF para el UUV.
 * Escucha una trayectoria y publica el estado simulado y las fuerzas.
 */
class PidControllerNode : public rclcpp::Node
{
public:
    PidControllerNode() : Node("pid_f_controller")
    {
        // --- 1. Declarar y Obtener Parámetros ---
        // (Usamos los valores por defecto del nodo ROS 1 original)
        this->declare_parameter("k_p", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        this->declare_parameter("k_i", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        this->declare_parameter("k_d", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        this->declare_parameter("init_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        this->declare_parameter("max_tau", std::vector<double>{127.0, 34.0, 118.0, 28.0, 9.6, 36.6});

        // Obtener los valores
        auto k_p_d = this->get_parameter("k_p").as_double_array();
        auto k_i_d = this->get_parameter("k_i").as_double_array();
        auto k_d_d = this->get_parameter("k_d").as_double_array();
        auto init_pose_d = this->get_parameter("init_pose").as_double_array();
        auto max_tau_d = this->get_parameter("max_tau").as_double_array();

        // --- 2. Convertir Parámetros a los Tipos de la Librería ---
        // La librería usa 'float', pero los parámetros de ROS 2 son 'double'
        std::vector<float> k_p_f(k_p_d.begin(), k_p_d.end());
        std::vector<float> k_i_f(k_i_d.begin(), k_i_d.end());
        std::vector<float> k_d_f(k_d_d.begin(), k_d_d.end());
        std::vector<float> init_pose_f(init_pose_d.begin(), init_pose_d.end());
        
        if (max_tau_d.size() != 6) {
            RCLCPP_FATAL(this->get_logger(), "El parámetro 'max_tau' DEBE tener 6 elementos.");
            rclcpp::shutdown();
            return;
        }
        std::copy(max_tau_d.begin(), max_tau_d.end(), max_tau_.begin());

        std::array<DOFControllerType_E, 6> types = {LINEAR_DOF, LINEAR_DOF, LINEAR_DOF, ANGULAR_DOF, ANGULAR_DOF, ANGULAR_DOF};

        // --- 3. Instanciar el Controlador/Modelo ---
        // (Usamos std::unique_ptr para manejar el objeto)
        model_ = std::make_unique<VTEC_U4_6DOF_PID>(SAMPLE_TIME_S, k_p_f, k_i_f, k_d_f, max_tau_, types);
        model_->setInitPose(init_pose_f);

        // --- 4. Crear Publishers ---
        // (Usando los nombres y tipos que definiste en tu script de Python)
        accel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("acceleration", 10);
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("velocity", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("pose", 10);
        thrust_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("thrust", 10);

        // --- 5. Crear Subscriber a la Trayectoria ---
        trajectory_sub_ = this->create_subscription<uuv_msgs::msg::Trayectory>(
            "trayectory", 10, std::bind(&PidControllerNode::trajectory_callback, this, std::placeholders::_1));

        // --- 6. Crear el Timer (el bucle principal) ---
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(SAMPLE_TIME_S),
            std::bind(&PidControllerNode::control_loop_callback, this));

        RCLCPP_INFO(this->get_logger(), "Nodo de control PID (C++) inicializado y listo.");
    }

private:
    /**
     * @brief Callback cuando se recibe una nueva trayectoria.
     */
    void trajectory_callback(const uuv_msgs::msg::Trayectory::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Nueva trayectoria recibida con %zu puntos.", msg->points.size());
        current_trajectory_ = msg->points;
        trajectory_index_ = 0; // Reiniciamos el índice al recibir una nueva trayectoria
    }

    /**
     * @brief Bucle principal de control, llamado por el timer.
     */
    void control_loop_callback()
    {
        // Si no hay trayectoria o ya se terminó, no hacer nada.
        if (current_trajectory_.empty() || trajectory_index_ >= current_trajectory_.size())
        {
            return;
        }

        // --- 1. Obtener el punto de referencia actual ---
        auto& current_point = current_trajectory_[trajectory_index_];

        // --- 2. Formatear las referencias para el controlador ---
        // (¡ADVERTENCIA! Ver nota abajo sobre 3-DOF vs 6-DOF)
        std::array<float, 6> chi1_d = {
            (float)current_point.position.x,
            (float)current_point.position.y,
            (float)current_point.position.z,
            0.0f, // Roll deseado (No está en tu mensaje)
            0.0f, // Pitch deseado (No está en tu mensaje)
            0.0f  // Yaw deseado (No está en tu mensaje)
        };
        std::array<float, 6> chi2_d = {
            (float)current_point.velocity.x,
            (float)current_point.velocity.y,
            (float)current_point.velocity.z,
            0.0f, // Roll-rate deseado (No está en tu mensaje)
            0.0f, // Pitch-rate deseado (No está en tu mensaje)
            0.0f  // Yaw-rate deseado (No está en tu mensaje)
        };
        std::array<float, 6> chi2_dot_d = {
            (float)current_point.acceleration.x,
            (float)current_point.acceleration.y,
            (float)current_point.acceleration.z,
            0.0f, // Roll-accel deseado (No está en tu mensaje)
            0.0f, // Pitch-accel deseado (No está en tu mensaje)
            0.0f  // Yaw-accel deseado (No está en tu mensaje)
        };

        // --- 3. Ejecutar el ciclo de simulación y control ---
        // (Esta es la misma lógica del main() de ROS 1)
        model_->calculateStates();
        model_->updateNonLinearFunctions();
        model_->updateReferences(chi1_d, chi2_d, chi2_dot_d);
        model_->calculateControlSignals();
        model_->updateControlSignals();

        // --- 4. Publicar los resultados ---
        // Publicar aceleración y velocidad (estos tipos coinciden)
        accel_pub_->publish(model_->accelerations_);
        vel_pub_->publish(model_->velocities_);

        // Convertir y publicar Pose (EtaPose -> Pose)
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = model_->eta_pose_.x;
        pose_msg.position.y = model_->eta_pose_.y;
        pose_msg.position.z = model_->eta_pose_.z;
        
        // Convertir Euler (phi, theta, psi) a Cuaternión
        tf2::Quaternion q;
        q.setRPY(model_->eta_pose_.phi, model_->eta_pose_.theta, model_->eta_pose_.psi);
        pose_msg.orientation.x = q.x();
        pose_msg.orientation.y = q.y();
        pose_msg.orientation.z = q.z();
        pose_msg.orientation.w = q.w();
        
        pose_pub_->publish(pose_msg);

        // Convertir y publicar Thrust (std::array<float, 6> -> Wrench)
        // El control calculado 'u_' está en PID6DOFLin, del cual hereda el modelo
        geometry_msgs::msg::Wrench wrench_msg;
        wrench_msg.force.x = model_->u_[0];
        wrench_msg.force.y = model_->u_[1];
        wrench_msg.force.z = model_->u_[2];
        wrench_msg.torque.x = model_->u_[3];
        wrench_msg.torque.y = model_->u_[4];
        wrench_msg.torque.z = model_->u_[5];
        thrust_pub_->publish(wrench_msg);

        // --- 5. Avanzar al siguiente punto de la trayectoria ---
        trajectory_index_++;
    }

    // --- Variables Miembro ---
    const double SAMPLE_TIME_S = 0.01;
    std::array<float, 6> max_tau_;

    // El objeto del controlador y modelo
    std::unique_ptr<VTEC_U4_6DOF_PID> model_;

    // ROS Comms
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<uuv_msgs::msg::Trayectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr accel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr thrust_pub_;

    // Estado de la trayectoria
    std::vector<uuv_msgs::msg::TrajectoryPoint> current_trajectory_;
    size_t trajectory_index_ = 0;
};

// --- Función Main ---
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PidControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}