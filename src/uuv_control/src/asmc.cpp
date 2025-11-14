/** ----------------------------------------------------------------------------
 * @file: vtec_u4_asmc_node.cpp
 * @date: Nov 13, 2025 (Migrated to ROS 2)
 * @author: Sebas Mtz (Original), Gemini (Migration)
 * @email: sebas.martp@gmail.com
 * * @brief: ROS 2 adaptive sliding mode control node for the VTec U4.
 * -----------------------------------------------------------------------------
 **/

#include "rclcpp/rclcpp.hpp"
#include "6dof_asmc.hpp" // ASUNCIÓN: Este header fue migrado a ROS 2

// --- INCLUDES DE MENSAJES ROS 2 ---
// Asegúrate de que tu paquete 'vanttec_msgs' haya sido migrado a ROS 2
// y genere estos headers en 'install/vanttec_msgs/include/vanttec_msgs/msg/'
#include "vanttec_msgs/msg/thrust_control.hpp"
#include "vanttec_msgs/msg/eta_pose.hpp"

// !!! ATENCIÓN !!!
// El tipo de mensaje para '/non_linear_functions' no estaba en el código original.
// He asumido un tipo llamado 'UuvDynamics'. Debes reemplazarlo por el tipo correcto.
#include "vanttec_msgs/msg/uuv_dynamics.hpp" // <-- REEMPLAZAR ESTO

#include <vector>
#include <memory>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

// --- Alias para los tipos de mensajes (buena práctica) ---
using ThrustMsg = vanttec_msgs::msg::ThrustControl;
using PoseMsg = vanttec_msgs::msg::EtaPose;
// Reemplaza esto con tu tipo de mensaje real
using DynamicsMsg = vanttec_msgs::msg::UuvDynamics; // <-- REEMPLAZAR ESTO


// --- CLASE DEL NODO ---
class AsmcNode : public rclcpp::Node
{
public:
    AsmcNode() : Node("uuv_control_node")
    {
        RCLCPP_INFO(this->get_logger(), "Iniciando nodo ASMC...");

        // --- 1. Cargar Parámetros ---
        // Los vectores de parámetros se declaran como std::vector<double> en ROS 2
        // Los convertiremos a float para la clase controladora
        lambda_v_ = get_float_vector_param("lambda");
        k1_init_v_ = get_float_vector_param("K1_init");
        k2_v_ = get_float_vector_param("K2");
        k_alpha_v_ = get_float_vector_param("K_alpha");
        k_min_v_ = get_float_vector_param("K_min");
        mu_v_ = get_float_vector_param("mu");

        // --- 2. Inicializar Controlador ---
        const float SAMPLE_TIME_S = 0.01;
        
        system_controller_ = std::make_unique<ASMC6DOF>(
            SAMPLE_TIME_S,
            lambda_v_.data(),
            k2_v_.data(),
            k_alpha_v_.data(),
            k1_init_v_.data(),
            k_min_v_.data(),
            mu_v_.data()
        );

        float MAX_TAU[6] = {127, 34, 118, 28, 9.6, 36.6};
        system_controller_->setMaxThrust(MAX_TAU);

        // --- 3. Crear Publicador ---
        // El '1' de ROS 1 (queue size) se reemplaza por '10' (QoS depth)
        uuv_thrust_pub_ = this->create_publisher<ThrustMsg>("/uuv_control/uuv_control_node/thrust", 10);

        // --- 4. Crear Suscriptores ---
        // Usamos std::bind para enlazar los callbacks a la instancia del controlador
        
        // !! ASUNCIÓN CRÍTICA !!
        // Se asume que los métodos en ASMC6DOF han sido actualizados para
        // aceptar un 'const std::shared_ptr<MsgType>' como argumento.
        // Ej: void updateDynamics(const DynamicsMsg::SharedPtr msg)
        
        uuv_dynamics_sub_ = this->create_subscription<DynamicsMsg>(
            "/uuv_simulation/dynamic_model/non_linear_functions", 10,
            std::bind(&ASMC6DOF::updateDynamics, system_controller_.get(), std::placeholders::_1)
        );

        uuv_pose_sub_ = this->create_subscription<PoseMsg>(
            "/uuv_simulation/dynamic_model/eta_pose", 10,
            std::bind(&ASMC6DOF::updatePose, system_controller_.get(), std::placeholders::_1)
        );

        // Asumiendo que el set_point también usa el mensaje EtaPose
        uuv_set_point_sub_ = this->create_subscription<PoseMsg>(
            "/uuv_control/uuv_control_node/set_point", 10,
            std::bind(&ASMC6DOF::updateSetPoints, system_controller_.get(), std::placeholders::_1)
        );

        // --- 5. Crear Timer para el Bucle de Control ---
        // Esto reemplaza el 'while(ros::ok())' y 'ros::Rate'
        auto timer_period = std::chrono::duration<double>(SAMPLE_TIME_S);
        timer_ = this->create_wall_timer(timer_period, std::bind(&AsmcNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Nodo ASMC inicializado y listo.");
    }

private:
    /**
     * @brief Función de ayuda para declarar y obtener parámetros de tipo vector<float>.
     * * ROS 2 maneja los arrays de flotantes como 'double' por defecto.
     * Esta función declara el parámetro, lo obtiene como vector<double>
     * y lo convierte a vector<float>.
     */
    std::vector<float> get_float_vector_param(const std::string& param_name)
    {
        // 1. Declarar el parámetro con un vector<double> vacío por defecto
        this->declare_parameter(param_name, std::vector<double>{});
        
        // 2. Obtener el parámetro como vector<double>
        std::vector<double> vec_double = this->get_parameter(param_name).as_double_array();
        
        // 3. Convertir a vector<float>
        std::vector<float> vec_float(vec_double.begin(), vec_double.end());
        
        // Advertencia si el parámetro no se cargó
        if (vec_float.empty()) {
            RCLCPP_WARN(this->get_logger(), "Parámetro '%s' no encontrado o está vacío.", param_name.c_str());
        }
        
        return vec_float;
    }

    /**
     * @brief Callback del timer, se ejecuta a la frecuencia de muestreo (100Hz).
     * * Reemplaza el contenido del bucle 'while(ros::ok())'.
     * No se necesita 'ros::spinOnce()' aquí.
     */
    void timer_callback()
    {
        // 1. Calcular la acción de control
        // Los datos de los suscriptores se actualizan automáticamente en 'system_controller_'
        // gracias a que rclcpp::spin() maneja los callbacks en segundo plano.
        system_controller_->calculateManipulation();
       
        // 2. Publicar el empuje
        // ASUNCIÓN: 'system_controller_->thrust_' es del tipo 'vanttec_msgs::msg::ThrustControl'
        uuv_thrust_pub_->publish(system_controller_->thrust_);
    }

    // --- Variables Miembro ---

    // Puntero al controlador
    std::unique_ptr<ASMC6DOF> system_controller_;

    // Vectores para guardar los parámetros (necesarios para que .data() siga siendo válido)
    std::vector<float> lambda_v_;
    std::vector<float> k1_init_v_;
    std::vector<float> k2_v_;
    std::vector<float> k_alpha_v_;
    std::vector<float> k_min_v_;
    std::vector<float> mu_v_;

    // Componentes de ROS 2
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ThrustMsg>::SharedPtr uuv_thrust_pub_;
    rclcpp::Subscription<DynamicsMsg>::SharedPtr uuv_dynamics_sub_; // <-- REVISAR TIPO
    rclcpp::Subscription<PoseMsg>::SharedPtr uuv_pose_sub_;
    rclcpp::Subscription<PoseMsg>::SharedPtr uuv_set_point_sub_;
};


// --- FUNCIÓN MAIN ---
int main(int argc, char **argv)
{
    // 1. Inicializar ROS 2
    rclcpp::init(argc, argv);
    
    // 2. Crear una instancia del nodo
    auto asmc_node = std::make_shared<AsmcNode>();
    
    // 3. Hacer "spin" (girar) el nodo para que procese callbacks (timers, subs, etc.)
    // rclcpp::spin() bloqueará este hilo hasta que el nodo se apague.
    rclcpp::spin(asmc_node);
    
    // 4. Apagar ROS 2
    rclcpp::shutdown();
    return 0;
}