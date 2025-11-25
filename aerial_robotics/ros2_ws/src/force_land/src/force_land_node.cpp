#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <cmath> // Necessario per std::abs

using namespace std::chrono_literals;

class ForceLand : public rclcpp::Node
{
public:
    // Inizializza i due stati di controllo richiesti: land_lock e landing_active
    ForceLand() : Node("force_land_monitor"), land_lock(false), landing_active(false) 
    {
        // Qualità del Servizio (QoS) per i dati del sensore/posizione
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Sottoscrizione 1: Altitudine
        subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, 
                                                                                    std::bind(&ForceLand::height_callback, this, std::placeholders::_1));
        
        // Sottoscrizione 2: Interruzione Pilota (Roll/Pitch)
        manual_sub_ = this->create_subscription<px4_msgs::msg::ManualControlSetpoint>("/fmu/out/manual_control_setpoint", qos, 
                                                                                    std::bind(&ForceLand::manual_callback, this, std::placeholders::_1));

        // Sottoscrizione 3: Atterraggio Completato
        land_detected_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected", qos, 
                                                                                         std::bind(&ForceLand::land_detected_callback, this, std::placeholders::_1));

        // Editore per inviare comandi al PX4
        publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        // Timer per inviare il comando di atterraggio in modo persistente (ogni 100ms)
        timer_ = this->create_wall_timer(100ms, std::bind(&ForceLand::activate_switch, this));
        
        RCLCPP_INFO(this->get_logger(), "ForceLand Monitor avviato. Soglia: 20 metri.");
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
    rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Stato di controllo
    bool land_lock;        // True se il pilota ha ripreso il controllo. Impedisce nuove attivazioni.
    bool landing_active;   // True se la soglia > 20m è stata raggiunta e la procedura è in corso/bloccata.

    // --- CALLBACKS ---
    
    // Callback 1: Gestione Altitudine (Attiva la procedura una tantum)
    void height_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
    {
        // Altitudine positiva
        float z_ = -msg->z;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Current drone height: %.2f meters", z_);
        
        // Attiva la procedura Land Forzato SOLO se l'altitudine > 20m E NON c'è il blocco attivo.
        if (z_ > 20.0 && !land_lock) 
        {
            // Imposta landing_active solo la prima volta che si supera la soglia.
            if (!landing_active) {
                 RCLCPP_INFO(this->get_logger(), "CONDIZIONE LANDING SODDISFATTA: Altitudine > 20m e Blocco OFF. Avvio Atterraggio Forzato.");
                 landing_active = true;
            }
        }
        return;
    }
    
    // Callback 2: Rileva l'uso degli stick e attiva il blocco
    void manual_callback(const px4_msgs::msg::ManualControlSetpoint::UniquePtr msg)
    {
        // Il blocco si attiva SOLO se:
        // 1. La procedura di Land è stata innescata (landing_active).
        // 2. Non è già stato bloccato (!land_lock).
        // 3. C'è un movimento significativo di Roll o Pitch (> 10%).
        if (landing_active && !land_lock && (std::abs(msg->roll) > 0.1f || std::abs(msg->pitch) > 0.1f)) 
        {
            land_lock = true;
            // land_lock = true impedisce al timer di continuare a inviare il comando di Land.
            RCLCPP_WARN(this->get_logger(), "Pilota ha ripreso il controllo! Blocco Land Forced ATTIVO. LANDING INTERROTTO.");
        }
    }

    // Callback 3: Resetta lo stato quando l'atterraggio è completato
    void land_detected_callback(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg)
    {
        // Resetta la procedura e il blocco solo quando il drone è a terra E una delle due condizioni era attiva.
        if (msg->landed && (land_lock || landing_active)) 
        {
            land_lock = false;
            landing_active = false; 
            RCLCPP_INFO(this->get_logger(), "Atterraggio completato. Blocco Land Forced RESETTATO.");
        }
    }

    // --- TIMER / COMANDI ---

    // La funzione del timer invia il comando di Land in modo persistente.
    void activate_switch()
    {
        // Invia il comando di Land ripetutamente solo se la procedura è attiva e non è stata bloccata dal pilota.
        if (landing_active && !land_lock) 
        {
            RCLCPP_ERROR(this->get_logger(), ">>> LANDING FORCED & PERSISTENT! Pilot must take control. <<<");
            
            auto command = px4_msgs::msg::VehicleCommand();
            
            command.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            command.param1 = 0.0; 
            command.param2 = 0.0; 
            command.param7 = 0.0; 
            
            command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
            command.target_system = 1;
            command.target_component = 1;
            command.source_system = 1;
            command.source_component = 1;
            command.confirmation = true;
            
            this->publisher_->publish(command);
        }
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting force_land node..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceLand>());
    rclcpp::shutdown();
    return 0;
}
