#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <offboard_rl/utils.h> // Assumo che offboard_rl/utils.h contenga Vector4d, VectorXd e la funzione quatToRpy

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

using Vector4d = Eigen::Vector4d;
using VectorXd = Eigen::VectorXd;

class GoToPoint : public rclcpp::Node
{
    public:
    GoToPoint() : Node("go_to_point")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Subscriptions
        local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",
        qos, std::bind(&GoToPoint::vehicle_local_position_callback, this, std::placeholders::_1));
        attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude",
        qos, std::bind(&GoToPoint::vehicle_attitude_callback, this, std::placeholders::_1));
        
        // Publishers
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        // Timers
        timer_offboard_ = this->create_wall_timer(100ms, std::bind(&GoToPoint::activate_offboard, this));
        timer_trajectory_publish_ = this->create_wall_timer(20ms, std::bind(&GoToPoint::publish_trajectory_setpoint, this));
    }

    private:
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subscription_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    rclcpp::TimerBase::SharedPtr timer_offboard_;
    rclcpp::TimerBase::SharedPtr timer_trajectory_publish_;

	    // Definizione dei 8 Waypoint in NED (Z negativo = Altezza positiva)
	const std::vector<Vector4d> WAYPOINTS = {
	    {0.0, 0.0, -10.0, 0.0}, 
	    
	    {15.0, 20.0, -10.0, 0.5}, 
	    
	    {40.0, 0.0, -10.0, 1.57}, 
	    
	    {0.0, -40.0, -10.0, 3.14}, 
	    
	    {-40.0, 0.0, -10.0, -1.57}, 
	    
        {-15.0, 20.0, -10.0, 0.0}, 

        {0.0, 0.0, -10.0, 0.0},
	    
	    {0.0, 0.0, -10.0, 0.0} 
	};

	// Tempo T per ogni segmento (mantenuti i tempi veloci per V_min alta)
	const std::vector<double> SEGMENT_TIMES = {5.0, 5.0, 8.0, 8.0, 5.0, 5.0, 5.0};

    int current_segment_index{0}; 
    double t{0.0}; 
    double T_segment{0.0}; 

    Vector4d pos_i_segment; 
    Vector4d pos_f_segment; 

    bool offboard_active{false};
    bool trajectory_running{false}; 
    bool trajectory_computed{false};
    bool landing_sent{false}; 
    double initial_yaw_{0.0}; // Variabile per salvare lo Yaw iniziale
    
    Eigen::Vector<double, 6> x; 

    VehicleLocalPosition current_position_{};
    VehicleAttitude current_attitude_{};
    double offboard_counter{0}; 


    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        current_position_ = *msg;
    }

    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        current_attitude_ = *msg;
    }

    // ----------------------------------------------------
    // Trajectory Computation (Polinomio 5° Ordine)
    // ----------------------------------------------------
    
    TrajectorySetpoint compute_trajectory_setpoint(double t, const Vector4d& pos_i, const Vector4d& pos_f, double T)
    {
        // 1. Calcola il vettore errore e la distanza euclidea (s_f)
        Vector4d e = pos_f - pos_i;
        e(3) = utilities::angleError(pos_f(3), pos_i(3)); // Errore di Yaw gestito
        double s_f = e.head<3>().norm(); // Usiamo solo X, Y, Z per la distanza spaziale

        // 2. Calcola i coefficienti solo una volta all'inizio di ogni segmento
        if (!trajectory_computed)
        {
            RCLCPP_INFO(this->get_logger(), "Segmento %d: Calcolo Traiettoria (T=%.2f s)", current_segment_index, T);
            
            VectorXd b(6);
            Eigen::Matrix<double, 6, 6> A;

            b << 0.0, 0.0, 0.0, s_f, 0.0, 0.0;
            
            A << 0, 0, 0, 0, 0, 1, 
                  0, 0, 0, 0, 1, 0, 
                  0, 0, 0, 1, 0, 0, 
                  pow(T,5), pow(T,4), pow(T,3), pow(T,2), T, 1, 
                  5*pow(T,4), 4*pow(T,3), 3*pow(T,2), 2*T, 1, 0, 
                  20*pow(T,3), 12*pow(T,2), 6*T, 2, 0, 0; 

            x = A.inverse() * b;
            trajectory_computed = true;
        }

        // 3. Calcola Posizione (s), Velocità (s_d) e Accelerazione (s_dd)
        double s, s_d, s_dd;
        
        s = x(0) * std::pow(t, 5.0) + x(1) * std::pow(t, 4.0) + x(2) * std::pow(t, 3.0) + x(3) * std::pow(t, 2.0) + x(4) * t + x(5);
        s_d = 5.0 * x(0) * std::pow(t, 4.0) + 4.0 * x(1) * std::pow(t, 3.0) + 3.0 * x(2) * std::pow(t, 2.0) + 2.0 * x(3) * t + x(4);
        s_dd = 20.0 * x(0) * std::pow(t, 3.0) + 12.0 * x(1) * std::pow(t, 2.0) + 6.0 * x(2) * t + 2.0 * x(3); 

        // 4. Mappa i valori scalari sul vettore 4D (x, y, z, yaw)
        Vector4d ref_traj_pos, ref_traj_vel, ref_traj_acc;
        
        if (s_f > 1e-6) { // Evita divisione per zero
              // Calcolo per X, Y, Z
              ref_traj_pos.head<3>() = pos_i.head<3>() + s * e.head<3>() / s_f;
              ref_traj_vel.head<3>() = s_d * e.head<3>() / s_f;
              ref_traj_acc.head<3>() = s_dd * e.head<3>() / s_f;
              
              // Yaw non è più calcolato dal polinomio, ma è preso dal waypoint finale (solo come fallback)
              ref_traj_pos(3) = pos_f(3); 
              
        } else {
              ref_traj_pos = pos_i;
              ref_traj_vel.setZero();
              ref_traj_acc.setZero();
        }

        // 5. Compila il messaggio TrajectorySetpoint
        TrajectorySetpoint msg{};
        msg.position = {float(ref_traj_pos(0)), float(ref_traj_pos(1)), float(ref_traj_pos(2))};
        msg.velocity = {float(ref_traj_vel(0)), float(ref_traj_vel(1)), float(ref_traj_vel(2))};
        msg.acceleration = {float(ref_traj_acc(0)), float(ref_traj_acc(1)), float(ref_traj_acc(2))};
        
        // Calcola la velocità orizzontale istantanea
        double horizontal_speed = std::sqrt(std::pow(ref_traj_vel(0), 2) + std::pow(ref_traj_vel(1), 2));
        
        if (horizontal_speed > 0.1) 
        {
            // Calcola Yaw = atan2(Vy, Vx) in radianti
            msg.yaw = float(std::atan2(ref_traj_vel(1), ref_traj_vel(0)));
        } 
        else 
        {
            // Se la velocità orizzontale è trascurabile (es. hovering o punto di arrivo), 
            // usa lo Yaw pianificato per il waypoint finale (pos_f) come fallback, 
            // o lo Yaw iniziale se è l'ultimo segmento.
            msg.yaw = float(pos_f(3)); 
        }

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        return msg;
    }
    
    void activate_offboard()
    {
        if (offboard_counter == 10) {
            
            RCLCPP_INFO(this->get_logger(), "Attivazione Offboard e Armamento...");

            // 1. Arm the vehicle
            VehicleCommand arm_msg{};
            arm_msg.param1 = 1.0;
            arm_msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
            arm_msg.target_system = 1;
            arm_msg.target_component = 1;
            arm_msg.source_system = 1;
            arm_msg.source_component = 1;
            arm_msg.from_external = true;
            arm_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_command_publisher_->publish(arm_msg);
            
            // 2. Change to Offboard mode
            VehicleCommand mode_msg{};
            mode_msg.param1 = 1; 
            mode_msg.param2 = 6; // Custom mode: 6 = Offboard
            mode_msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
            mode_msg.target_system = 1;
            mode_msg.target_component = 1;
            mode_msg.source_system = 1;
            mode_msg.source_component = 1;
            mode_msg.from_external = true;
            mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_command_publisher_->publish(mode_msg);

            // 3. Inizializzazione del PRIMO Segmento (Decollo/WP 0 -> WP 1)
            pos_i_segment(0) = current_position_.x;
            pos_i_segment(1) = current_position_.y;
            pos_i_segment(2) = current_position_.z; 
            
            auto rpy = utilities::quatToRpy( Vector4d( current_attitude_.q[0], current_attitude_.q[1], current_attitude_.q[2], current_attitude_.q[3] ) );
            pos_i_segment(3) = rpy[2]; // Yaw attuale (Yaw di partenza)
            
            initial_yaw_ = rpy[2]; 

            current_segment_index = 0;
            pos_f_segment = WAYPOINTS[current_segment_index + 1]; 
            T_segment = SEGMENT_TIMES[current_segment_index];

            trajectory_computed = false; 
            trajectory_running = true;
            offboard_active = true;
            
            RCLCPP_INFO(this->get_logger(), "Inizio missione: Partenza da (%.1f, %.1f, %.1f) con YAW iniziale %.2f , IL DRONE EFFETTUERA' UN CUORE !",
                        pos_i_segment(0), pos_i_segment(1), pos_i_segment(2), initial_yaw_);
        }

        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = true; 
        msg.acceleration = true; 
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);    

        if (offboard_counter < 11) offboard_counter++;
    }

    void publish_trajectory_setpoint()
    {
        if (!offboard_active) {
            return;
        }

        if (!trajectory_running && !landing_sent) {
            RCLCPP_INFO(this->get_logger(), "Missione Multi-Waypoint Completata. Invio comando LAND. Atterraggio con Yaw: %.2f", initial_yaw_);
            
            VehicleCommand land_msg{};
            land_msg.command = VehicleCommand::VEHICLE_CMD_NAV_LAND;
            land_msg.target_system = 1;
            land_msg.target_component = 1;
            land_msg.source_system = 1;
            land_msg.source_component = 1;
            land_msg.from_external = true;
            land_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_command_publisher_->publish(land_msg);

            landing_sent = true; 
            return;
        }
        
        if (!trajectory_running && landing_sent) {
            return;
        }

        double dt = 1/50.0; 
        
        // 1. Calcola il setpoint
        TrajectorySetpoint msg{compute_trajectory_setpoint(t, pos_i_segment, pos_f_segment, T_segment)};
        trajectory_setpoint_publisher_->publish(msg);
        
        t += dt;

        // 2. Transizione al Segmento Successivo
        if (t >= T_segment) {
            
            // Verifichiamo se ci sono ancora segmenti da iniziare. SEGMENT_TIMES.size() è 7 (Indici 0-6)
            if (current_segment_index < SEGMENT_TIMES.size()) {
                
                // Se siamo all'ultimo segmento (indice 6), è stato completato.
                if (current_segment_index == SEGMENT_TIMES.size() - 1) { 
                    RCLCPP_INFO(this->get_logger(), "Ultimo Segmento (6) completato. Termino Traiettoria A FORMA DI CUORE");
                    trajectory_running = false; // FINITO! Triggera LAND.
                    return;
                }
                
                // Transizione al nuovo segmento (Segmenti 0, 1, ..., 5)
                current_segment_index++;
                
                RCLCPP_INFO(this->get_logger(), "Segmento %d completato. Passaggio a WP %d.", 
                                current_segment_index - 1, current_segment_index + 1);
                
                pos_i_segment = pos_f_segment; 
                pos_f_segment = WAYPOINTS[current_segment_index + 1]; 
                T_segment = SEGMENT_TIMES[current_segment_index];

                if (current_segment_index == SEGMENT_TIMES.size() - 1) { // L'indice 6
                    RCLCPP_INFO(this->get_logger(), "IMPOSTAZIONE FINALE: Imposto Yaw del WP 7 a %.2f (Yaw Iniziale).", initial_yaw_);
                    pos_f_segment(3) = initial_yaw_; 
                }
                
                t = 0.0;
                trajectory_computed = false; 

                RCLCPP_INFO(this->get_logger(), "Inizio Segmento %d: Target (%.1f, %.1f, %.1f) in %.1fs",
                            current_segment_index, pos_f_segment(0), pos_f_segment(1), pos_f_segment(2), T_segment);
            }
        }
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting GoToPoint Multi-Waypoint Trajectory Planner..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoToPoint>());
    rclcpp::shutdown();
    return 0;
}
