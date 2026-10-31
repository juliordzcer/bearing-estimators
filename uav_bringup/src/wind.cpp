// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <random>
#include <cmath>

using namespace std::chrono_literals;

class DeterministicWindPublisher : public rclcpp::Node
{
public:
    DeterministicWindPublisher() : Node("deterministic_wind_publisher"),
                                   gen_(rd_()),
                                   prob_dist_(0.0, 1.0),
                                   noise_dist_(-1.0, 1.0)
    {
        // Crear publishers
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/wind/velocity", 10);
        enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/wind/enable", 10);

        // ---------------------------------------------------------
        // 1. V_base: Componente media (Mean Velocity) [m/s]
        //    Representa viento sostenido (e.g. corriente de aire del laboratorio).
        //    El viento PUEDE ir en cualquier dirección — sin cota inferior.
        // ---------------------------------------------------------
        base_x_ = 0.5;
        base_y_ = 0.2;
        base_z_ = 0.0;

        // Ruido estocástico de alta frecuencia (turbulencia)
        noise_amp_ = 0.04;  // [m/s] — amplitud de turbulencia

        // ---------------------------------------------------------
        // 2. V_osc: Oscilaciones Armónicas
        // ---------------------------------------------------------
        // Ecuaciones:
        // x: 0.15 * sin(0.16 * pi * t) -> Freq = 0.08 Hz
        // y: 0.08 * cos(0.24 * pi * t) -> Freq = 0.12 Hz
        // z: 0.02 * sin(0.10 * pi * t + 1) -> Freq = 0.05 Hz
        
        osc_amp_x_ = 0.15;  osc_freq_x_ = 0.08;
        osc_amp_y_ = 0.08;  osc_freq_y_ = 0.12;
        osc_amp_z_ = 0.02;  osc_freq_z_ = 0.05;

        // ---------------------------------------------------------
        // 3. V_raf: Ráfagas (Gusts)
        // ---------------------------------------------------------
        gust_amp_ = 0.2;      // Magnitud
        gust_duration_ = 1.5; // Duración (s)
        gust_prob_ = 0.03;    // Probabilidad de inicio por ciclo
        in_gust_ = false;

        // Habilitar
        auto enable_msg = std_msgs::msg::Bool();
        enable_msg.data = true;
        enable_pub_->publish(enable_msg);

        RCLCPP_INFO(this->get_logger(), "Deterministic Wind (Base + Osc + Gust) Started");

        // Timer a 20 Hz (dt = 0.05s). create_timer (not create_wall_timer)
        // uses the node clock, so with use_sim_time it ticks on simulated
        // time instead of wall time.
        timer_ = this->create_timer(
            50ms, std::bind(&DeterministicWindPublisher::timer_callback, this));
        
        start_time_ = this->now();
    }

private:
    void timer_callback()
    {
        auto current_time = this->now();
        double t = (current_time - start_time_).seconds(); // Tiempo total 't'

        // =========================================================
        // CALCULO DE V_osc (Oscilaciones)
        // =========================================================
        // Nota: 2*PI*f*t es equivalente a (w*t) donde w = 2*PI*f
        // Para x: w = 0.16*PI -> f = 0.08
        double v_osc_x = osc_amp_x_ * sin(2 * M_PI * osc_freq_x_ * t);
        double v_osc_y = osc_amp_y_ * cos(2 * M_PI * osc_freq_y_ * t);
        double v_osc_z = osc_amp_z_ * sin(2 * M_PI * osc_freq_z_ * t + 1.0);

        // =========================================================
        // CALCULO DE V_raf (Ráfagas)
        // =========================================================
        double v_raf_x = 0.0;
        double v_raf_y = 0.0;
        double v_raf_z = 0.0;

        // Inicio aleatorio de la ráfaga
        if (!in_gust_ && prob_dist_(gen_) < gust_prob_) {
            in_gust_ = true;
            gust_start_time_ = t;
            gust_theta_ = prob_dist_(gen_) * 2 * M_PI; // Theta ~ U(0, 2pi)
        }

        if (in_gust_) {
            double tau = t - gust_start_time_; // Tiempo transcurrido de ráfaga
            
            if (tau <= gust_duration_) {
                // Perfil sinusoidal: sin(pi * tau / 1.5)
                double intensity = sin((tau / gust_duration_) * M_PI);
                
                v_raf_x = gust_amp_ * cos(gust_theta_) * intensity;
                v_raf_y = gust_amp_ * sin(gust_theta_) * intensity;
                v_raf_z = gust_amp_ * 0.3 * intensity; // Escalado vertical 0.3
            } else {
                in_gust_ = false;
            }
        }

        // =========================================================
        // Turbulencia: ruido de alta frecuencia (U[-1,1] * amp)
        // Representa fluctuaciones de microescala, simétricas.
        // =========================================================
        double eta_x = noise_dist_(gen_) * noise_amp_;
        double eta_y = noise_dist_(gen_) * noise_amp_;
        double eta_z = noise_dist_(gen_) * noise_amp_ * 0.5;

        // =========================================================
        // SUMATORIA: Δ = V_base + V_osc + V_raf + η
        // Consistente con Assumption 1: ||Δ|| ≤ D, ||Δ̇|| ≤ D̄.
        // Sin cotas inferiores — el viento puede revertir dirección.
        // =========================================================
        double total_x = base_x_ + v_osc_x + v_raf_x + eta_x;
        double total_y = base_y_ + v_osc_y + v_raf_y + eta_y;
        double total_z = base_z_ + v_osc_z + v_raf_z + eta_z;

        // Publicar
        auto vel_msg = geometry_msgs::msg::Vector3();
        vel_msg.x = total_x;
        vel_msg.y = total_y;
        vel_msg.z = total_z;

        vel_pub_->publish(vel_msg);
    }

    // Variables miembro
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    
    // Generación de números aleatorios (Solo para dirección/tiempo de ráfaga)
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> prob_dist_;
    std::uniform_real_distribution<> noise_dist_;

    // Parámetros Base
    double base_x_, base_y_, base_z_;
    double noise_amp_;
    
    // Parámetros Oscilación
    double osc_amp_x_, osc_amp_y_, osc_amp_z_;
    double osc_freq_x_, osc_freq_y_, osc_freq_z_;

    // Parámetros Ráfaga
    double gust_amp_, gust_duration_, gust_prob_;
    bool in_gust_;
    double gust_start_time_, gust_theta_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeterministicWindPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/vector3.hpp>
// #include <std_msgs/msg/bool.hpp>
// #include <chrono>
// #include <random>
// #include <cmath>

// using namespace std::chrono_literals;

// class ComplexWindPublisher : public rclcpp::Node
// {
// public:
//     ComplexWindPublisher() : Node("complex_wind_publisher"),
//                             gen_(rd_()),
//                             dist_(-1.0, 1.0),       // Distribución para el ruido n
//                             prob_dist_(0.0, 1.0)    // Distribución para probabilidad de ráfaga
//     {
//         // Crear publishers
//         vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/wind/velocity", 10);
//         enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/wind/enable", 10);
        
//         // Valores base (Offset sobre el cual se aplican las oscilaciones)
//         base_x_ = 0.5;
//         base_y_ = 0.2;
//         base_z_ = 0.05;
        
//         // ---------------------------------------------------------
//         // Parámetros para v_osc (Oscilaciones Armónicas)
//         // ---------------------------------------------------------
//         sine_amp_x_ = 0.15;  // Amplitud Ax
//         sine_amp_y_ = 0.08;  // Amplitud Ay
//         sine_amp_z_ = 0.02;  // Amplitud Az
//         sine_freq_x_ = 0.08; // Frecuencia fx (0.16pi rad/s / 2pi)
//         sine_freq_y_ = 0.12; // Frecuencia fy (0.24pi rad/s / 2pi)
//         sine_freq_z_ = 0.05; // Frecuencia fz (0.1pi rad/s / 2pi)
        
//         // ---------------------------------------------------------
//         // Parámetro para eta_net (Ruido Estocástico)
//         // ---------------------------------------------------------
//         noise_amp_ = 0.05;   // Amplitud del ruido sigma
        
//         // ---------------------------------------------------------
//         // Parámetros para v_raf (Ráfagas Esporádicas)
//         // ---------------------------------------------------------
//         gust_amp_ = 0.2;      // Magnitud máxima de la ráfaga
//         gust_duration_ = 1.5; // Duración T = 1.5s
//         gust_prob_ = 0.03;    // Probabilidad de ocurrencia (determina t_inicio)
//         in_gust_ = false;
        
//         // Habilitar viento
//         auto enable_msg = std_msgs::msg::Bool();
//         enable_msg.data = true;
//         enable_pub_->publish(enable_msg);
        
//         RCLCPP_INFO(this->get_logger(), "Complex wind pattern enabled");
        
//         // Temporizador (20 Hz -> dt = 0.05s)
//         timer_ = this->create_wall_timer(
//             50ms, std::bind(&ComplexWindPublisher::timer_callback, this));
        
//         start_time_ = this->now();
//     }

// private:
//     void timer_callback()
//     {
//         auto current_time = this->now();
//         double elapsed = (current_time - start_time_).seconds(); // Variable t
        
//         // =========================================================
//         // 1. CÁLCULO DE v_osc (Componente Oscilatoria)
//         // Ecuación: v_osc = [Ax*sin(wt), Ay*cos(wt), Az*sin(wt+phi)]
//         // =========================================================
//         double sine_x = sine_amp_x_ * sin(2 * M_PI * sine_freq_x_ * elapsed);
//         double sine_y = sine_amp_y_ * cos(2 * M_PI * sine_freq_y_ * elapsed);
//         double sine_z = sine_amp_z_ * sin(2 * M_PI * sine_freq_z_ * elapsed + 1.0); // +1.0 es phi
        
//         // =========================================================
//         // 2. CÁLCULO DE eta_net (Componente Estocástica/Ruido)
//         // Ecuación: eta ~ N(0, sigma^2)
//         // =========================================================
//         double noise_x = dist_(gen_) * noise_amp_;
//         double noise_y = dist_(gen_) * noise_amp_;
//         double noise_z = dist_(gen_) * noise_amp_ * 0.5; // Ruido vertical atenuado
        
//         // =========================================================
//         // 3. CÁLCULO DE v_raf (Componente de Ráfagas)
//         // =========================================================
//         double gust_x = 0.0, gust_y = 0.0, gust_z = 0.0;
        
//         // Determinar t_inicio de forma aleatoria
//         if (!in_gust_ && prob_dist_(gen_) < gust_prob_) {
//             in_gust_ = true;
//             gust_start_time_ = elapsed;                 // Marca t_inicio
//             gust_direction_ = prob_dist_(gen_) * 2 * M_PI; // Dirección theta ~ U(0, 2pi)
//         }
        
//         if (in_gust_) {
//             double gust_time = elapsed - gust_start_time_; // Variable tau = t - t_inicio
            
//             // Ecuación: sin(pi * tau / 1.5)
//             if (gust_time < gust_duration_) {
//                 // Perfil de campana (seno) para la intensidad
//                 double intensity = sin((gust_time / gust_duration_) * M_PI);
                
//                 // Proyección del vector ráfaga según theta
//                 gust_x = gust_amp_ * cos(gust_direction_) * intensity;
//                 gust_y = gust_amp_ * sin(gust_direction_) * intensity;
//                 gust_z = gust_amp_ * 0.3 * intensity; 
//             } else {
//                 in_gust_ = false; // Fin del evento transitorio
//             }
//         }
        
//         // =========================================================
//         // 4. SUMATORIA TOTAL (Delta)
//         // Ecuación: Delta = v_base + v_osc + v_raf + eta
//         // =========================================================
//         double total_x = base_x_ + sine_x + noise_x + gust_x;
//         double total_y = base_y_ + sine_y + noise_y + gust_y;
//         double total_z = base_z_ + sine_z + noise_z + gust_z;
        
//         // =========================================================
//         // 5. ACOTACIÓN INFERIOR (Lower Bound Constraint)
//         // Evita singularidades numéricas (velocidad cero)
//         // =========================================================
//         total_x = std::max(total_x, 0.1);   // Mínimo 0.1 m/s
//         total_y = std::max(total_y, 0.05);  // Mínimo 0.05 m/s
//         total_z = std::max(total_z, 0.01);  // Mínimo 0.01 m/s
        
//         // Publicar
//         auto vel_msg = geometry_msgs::msg::Vector3();
//         vel_msg.x = total_x;
//         vel_msg.y = total_y;
//         vel_msg.z = total_z;
        
//         vel_pub_->publish(vel_msg);
        
//         // Log cada 10 segundos
//         static auto last_log_time = this->now();
//         if ((current_time - last_log_time).seconds() >= 10.0) {
//             // RCLCPP_INFO(this->get_logger(),
//             //     "Wind: X=%.3f, Y=%.3f, Z=%.3f | Gust: %s",
//             //     total_x, total_y, total_z,
//             //     in_gust_ ? "YES" : "NO");
//             last_log_time = current_time;
//         }
//     }
    
//     // ... (Declaración de variables miembro igual que antes) ...
//     rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vel_pub_;
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Time start_time_;
//     std::random_device rd_;
//     std::mt19937 gen_;
//     std::uniform_real_distribution<> dist_;
//     std::uniform_real_distribution<> prob_dist_;
//     double base_x_, base_y_, base_z_;
//     double sine_amp_x_, sine_amp_y_, sine_amp_z_;
//     double sine_freq_x_, sine_freq_y_, sine_freq_z_;
//     double noise_amp_;
//     double gust_amp_, gust_duration_, gust_prob_;
//     bool in_gust_;
//     double gust_start_time_, gust_direction_;
// };

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ComplexWindPublisher>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
