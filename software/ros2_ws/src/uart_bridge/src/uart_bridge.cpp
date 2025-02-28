#include <chrono>
#include <memory>
#include <vector>
#include <sstream>
#include <cstdint>
#include <iomanip>
#include <string>
#include <cstdio> // Pour sprintf
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial/serial.h>
#include "uart_commands.h" // Doit définir, par exemple, UART_CMD_STRATEGY

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

// Pour l'exemple, définition d'énumérations et structure pour la stratégie
enum Color
{
    RED = 0,
    BLUE = 1
};
enum Zone
{
    ZONE_A = 0,
    ZONE_B = 1,
    ZONE_C = 2
};

struct Strategy
{
    int color;
    int teamzone;
    int enemyzone;
};

class UARTBridgeNode : public rclcpp::Node
{
public:
    UARTBridgeNode() : Node("uart_bridge_node_cpp")
    {
        try
        {
            serial_.setPort("/dev/ttyTHS1");
            serial_.setBaudrate(115200);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
            serial_.setTimeout(timeout);
            serial_.open();
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Impossible d'ouvrir le port série /dev/ttyTHS1");
            return;
        }
        if (serial_.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "Port série /dev/ttyTHS1 ouvert");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Port série /dev/ttyTHS1 non ouvert");
            return;
        }

        strategy_pub_ = this->create_publisher<std_msgs::msg::String>("/strategy", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&UARTBridgeNode::read_uart, this));
        timeout_timer_ = this->create_wall_timer(300ms, std::bind(&UARTBridgeNode::check_rx_timeout, this));
        // Timer pour envoyer périodiquement une stratégie (exemple toutes les 5 secondes)
        strategy_timer_ = this->create_wall_timer(5s, std::bind(&UARTBridgeNode::apply_strategy_callback, this));

        last_byte_time_ = Clock::now();
    }

    ~UARTBridgeNode()
    {
        if (serial_.isOpen())
        {
            serial_.close();
        }
    }

    // Fonction d'encodage et d'envoi d'une trame UART
    void send_uart_message(uint16_t msg_function, const std::vector<uint8_t> &payload)
    {
        uint16_t payload_length = payload.size();
        uint16_t total_length = 1 + 2 + 2 + payload_length + 1;
        std::vector<uint8_t> message(total_length);
        uint16_t pos = 0;

        // En-tête
        message[pos++] = HEADER;

        // Fonction (MSB puis LSB)
        message[pos++] = (msg_function >> 8) & 0xFF;
        message[pos++] = msg_function & 0xFF;

        // Longueur du payload (MSB puis LSB)
        message[pos++] = (payload_length >> 8) & 0xFF;
        message[pos++] = payload_length & 0xFF;

        // Payload
        for (auto byte : payload)
        {
            message[pos++] = byte;
        }

        // Calcul du checksum (initialisé par HEADER puis XOR sur tous les octets)
        uint8_t checksum = HEADER;
        checksum ^= (msg_function >> 8) & 0xFF;
        checksum ^= (msg_function & 0xFF);
        checksum ^= (payload_length >> 8) & 0xFF;
        checksum ^= (payload_length & 0xFF);
        for (auto byte : payload)
        {
            checksum ^= byte;
        }
        message[pos++] = checksum;

        // Envoi via UART
        serial_.write(message);
        RCLCPP_INFO(this->get_logger(), "Message UART envoyé (Fonction=0x%04X, Taille=%u)", msg_function, total_length);
    }

    // Méthode inspirée d'Apply_Strategy côté STM32
    void apply_strategy(Color teamColor, Zone teamZone, Zone enemyZone)
    {
        Strategy strat;
        strat.color = teamColor;
        strat.teamzone = teamZone;
        strat.enemyzone = enemyZone;

        char msg[50];
        int length = std::sprintf(msg, "STRATEGY:%d:%d:%d", strat.color, strat.teamzone, strat.enemyzone);

        // Conversion de la chaîne en vecteur d'octets
        std::vector<uint8_t> payload(msg, msg + length);

        send_uart_message(UART_CMD_STRATEGY, payload);
    }

private:
    serial::Serial serial_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr strategy_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    rclcpp::TimerBase::SharedPtr strategy_timer_;

    // Protocole
    const uint8_t HEADER = 0x4A;

    // Machine à états pour le décodage
    enum class RcvState
    {
        WAITING,
        FUNCTION_MSB,
        FUNCTION_LSB,
        LENGTH_MSB,
        LENGTH_LSB,
        PAYLOAD,
        CHECKSUM
    };
    RcvState rcv_state_ = RcvState::WAITING;
    uint16_t msg_function_ = 0;
    uint16_t msg_payload_length_ = 0;
    std::vector<uint8_t> msg_payload_;

    // Temps du dernier octet reçu
    Clock::time_point last_byte_time_;

    // Calcul du checksum
    uint8_t calculate_checksum_direct()
    {
        uint8_t checksum = HEADER;
        checksum ^= (msg_function_ >> 8) & 0xFF;
        checksum ^= (msg_function_ & 0xFF);
        checksum ^= (msg_payload_length_ >> 8) & 0xFF;
        checksum ^= (msg_payload_length_ & 0xFF);
        for (auto b : msg_payload_)
        {
            checksum ^= b;
        }
        return checksum;
    }

    // Lecture des données disponibles sur le port série
    void read_uart()
    {
        if (serial_.available())
        {
            std::vector<uint8_t> buffer;
            size_t available_bytes = serial_.available();
            serial_.read(buffer, available_bytes);

            std::ostringstream oss;
            oss << "Octets reçus (" << available_bytes << "): ";
            for (auto byte : buffer)
            {
                oss << "0x" << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<int>(byte) << " ";
                process_byte(byte);
            }
            RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
            last_byte_time_ = Clock::now();
        }
    }

    // Vérifie le timeout de réception (300ms sans octet)
    void check_rx_timeout()
    {
        auto now = Clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_byte_time_).count();
        if (rcv_state_ != RcvState::WAITING && diff > 300)
        {
            RCLCPP_WARN(this->get_logger(), "Timeout de réception (%ld ms), réinitialisation de la machine à états", diff);
            rcv_state_ = RcvState::WAITING;
        }
    }

    // Traitement des octets reçus par machine à états
    void process_byte(uint8_t byte)
    {
        switch (rcv_state_)
        {
        case RcvState::WAITING:
            if (byte == HEADER)
            {
                rcv_state_ = RcvState::FUNCTION_MSB;
                msg_payload_.clear();
                msg_function_ = 0;
                msg_payload_length_ = 0;
                RCLCPP_DEBUG(this->get_logger(), "En-tête détecté (0x4A)");
            }
            break;
        case RcvState::FUNCTION_MSB:
            msg_function_ = byte << 8;
            rcv_state_ = RcvState::FUNCTION_LSB;
            break;
        case RcvState::FUNCTION_LSB:
            msg_function_ |= byte;
            rcv_state_ = RcvState::LENGTH_MSB;
            break;
        case RcvState::LENGTH_MSB:
            msg_payload_length_ = byte << 8;
            rcv_state_ = RcvState::LENGTH_LSB;
            break;
        case RcvState::LENGTH_LSB:
            msg_payload_length_ |= byte;
            rcv_state_ = RcvState::PAYLOAD;
            RCLCPP_DEBUG(this->get_logger(), "Fonction=0x%04X, Longueur payload=%u", msg_function_, msg_payload_length_);
            break;
        case RcvState::PAYLOAD:
            msg_payload_.push_back(byte);
            if (msg_payload_.size() >= msg_payload_length_)
            {
                rcv_state_ = RcvState::CHECKSUM;
            }
            break;
        case RcvState::CHECKSUM:
        {
            uint8_t computed_checksum = calculate_checksum_direct();
            RCLCPP_DEBUG(this->get_logger(), "Checksum calculé=0x%02X, Reçu=0x%02X", computed_checksum, byte);
            if (computed_checksum == byte)
            {
                process_message(msg_function_, msg_payload_);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Erreur checksum pour le message (Fonction=0x%04X)", msg_function_);
            }
            rcv_state_ = RcvState::WAITING;
        }
        break;
        default:
            rcv_state_ = RcvState::WAITING;
            break;
        }
    }

    // Traitement de la trame complète décodée
    void process_message(uint16_t function, const std::vector<uint8_t> &payload)
    {
        RCLCPP_INFO(this->get_logger(), "Message décodé: Fonction=0x%04X, Taille payload=%zu", function, payload.size());
        if (function == UART_CMD_STRATEGY)
        {
            std::string strategy_msg(payload.begin(), payload.end());
            RCLCPP_INFO(this->get_logger(), "STRATEGY reçue: %s", strategy_msg.c_str());
            auto msg = std_msgs::msg::String();
            msg.data = strategy_msg;
            strategy_pub_->publish(msg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Commande inconnue reçue: 0x%04X", function);
        }
    }

    // Callback du timer pour envoyer périodiquement une stratégie
    void apply_strategy_callback()
    {
        // Exemple : envoyer une stratégie avec teamColor=RED, teamZone=ZONE_A et enemyZone=ZONE_B
        apply_strategy(RED, ZONE_A, ZONE_B);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UARTBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
