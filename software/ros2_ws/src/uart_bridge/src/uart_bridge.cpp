#include <chrono>
#include <memory>
#include <vector>
#include <sstream>
#include <cstdint>
#include <iomanip>
#include <string>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial/serial.h>
#include "uart_commands.h"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

class UARTBridgeNode : public rclcpp::Node
{
public:
    UARTBridgeNode() : Node("uart_bridge_node")
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
        }
        if (serial_.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "Port série /dev/ttyTHS1 ouvert");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Port série /dev/ttyTHS1 non ouvert");
        }

        strategy_pub_ = this->create_publisher<std_msgs::msg::String>("/strategy", 10);
        match_trigger_pub_ = this->create_publisher<std_msgs::msg::String>("/match_trigger", 10);
        match_command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/match_command", 10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                if (msg->data == "STOP_MATCH") // Fin du chrono
                {
                    RCLCPP_INFO(this->get_logger(), "STOP_MATCH reçu du topic, envoi à STM32");
                    send_uart_message(UART_CMD_STOP_MATCH, {});
                }
            });

        timer_ = this->create_wall_timer(50ms, std::bind(&UARTBridgeNode::read_uart, this));
        timeout_timer_ = this->create_wall_timer(300ms, std::bind(&UARTBridgeNode::check_rx_timeout, this));
        ping_timer_ = this->create_wall_timer(5s, std::bind(&UARTBridgeNode::send_ping_callback, this));
        text_timer_ = this->create_wall_timer(7s, std::bind(&UARTBridgeNode::send_hello_stm_callback, this));
        reconnect_timer_ = this->create_wall_timer(1s, std::bind(&UARTBridgeNode::attempt_serial_reconnect, this));

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

        // Construction de la trame
        message[pos++] = HEADER;
        message[pos++] = (msg_function >> 8) & 0xFF;
        message[pos++] = msg_function & 0xFF;
        message[pos++] = (payload_length >> 8) & 0xFF;
        message[pos++] = payload_length & 0xFF;
        for (auto byte : payload)
        {
            message[pos++] = byte;
        }

        // Calcul du checksum
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
        try
        {
            serial_.write(message);
            RCLCPP_INFO(this->get_logger(), "Message UART envoyé (Fonction=0x%04X, Taille=%u)", msg_function, total_length);
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Erreur lors de l'envoi: %s", e.what());
            if (serial_.isOpen())
            {
                serial_.close();
            }
        }
    }

    void send_ping_callback()
    {
        send_uart_message(UART_CMD_PING, {});
        RCLCPP_INFO(this->get_logger(), "PING envoyé");
    }

    void send_hello_stm_callback()
    {
        send_text("Hello STM!");
        RCLCPP_INFO(this->get_logger(), "Texte 'Hello STM!' envoyé");
    }

    void send_text(const std::string &text)
    {
        std::vector<uint8_t> payload(text.begin(), text.end());
        send_uart_message(UART_CMD_TEXT, payload);
    }

private:
    serial::Serial serial_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr strategy_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr match_trigger_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr match_command_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    rclcpp::TimerBase::SharedPtr ping_timer_;
    rclcpp::TimerBase::SharedPtr text_timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;

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

    Clock::time_point last_byte_time_;

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

    void read_uart()
    {
        try
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
                // RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
                last_byte_time_ = Clock::now();
            }
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Erreur de lecture UART: %s", e.what());
            if (serial_.isOpen())
            {
                serial_.close();
            }
        }
    }

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
            rcv_state_ = (msg_payload_length_ == 0) ? RcvState::CHECKSUM : RcvState::PAYLOAD;
            // RCLCPP_DEBUG(this->get_logger(), "Fonction=0x%04X, Longueur payload=%u", msg_function_, msg_payload_length_);
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
            // RCLCPP_DEBUG(this->get_logger(), "Checksum calculé=0x%02X, Reçu=0x%02X", computed_checksum, byte);
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
        switch (function)
        {
        case UART_CMD_STRATEGY:
        {
            std::string strategy_msg(payload.begin(), payload.end());
            RCLCPP_INFO(this->get_logger(), "STRATEGY reçue: %s", strategy_msg.c_str());
            auto msg = std_msgs::msg::String();
            msg.data = strategy_msg;
            strategy_pub_->publish(msg);
            break;
        }
        case UART_CMD_PING:
        {
            RCLCPP_INFO(this->get_logger(), "PING reçu");
            send_uart_message(UART_CMD_PONG, {});
            break;
        }
        case UART_CMD_PONG:
        {
            RCLCPP_INFO(this->get_logger(), "PONG reçu");
            break;
        }
        case UART_CMD_TEXT:
        {
            std::string text(payload.begin(), payload.end());
            RCLCPP_INFO(this->get_logger(), "Texte reçu: %s", text.c_str());
            break;
        }
        case UART_CMD_START_MATCH:
        {
            RCLCPP_INFO(this->get_logger(), "START_MATCH reçu");
            auto msg = std_msgs::msg::String();
            msg.data = "START_MATCH";
            match_trigger_pub_->publish(msg);
            break;
        }
        default:
            RCLCPP_WARN(this->get_logger(), "Commande inconnue reçue: 0x%04X", function);
            break;
        }
    }

    // Fonction de reconnexion du port
    void attempt_serial_reconnect()
    {
        if (!serial_.isOpen())
        {
            try
            {
                serial_.open();
                if (serial_.isOpen())
                {
                    RCLCPP_INFO(this->get_logger(), "Reconnexion réussie au port série");
                }
            }
            catch (serial::IOException &e)
            {
                RCLCPP_WARN(this->get_logger(), "Tentative de reconnexion échouée : %s", e.what());
            }
        }
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
