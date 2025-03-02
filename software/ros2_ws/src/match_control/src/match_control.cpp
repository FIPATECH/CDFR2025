#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class MatchControlNode : public rclcpp::Node
{
public:
    MatchControlNode() : Node("match_control")
    {
        // Abonnement au topic /match_trigger pour recevoir la commande START_MATCH
        match_trigger_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/match_trigger", 10,
            std::bind(&MatchControlNode::match_trigger_callback, this, std::placeholders::_1));

        // Publisher pour envoyer la commande STOP_MATCH sur le topic /match_command
        match_command_publisher_ = this->create_publisher<std_msgs::msg::String>("/match_command", 10);

        RCLCPP_INFO(this->get_logger(), "MatchControlNode démarré");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr match_trigger_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr match_command_publisher_;
    rclcpp::TimerBase::SharedPtr match_timer_;

    // Durée du match (100 secondes)
    const std::chrono::seconds match_duration_ = std::chrono::seconds(100);

    // Callback exécutée à la réception d'un message sur /match_trigger
    void match_trigger_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "START_MATCH")
        {
            RCLCPP_INFO(this->get_logger(), "START_MATCH reçu : démarrage du chronomètre pour %ld secondes", match_duration_.count());
            // Démarrage d'un timer qui, à expiration, enverra STOP_MATCH
            match_timer_ = this->create_wall_timer(
                match_duration_,
                std::bind(&MatchControlNode::match_timer_callback, this));
        }
    }

    // Callback du timer qui envoie STOP_MATCH à l'expiration de la durée du match
    void match_timer_callback()
    {
        auto stop_msg = std_msgs::msg::String();
        stop_msg.data = "STOP_MATCH";
        match_command_publisher_->publish(stop_msg);
        RCLCPP_INFO(this->get_logger(), "Durée du match écoulée, STOP_MATCH envoyé");
        match_timer_->cancel();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MatchControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
