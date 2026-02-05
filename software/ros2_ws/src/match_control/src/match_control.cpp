#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <exception>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

struct Waypoint
{
    double x;
    double y;
    double theta;
    std::string action;
};

struct Strategy
{
    std::string id;
    std::string name;
    std::string team_color;
    std::string team_zone;
    std::string enemy_zone;
    std::vector<Waypoint> waypoints;
};

Strategy load_strategy_from_file(const std::string &filepath)
{
    Strategy strat;
    YAML::Node config = YAML::LoadFile(filepath);
    strat.id = config["id"].as<std::string>();
    strat.name = config["name"].as<std::string>();
    strat.team_color = config["team_color"].as<std::string>();
    strat.team_zone = config["team_zone"].as<std::string>();
    strat.enemy_zone = config["enemy_zone"].as<std::string>();

    if (config["waypoints"])
    {
        for (const auto &wp : config["waypoints"])
        {
            Waypoint waypoint;
            waypoint.x = wp["x"].as<double>();
            waypoint.y = wp["y"].as<double>();
            waypoint.theta = wp["theta"].as<double>();
            waypoint.action = wp["action"].as<std::string>();
            strat.waypoints.push_back(waypoint);
        }
    }
    return strat;
}

// Construction du chemin du fichier YAML à partir de la trame de stratégie
std::string build_strategy_filepath(const std::string &strategy_msg)
{
    std::istringstream iss(strategy_msg);
    std::string token, team_color, team_zone, enemy_zone;

    // Découpage de la trame
    std::getline(iss, token, ':');
    std::getline(iss, team_color, ':');
    std::getline(iss, team_zone, ':');
    std::getline(iss, enemy_zone, ':');

    std::string package_share_dir = ament_index_cpp::get_package_share_directory("match_control");
    std::ostringstream oss;
    oss << package_share_dir << "/config/strategies/strategy_"
        << team_color << "_" << team_zone << "_" << enemy_zone << ".yaml";
    return oss.str();
}

class MatchControlNode : public rclcpp::Node
{
public:
    MatchControlNode() : Node("match_control_node")
    {
        // Souscription au topic /match_trigger pour recevoir "START_MATCH"
        match_trigger_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/match_trigger", 10,
            std::bind(&MatchControlNode::match_trigger_callback, this, std::placeholders::_1));

        // Souscription au topic /strategy pour recevoir la trame de stratégie
        strategy_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/strategy", 10,
            std::bind(&MatchControlNode::strategy_callback, this, std::placeholders::_1));

        // Publisher pour envoyer STOP_MATCH sur /match_command à la fin du match
        match_command_publisher_ = this->create_publisher<std_msgs::msg::String>("/match_command", 10);

        // Publisher pour envoyer des commandes d'action vers la STM32 et au noeud de navigation)
        action_command_publisher_ = this->create_publisher<std_msgs::msg::String>("/action_command", 10);

        RCLCPP_INFO(this->get_logger(), "MatchControlNode démarré");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr match_trigger_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr strategy_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr match_command_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_command_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_status_subscriber_;
    rclcpp::TimerBase::SharedPtr match_timer_;

    const std::chrono::seconds match_duration_{98};
    Strategy current_strategy_;

    // Flag pour la synchronisation avec l'ACK de navigation
    bool nav_ack_received_ = false;

    // Callback pour la réception de START_MATCH
    void match_trigger_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "START_MATCH")
        {
            RCLCPP_INFO(this->get_logger(), "START_MATCH reçu : démarrage du match pour %ld secondes", match_duration_.count());
            // Démarrer le timer du match
            match_timer_ = this->create_wall_timer(
                match_duration_,
                std::bind(&MatchControlNode::match_timer_callback, this));

            // Souscription au topic navigation_status pour recevoir les ACK de navigation
            nav_status_subscriber_ = this->create_subscription<std_msgs::msg::String>(
                "navigation_status", 10,
                std::bind(&MatchControlNode::navStatusCallback, this, std::placeholders::_1));

            // Exécuter la stratégie dans un thread séparé
            std::thread strategy_thread(&MatchControlNode::execute_strategy, this, current_strategy_);
            strategy_thread.detach();
        }
    }

    // Callback pour la réception de la trame STRATEGY
    void strategy_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string filepath = build_strategy_filepath(msg->data);
        try
        {
            current_strategy_ = load_strategy_from_file(filepath);
            RCLCPP_INFO(this->get_logger(), "Stratégie chargée : %s", current_strategy_.name.c_str());
            // Afficher les waypoints et les actions
            for (const auto &wp : current_strategy_.waypoints)
            {
                RCLCPP_INFO(this->get_logger(), "Waypoint: x=%.2f, y=%.2f, action=%s", wp.x, wp.y, wp.action.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Erreur lors du chargement de la stratégie depuis '%s' : %s", filepath.c_str(), e.what());
        }
    }

    // Callback pour le topic navigation_status qui reçoit l'ACK de navigation
    void navStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "ACK de navigation reçu: %s", msg->data.c_str());
        if (msg->data.find("ACK:MOVE:") != std::string::npos)
        {
            nav_ack_received_ = true;
        }
    }

    // Fonction pour exécuter la stratégie
    void execute_strategy(const Strategy &strat)
    {
        // Envoie une commande d'action, pour chaque waypoint de la stratégie
        // La commande est de la forme "ACTION:<action>:<x>:<y>:<theta_deg>"
        for (const auto &wp : strat.waypoints)
        {
            // Construire la commande d'action
            std::ostringstream oss;
            oss << "ACTION:" << wp.action << ":" << wp.x << ":" << wp.y << ":" << wp.theta;
            std_msgs::msg::String action_msg;
            action_msg.data = oss.str();

            // Publier la commande
            action_command_publisher_->publish(action_msg);
            RCLCPP_INFO(this->get_logger(), "Commande d'action envoyée : %s", action_msg.data.c_str());

            // Attendre que le flag ACK soit reçu
            nav_ack_received_ = false;
            auto start_time = this->now();
            while (!nav_ack_received_)
            {
                if ((this->now() - start_time).seconds() > 10.0)
                {
                    RCLCPP_WARN(this->get_logger(), "Timeout d'attente de l'ACK pour le waypoint : x=%.2f, y=%.2f", wp.x, wp.y);
                    break;
                }
                rclcpp::sleep_for(100ms);
            }
            // Simuler un délai d'exécution avant de passer au waypoint suivant

            rclcpp::sleep_for(2s);
        }
        RCLCPP_INFO(this->get_logger(), "Exécution de la stratégie terminée");
    }

    // Callback du timer pour envoyer STOP_MATCH à l'expiration du chrono
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
