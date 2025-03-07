#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <exception>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

struct Waypoint
{
    double x;
    double y;
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
        // Abonnement pour recevoir START_MATCH sur le topic /match_trigger
        match_trigger_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/match_trigger", 10,
            std::bind(&MatchControlNode::match_trigger_callback, this, std::placeholders::_1));

        // Abonnement pour recevoir la trame STRATEGY sur le topic /strategy
        strategy_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/strategy", 10,
            std::bind(&MatchControlNode::strategy_callback, this, std::placeholders::_1));

        // Publisher pour envoyer STOP_MATCH sur le topic /match_command
        match_command_publisher_ = this->create_publisher<std_msgs::msg::String>("/match_command", 10);

        RCLCPP_INFO(this->get_logger(), "MatchControlNode démarré");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr match_trigger_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr strategy_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr match_command_publisher_;
    rclcpp::TimerBase::SharedPtr match_timer_;
    
    const std::chrono::seconds match_duration_{100};
    Strategy current_strategy_;

    // Callback pour START_MATCH
    void match_trigger_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "START_MATCH")
        {
            RCLCPP_INFO(this->get_logger(), "START_MATCH reçu : démarrage du chronomètre pour %ld secondes", match_duration_.count());
            match_timer_ = this->create_wall_timer(
                match_duration_,
                std::bind(&MatchControlNode::match_timer_callback, this));

            // Lancer l'exécution de la stratégie chargée
        }
    }

    // Callback pour la trame STRATEGY
    void strategy_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string filepath = build_strategy_filepath(msg->data);
        try
        {
            current_strategy_ = load_strategy_from_file(filepath);
            RCLCPP_INFO(this->get_logger(), "Stratégie chargée : %s", current_strategy_.name.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Erreur lors du chargement de la stratégie depuis '%s' : %s", filepath.c_str(), e.what());
        }
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
