#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <functional>
#include <cmath>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Conversion degrés -> radians
constexpr double DEG2RAD = M_PI / 180.0;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode()
        : Node("navigation_node"), server_ready_(false)
    {
        // Client d'action Nav2
        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        /* Timer périodique : on essaye toutes les secondes jusqu’à ce que
            le serveur devienne disponible, ensuite on annule ce timer. */
        server_wait_timer_ = create_wall_timer(
            1s, std::bind(&NavigationNode::checkServerReady, this));

        // Subscription aux commandes
        command_sub_ = create_subscription<std_msgs::msg::String>(
            "/action_command", 10,
            std::bind(&NavigationNode::commandCallback, this, std::placeholders::_1));

        // Publisher pour l'ACK final
        ack_pub_ = create_publisher<std_msgs::msg::String>("navigation_status", 10);

        RCLCPP_INFO(get_logger(),
                    "NavigationNode initialisé. Attente de 'navigate_to_pose'...");
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ack_pub_;

    rclcpp::TimerBase::SharedPtr server_wait_timer_;
    bool server_ready_;

    /*--------------------------------------------------------------*/
    /* Timer : vérifie la disponibilité du serveur d’action Nav2    */
    void checkServerReady()
    {
        if (server_ready_)
            return;
        if (action_client_->action_server_is_ready())
        {
            server_ready_ = true;
            server_wait_timer_->cancel();
            RCLCPP_INFO(get_logger(),
                        "'navigate_to_pose' est prêt → commandes MOVE activées.");
        }
    }

    void commandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!server_ready_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(),
                                 2000, "Commande ignorée : Nav2 pas encore prêt.");
            return;
        }
        // on découpe la string par ':'
        std::vector<std::string> parts;
        std::istringstream iss(msg->data);
        std::string token;
        while (std::getline(iss, token, ':'))
        {
            parts.push_back(token);
        }

        // On attend ACTION:MOVE:x:y[:theta]
        if (parts.size() < 4 || parts[0] != "ACTION" || parts[1] != "MOVE")
        {
            RCLCPP_WARN(get_logger(),
                        "Format invalide '%s'. Attendu ACTION:MOVE:<x>:<y>[:<theta_deg>]",
                        msg->data.c_str());
            return;
        }

        double goal_x, goal_y, theta_deg = 0.0;
        try
        {
            double goal_x_cm = std::stod(parts[2]);
            double goal_y_cm = std::stod(parts[3]);
            // conversion cm → m
            goal_x = goal_x_cm / 100.0;
            goal_y = goal_y_cm / 100.0;

            if (parts.size() >= 5)
            {
                theta_deg = std::stod(parts[4]);
            }
            else
            {
                RCLCPP_WARN(get_logger(),
                            "Angle non fourni dans '%s', utilisation de θ=0° par défaut.",
                            msg->data.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(),
                         "Erreur de conversion dans '%s' : %s",
                         msg->data.c_str(), e.what());
            return;
        }

        double theta_rad = theta_deg * DEG2RAD;
        RCLCPP_INFO(get_logger(),
                    "MOVE reçu : x=%.2f, y=%.2f, θ=%.2f° (%.2f rad)",
                    goal_x, goal_y, theta_deg, theta_rad);

        sendNavigationGoal(goal_x, goal_y, theta_rad);
    }

    void sendNavigationGoal(double x, double y, double theta_rad)
    {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        // quaternion pour rotation Z
        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.z = std::sin(theta_rad / 2.0);
        goal_msg.pose.pose.orientation.w = std::cos(theta_rad / 2.0);

        RCLCPP_INFO(get_logger(),
                    "Envoi goal Nav2 : x=%.2f, y=%.2f, θ=%.2f rad",
                    x, y, theta_rad);

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.goal_response_callback =
            std::bind(&NavigationNode::goalResponseCallback, this, std::placeholders::_1);
        options.feedback_callback =
            std::bind(&NavigationNode::feedbackCallback, this,
                      std::placeholders::_1, std::placeholders::_2);
        options.result_callback =
            std::bind(&NavigationNode::resultCallback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, options);
    }

    void goalResponseCallback(std::shared_future<GoalHandle::SharedPtr> future)
    {
        if (!future.get())
        {
            RCLCPP_ERROR(get_logger(), "Le goal a été rejeté par Nav2.");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Goal accepté, exécution en cours...");
        }
    }

    void feedbackCallback(
        GoalHandle::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(),
                    "Feedback Nav2 : distance restante = %.2f",
                    feedback->distance_remaining);
    }

    void resultCallback(
        const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result)
    {
        std_msgs::msg::String ack;
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal atteint avec succès !");
            ack.data = "ACK:MOVE:SUCCEEDED";
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal ABORTED");
            ack.data = "ACK:MOVE:ABORTED";
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(get_logger(), "Goal CANCELED");
            ack.data = "ACK:MOVE:CANCELED";
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Résultat Nav2 inconnu");
            ack.data = "ACK:MOVE:UNKNOWN";
            break;
        }
        ack_pub_->publish(ack);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
