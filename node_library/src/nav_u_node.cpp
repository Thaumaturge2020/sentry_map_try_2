#include "node_library/nav_u_node.hpp"

namespace BehaviorTree{
    nav_u_node::nav_u_node(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    node1 = rclcpp::Node::make_shared("1919");
                    publisher_decision_pos = node1->create_publisher<robot_msgs::msg::WalkCmd>("decision2pathplan",10);
                }
    BT::NodeStatus nav_u_node::tick()
    {
        RCLCPP_INFO(rclcpp::get_logger("nav_u_node"),"I'm ticked");
        rclcpp::spin_some(node1);
        geometry_msgs::msg::Point point;
        if(getInput<geometry_msgs::msg::Point>("navigation_point",point))
        {
            robot_msgs::msg::WalkCmd my_cmd;
            my_cmd.pos = point;
            my_cmd.opt = 2;
            my_cmd.radium = 2.0;
            publisher_decision_pos ->publish(my_cmd);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::nav_u_node>("NavUNode");
// }